/*
 * QEMU emulation of an RISC-V IOMMU (Ziommu)
 *
 * Copyright (C) 2022 Rivos Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qom/object.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "hw/qdev-properties.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/riscv_iommu.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/error-report.h"

#include "trace.h"

/* Rivos Inc. PCIe Device Emulation of RISC-V I/O Management Unit  */

#ifndef PCI_VENDOR_ID_RIVOS
#define PCI_VENDOR_ID_RIVOS           0x1efd
#endif

#ifndef PCI_DEVICE_ID_RIVOS_IOMMU
#define PCI_DEVICE_ID_RIVOS_IOMMU     0x8001
#endif

/* MSI Control Registers */
#define RIO_REG_MSI_ADDR_BASE   (RIO_REG_SIZE)
#define RIO_REG_MSI_PBA_BASE    (RIO_REG_SIZE + 256)

/* Supported S/G Stage translation modes. */
#define RIO_CAP_S_ANY          (RIO_CAP_S_SV32 | RIO_CAP_S_SV39 | \
                                RIO_CAP_S_SV48 | RIO_CAP_S_SV57)
#define RIO_CAP_G_ANY          (RIO_CAP_G_SV32 | RIO_CAP_G_SV39 | \
                                RIO_CAP_G_SV48 | RIO_CAP_G_SV57)

/* Physical page number coversions */
#define PPN_PHYS(ppn)                 ((ppn) << TARGET_PAGE_BITS)
#define PPN_DOWN(phy)                 ((phy) >> TARGET_PAGE_BITS)

/* PASID Consts */
#define PASID_NONE (~0U)

/* Core logic actions (core_exec bit location) */
enum {
    RIO_EXEC_EXIT,
    RIO_EXEC_DDTP,
    RIO_EXEC_CQ_CONTROL,
    RIO_EXEC_CQ_DB,
    RIO_EXEC_FQ_CONTROL,
    RIO_EXEC_FQ_DB,
    RIO_EXEC_PQ_CONTROL,
    RIO_EXEC_PQ_DB,
    RIO_EXEC_LAST,
};

/* Internal IOMMU Fault codes */
enum {
    RIO_FAULT_NONE = 0,                 /* success, action completed */
    RIO_FAULT_PASS = 1,                 /* success, action ignored */
    RIO_FAULT_BASE = 0x100000,          /* base value for fault codes */

    RIO_FAULT_DMA_DISABLED = RIO_FAULT_BASE + 256,
    RIO_FAULT_RID_INVALID,              /* Invalid requestor id */
    RIO_FAULT_MSIPTE_LOAD,              /* MSI PTE access fault */
    RIO_FAULT_MSIPTE_INVALID,           /* Invalid MSI PTE Content */
    RIO_FAULT_DDT_FAULT,                /* Device directory access fault */
    RIO_FAULT_DDT_INVALID,              /* Invalid device directory entry */
    RIO_FAULT_DDT_UNSUPPORTED,          /* Incorrect DDTP mode filed */
    RIO_FAULT_MSI_INVALID,              /* Invalid interrupt index number */
    RIO_FAULT_PTE_LOAD,                 /* PTE access fault */
    RIO_FAULT_PTE_INVALID,              /* PTE invalid or incorrect flags */
    RIO_FAULT_PTE_GFAULT_RD,            /* PTE_R not set for G-Stage */
    RIO_FAULT_PTE_GFAULT_WR,            /* PTE_W not set for G-Stage */
    RIO_FAULT_PTE_SFAULT_RD,            /* PTE_R not set for S-Stage */
    RIO_FAULT_PTE_SFAULT_WR,            /* PTE_W not set for S-Stage */
    RIO_FAULT_PASID,                    /* PASID invalid or disabled */
};

/* private: translation context data */
typedef struct RISCVIOMMUContext RISCVIOMMUContext;
typedef struct RISCVIOMMUState RISCVIOMMUState;

struct RISCVIOMMUState {
    uint8_t regs_rw[RIO_REG_SIZE];  /* MMIO register state */
    uint8_t regs_wc[RIO_REG_SIZE];  /* MMIO write-1-to-clear */
    uint8_t regs_ro[RIO_REG_SIZE];  /* MMIO read/only mask */

    uint32_t devid;       /* IOMMU requester Id, 0 if not assigned. */
    uint32_t version;     /* Reported interface version number */
    bool enable_off;      /* Enable out-of-reset OFF mode (DMA disabled) */
    bool enable_msi;      /* Enable MSI / FLAT PAGE remapping */
    bool enable_ats;      /* Enable ATS support */
    bool enable_s_stage;  /* Enable S/VS-Stage translation */
    bool enable_g_stage;  /* Enable G-Stage translation */

    uint64_t ddtp;        /* Validated Device Directory Tree Root Pointer */
    uint32_t cq_head;     /* Command queue head index */
    uint32_t cq_mask;     /* Command queue index bitmask */
    dma_addr_t cq;        /* Command queue pointer */
    uint32_t fq_tail;     /* Fault/event queue tail index */
    uint32_t fq_mask;     /* Fault/event queue index bitmask */
    dma_addr_t fq;        /* Fault/event queue pointer */
    uint32_t pq_tail;     /* Page request queue tail index */
    uint32_t pq_mask;     /* Page request queue index bitmask */
    dma_addr_t pq;        /* Page request queue pointer */

    QemuThread core_proc; /* Background processing thread */
    QemuCond core_cond;   /* Background processing wakeup signal */
    QemuMutex core_lock;  /* Global IOMMU lock, used for cache/regs updates */
    unsigned core_exec;   /* Processing thread execution actions */

    /* interrupt delivery callback */
    void (*notify)(RISCVIOMMUState *iommu, unsigned vector);

    QLIST_HEAD(, RISCVIOMMUSpace) spaces;
};

struct RISCVIOMMUSpace {
    IOMMUMemoryRegion mr;          /* IOVA memory region */
    AddressSpace as;               /* IOVA address space */
    RISCVIOMMUState *iommu;        /* Managing IOMMU device */
    RISCVIOMMUContext *cache;      /* Cached translation data, RCU synch */
    uint32_t devid;                /* Device identifier, requestor-id */

    QLIST_ENTRY(RISCVIOMMUSpace) list;
};

/* Translation context, RCU protected. */
struct RISCVIOMMUContext {
    struct rcu_head rcu;
    RISCVIOMMUDeviceContext dc;    /* latest device context copy */
    RISCVIOMMUProcessContext pc;   /* latest process context copy */
    uint32_t pasid;                /* cached process ID */
};

/* Helper functions */
static uint32_t riscv_iommu_reg_mod(RISCVIOMMUState *s,
    unsigned idx, uint32_t set, uint32_t clr)
{
    uint32_t val;
    qemu_mutex_lock(&s->core_lock);
    val = ldl_le_p(&s->regs_rw[idx]);
    stl_le_p(&s->regs_rw[idx], set | (val & ~clr));
    qemu_mutex_unlock(&s->core_lock);
    return val;
}

static unsigned riscv_iommu_irq_vector(RISCVIOMMUState *s, int source)
{
    const uint32_t ivec = ldl_le_p(&s->regs_rw[RIO_REG_IVEC]);
    return (ivec >> (source * 4)) & 0x0F;
}

static void riscv_iommu_irq_assert(RISCVIOMMUState *s, int source)
{
    uint32_t ipsr = riscv_iommu_reg_mod(s, RIO_REG_IPSR, (1 << source), 0);

    if (s->notify &&  !(ipsr & (1 << source))) {
        s->notify(s, riscv_iommu_irq_vector(s, source));
    }
}

static void riscv_iommu_post_event(RISCVIOMMUSpace *as, RISCVIOMMUEvent *ev)
{
    RISCVIOMMUState *s = as->iommu;
    uint32_t head = ldl_le_p(&s->regs_rw[RIO_REG_FQ_HEAD]) & s->fq_mask;
    uint32_t next = (s->fq_tail + 1) & s->fq_mask;
    uint32_t ctrl = ldl_le_p(&s->regs_rw[RIO_REG_FQ_CONTROL]);
    uint32_t ctrl_err = 0;

    trace_riscv_iommu_flt(PCI_BUS_NUM(as->devid), PCI_SLOT(as->devid),
                          PCI_FUNC(as->devid), ev->reason, ev->iova);

    if (!(ctrl & RIO_FQ_ACTIVE) || !!(ctrl & (RIO_FQ_FULL | RIO_FQ_FAULT))) {
        return;
    }

    if (head == next) {
        ctrl_err = RIO_FQ_FULL;
    } else {
        dma_addr_t addr = s->fq + s->fq_tail * sizeof(RISCVIOMMUEvent);
        if (dma_memory_write(&address_space_memory, addr, ev, sizeof(*ev),
                             MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            ctrl_err = RIO_FQ_FAULT;
        } else {
            s->fq_tail = next;
        }
    }

    stl_le_p(&s->regs_rw[RIO_REG_FQ_TAIL], s->fq_tail);

    if (ctrl_err) {
        riscv_iommu_reg_mod(s, RIO_REG_CQ_CONTROL, ctrl_err, 0);
    }

    if (ctrl & RIO_FQ_IE) {
        riscv_iommu_irq_assert(s, RIO_INT_FQ);
    }
}

/*
 * RISCV IOMMU Address Translation Lookup - Page Table Walk
 *
 * Note: Code is based on get_physical_address() from target/riscv/cpu_helper.c
 * Both implementation can be merged into single helper function in future.
 * Keeping them separate for now, as error reporting and flow specifics are
 * sufficiently different for separate implementation.
 */
static int riscv_iommu_fetch_pa(hwaddr addr, bool s_stage,
        hwaddr *physical, hwaddr *mask, uint64_t gatp, uint64_t satp,
        IOMMUAccessFlags access)
{
    int i, levels, ptidxbits, ptshift, ptesize, mode, widened;
    hwaddr base;

    if (s_stage) {
        int fault = riscv_iommu_fetch_pa(satp, false, &base, NULL,
                                         gatp, satp, access);
        if (fault) {
            return fault;
        }
        mode = get_field(base, RIO_ATP_MASK_MODE);
        base = PPN_PHYS(get_field(base, RIO_ATP_MASK_PPN));
    } else {
        mode = get_field(gatp, RIO_ATP_MASK_MODE);
        base = PPN_PHYS(get_field(gatp, RIO_ATP_MASK_PPN));
    }

    switch (mode) {
    case RIO_ATP_MODE_SV32:
        levels = 2;
        ptidxbits = 10;
        ptesize = 4;
        break;
    case RIO_ATP_MODE_SV39:
        levels = 3;
        ptidxbits = 9;
        ptesize = 8;
        break;
    case RIO_ATP_MODE_SV48:
        levels = 4;
        ptidxbits = 9;
        ptesize = 8;
        break;
    case RIO_ATP_MODE_SV57:
        levels = 5;
        ptidxbits = 9;
        ptesize = 8;
        break;
    case RIO_ATP_MODE_BARE:
        if (s_stage) {
            return riscv_iommu_fetch_pa(addr, false, physical, mask,
                                        gatp, satp, access);
        }
        *physical = addr;
        return 0;
    default:
        return RIO_FAULT_DDT_UNSUPPORTED;
    }

    widened = s_stage ? 0 : 2;
    ptshift = (levels - 1) * ptidxbits;

    /* zero extended address range check */
    int va_bits = TARGET_PAGE_BITS + levels * ptidxbits + widened;
    uint64_t va_mask = (1ULL << va_bits) - 1;
    if ((addr & va_mask) != addr) {
        return RIO_FAULT_DMA_DISABLED;
    }

    for (i = 0; i < levels; i++, ptshift -= ptidxbits) {
        uint64_t pte;
        unsigned idx;
        hwaddr pte_addr;

        idx = (addr >> (TARGET_PAGE_BITS + ptshift)) &
              ((1 << (ptidxbits + widened)) - 1);
        pte_addr = base + idx * ptesize;
        widened = 0;

        if (dma_memory_read(&address_space_memory, pte_addr, &pte, ptesize,
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            return RIO_FAULT_PTE_LOAD;
        }

        if (ptesize == 4) {
            pte = (uint64_t) le32_to_cpu(*((uint32_t *)&pte));
        } else {
            pte = le64_to_cpu(pte);
        }

        hwaddr ppn = pte >> PTE_PPN_SHIFT;

        if (!(pte & PTE_V)) {
            /* Invalid PTE */
            return RIO_FAULT_PTE_INVALID;
        } else if (!(pte & (PTE_R | PTE_W | PTE_X))) {
            /* Inner PTE, continue walking */
            base = PPN_PHYS(ppn);
        } else if ((pte & (PTE_R | PTE_W | PTE_X)) == PTE_W) {
            /* Reserved leaf PTE flags: PTE_W */
            return RIO_FAULT_PTE_INVALID;
        } else if ((pte & (PTE_R | PTE_W | PTE_X)) == (PTE_W | PTE_X)) {
            /* Reserved leaf PTE flags: PTE_W + PTE_X */
            return RIO_FAULT_PTE_INVALID;
        } else if (ppn & ((1ULL << ptshift) - 1)) {
            /* Misaligned PPN */
            return RIO_FAULT_PTE_INVALID;
        } else if ((access & IOMMU_RO) && !(pte & PTE_R)) {
            /* Read access check failed */
            return s_stage ? RIO_FAULT_PTE_SFAULT_RD
                           : RIO_FAULT_PTE_GFAULT_RD;
        } else if ((access & IOMMU_WO) && !(pte & PTE_W)) {
            /* Write access check failed */
            return s_stage ? RIO_FAULT_PTE_SFAULT_WR
                           : RIO_FAULT_PTE_GFAULT_WR;
        } else {
            /* Leaf PTE, update base to translated address. */
            target_ulong vpn = PPN_DOWN(addr);
            base = PPN_PHYS((ppn | (vpn & ((1L << ptshift) - 1)))) |
                   (addr & ~TARGET_PAGE_MASK);
        }

        /* Do the second stage translation if enabled. */
        if (s_stage) {
            hwaddr spa;
            int err = riscv_iommu_fetch_pa(base, false, &spa, mask,
                                     gatp, satp, access);
            if (err) {
                /* Report back GPA causing G-Stage translation fault. */
                *physical = base;
                return err;
            }

            base = spa;
        }

        if (pte & (PTE_R | PTE_W | PTE_X)) {
            /* Leaf PTE, return translated address */
            *physical = base;
            if (mask) {
                *mask &= (1ULL << (TARGET_PAGE_BITS + ptshift)) - 1;
            }
            return RIO_FAULT_NONE;
        }
    }

    return RIO_FAULT_PTE_INVALID;
}

/* RISC-V IOMMU Device Context Loopkup - Device Directory Tree Walk */
static int riscv_iommu_fetch_dc(uint32_t devid, uint64_t ddtp,
        bool enable_ir, RISCVIOMMUDeviceContext *dc)
{
    hwaddr addr = PPN_PHYS(get_field(ddtp, RIO_DDTP_MASK_PPN));
    const bool dcbase = !enable_ir;
    unsigned depth = RIO_DDTP_MODE_1LVL - get_field(ddtp, RIO_DDTP_MASK_MODE);
    uint64_t dde;

    if (depth > 2) {
        /* this should never happen */
        return RIO_FAULT_DDT_UNSUPPORTED;
    }

    /* Check supported device id range. */
    if (devid >= (1 << (depth * 9 + 6 + (dcbase && depth != 2)))) {
        return RIO_FAULT_RID_INVALID;
    }

    for (; depth-- > 0; ) {
        const int split = depth * 9 + 6 + dcbase;
        addr |= ((devid >> split) << 3) & ~TARGET_PAGE_MASK;
        if (dma_memory_read(&address_space_memory, addr, &dde, sizeof(dde),
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            return RIO_FAULT_DDT_FAULT;
        }
        le64_to_cpus(&dde);
        if (!(dde & RIO_DCTC_VALID)) {
            return RIO_FAULT_DDT_INVALID;
        }
        /* TODO: check reserved bits, fault 259 */
        addr = dde & RIO_DDTE_MASK_PPN;
    }

    /* index into device context entry page */
    const size_t dcsize = sizeof(*dc) >> dcbase;
    addr |= (devid * dcsize) & ~TARGET_PAGE_MASK;

    memset(dc, 0, sizeof(*dc));
    if (dma_memory_read(&address_space_memory, addr, dc, dcsize,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return RIO_FAULT_DDT_FAULT;
    }

    le64_to_cpus(&dc->tc);
    le64_to_cpus(&dc->fsc);
    le64_to_cpus(&dc->gatp);
    le64_to_cpus(&dc->ta);
    le64_to_cpus(&dc->msiptp);
    le64_to_cpus(&dc->msi_addr_mask);
    le64_to_cpus(&dc->msi_addr_pattern);

    if (!(dc->tc & RIO_DCTC_VALID)) {
        return RIO_FAULT_DDT_INVALID;
    }

    /* TODO: check reserved bits, error 259 */

    return RIO_FAULT_NONE;
}

/* Portable implementation of pext_u64, bit-mask extraction. */
static uint64_t _pext_u64(uint64_t val, uint64_t ext)
{
    uint64_t ret = 0;
    uint64_t rot = 1;

    while (ext) {
        if (ext & 1) {
            if (val & 1) {
                ret |= rot;
            }
            rot <<= 1;
        }
        val >>= 1;
        ext >>= 1;
    }

    return ret;
}

/*
 * Check and translate IOVA if within MSI remapping range.
 * Returns:
 *  RIO_FAULT_NONE for successful MSI translation,
 *  RIO_FAULT_PASS if IOVA is not an MSI
 *  RIO_FAULT_* for other translation faults.
 */
static int riscv_iommu_check_msi(RISCVIOMMUContext *ctx, IOMMUTLBEntry *tlb)
{
    RISCVIOMMUDeviceContext *dc = &ctx->dc;
    uint64_t intn;
    uint64_t pte;
    int mode = get_field(dc->msiptp, RIO_DCMSI_MASK_MODE);
    hwaddr addr = PPN_PHYS(get_field(dc->msiptp, RIO_DCMSI_MASK_PPN));

    if (!(tlb->perm & IOMMU_WO)) {
        /* only explicit write translations requests are considered as MSI. */
        return RIO_FAULT_PASS;
    }

    if (mode != RIO_DCMSI_MODE_FLAT) {
        /* TODO: clarify error reporting for incorrect DCMSI.MODE config */
        return RIO_FAULT_PASS;
    }

    intn = PPN_DOWN(tlb->iova);
    if ((intn ^ dc->msi_addr_pattern) & ~dc->msi_addr_mask) {
        /* IOVA not in MSI range defined by AIA IMSIC rules. */
        return RIO_FAULT_PASS;
    }

    /* Interrupt File Number */
    intn = _pext_u64(intn, dc->msi_addr_mask);

    if (intn >= 256) {
        /* Interrupt file number out of range (exceeding single page size) */
        return RIO_FAULT_MSI_INVALID;
    }

    /* fetch MSI PTE */
    addr |= intn * 16;
    if (dma_memory_read(&address_space_memory, addr, &pte, sizeof(pte),
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return RIO_FAULT_MSIPTE_LOAD;
    }

    le64_to_cpus(&pte);

    if ((pte & (RIO_MSIPTE_V | RIO_MSIPTE_W | RIO_MSIPTE_C)) !=
        (RIO_MSIPTE_V | RIO_MSIPTE_W)) {
        /* TODO: add check for reserved bits */
        return RIO_FAULT_MSIPTE_INVALID;
    }

    tlb->translated_addr = PPN_PHYS(get_field(pte, RIO_MSIPTE_MASK_PPN));
    tlb->addr_mask = ~TARGET_PAGE_MASK;

    return RIO_FAULT_NONE;
}

static int riscv_iommu_check_ioatc(RISCVIOMMUContext *ctx, IOMMUTLBEntry *tlb)
{
    /* TODO: Merge IOATC */

    /* No entry found */
    return RIO_FAULT_PASS;
}

static void riscv_iommu_update_ioatc(RISCVIOMMUContext *ctx, IOMMUTLBEntry *tlb)
{
    /* TODO: Merge IOATC */
}

/* Returned pointer protected by RCU lock (caller responsibility). */
static int riscv_iommu_get_ctx(RISCVIOMMUSpace *as, uint64_t ddtp,
                               uint32_t pasid, RISCVIOMMUContext **ctxp)
{
    RISCVIOMMUContext *ctx, *old;
    int fault;

    /* TODO: Merge PDT/PASID */
    if (pasid != PASID_NONE) {
        return RIO_FAULT_PASID;
    }

    ctx = qatomic_rcu_read(&as->cache);
    if (!ctx) {
        ctx = g_new0(RISCVIOMMUContext, 1);
        fault = riscv_iommu_fetch_dc(as->devid, ddtp,
                                     as->iommu->enable_msi, &ctx->dc);
        if (fault) {
            g_free(ctx);
            return fault;
        }

        /* TODO: Merge PDT/PASID */

        /* TODO: check allowed translation modes for S/G stages */

        old = qatomic_xchg(&as->cache, ctx);
        if (old) {
            g_free_rcu(old, rcu);
        }
    }

    *ctxp = ctx;
    return RIO_FAULT_NONE;
}

static int riscv_iommu_translate(RISCVIOMMUSpace *as, IOMMUTLBEntry *iotlb)
{
    RISCVIOMMUContext *ctx = NULL;
    uint32_t pasid = PASID_NONE;
    uint64_t ddtp;
    uint64_t satp = 0;
    uint64_t gatp = 0;
    unsigned mode;
    int fault;
    bool enable_faults = true;
    bool enable_pasid = false;

    RCU_READ_LOCK_GUARD();

    ddtp = qatomic_rcu_read(&as->iommu->ddtp);
    mode = get_field(ddtp, RIO_DDTP_MASK_MODE);

    if (mode == RIO_DDTP_MODE_OFF) {
        /* All DMA and translations are disabled. */
        iotlb->perm = IOMMU_NONE;
        return RIO_FAULT_DMA_DISABLED;
    } else if (mode == RIO_DDTP_MODE_BARE) {
        /* Global passthrough mode enabled for all devices. Map 4K page. */
        iotlb->translated_addr = iotlb->iova;
        iotlb->addr_mask = ~TARGET_PAGE_MASK;
        return RIO_FAULT_NONE;
    }

    /* Fetch latest translation context, use cached version if possible. */
    fault = riscv_iommu_get_ctx(as, ddtp, pasid, &ctx);
    if (fault != RIO_FAULT_NONE) {
        goto done;
    }

    enable_faults = !(ctx->dc.tc & RIO_DCTC_DTF);
    enable_pasid = (ctx->dc.tc & RIO_DCTC_PDTV) && (ctx->pc.ta & RIO_PCTA_V);

    if (iotlb->perm == IOMMU_NONE && !(as->iommu->enable_ats &&
                                      (ctx->dc.tc & RIO_DCTC_EN_ATS))) {
        /* ATS request while ATS is disabled */
        fault = RIO_FAULT_DMA_DISABLED;
        goto done;
    }

    /* Check if IOVA is a MSI Interrupt File address. */
    fault = riscv_iommu_check_msi(ctx, iotlb);
    if (fault != RIO_FAULT_PASS) {
        goto done;
    }

    /* Check IOATC */
    fault = riscv_iommu_check_ioatc(ctx, iotlb);
    if (fault != RIO_FAULT_PASS) {
        goto done;
    }

    if (enable_pasid) {
        satp = ctx->pc.fsc;
    } else {
        satp = ctx->dc.fsc;
    }

    if (as->iommu->enable_g_stage) {
        gatp = ctx->dc.gatp;
    } else {
        gatp = set_field(0, RIO_ATP_MASK_MODE, RIO_ATP_MODE_BARE);
    }

    /* Translate using device directory / page table information. */
    fault = riscv_iommu_fetch_pa(iotlb->iova, as->iommu->enable_s_stage,
                &iotlb->translated_addr, &iotlb->addr_mask,
                gatp, satp, iotlb->perm);

    /* Update IOATC */
    if (!fault) {
        riscv_iommu_update_ioatc(ctx, iotlb);
    }

done:
    if (enable_faults && fault >= RIO_FAULT_BASE) {
        RISCVIOMMUEvent ev;
        const unsigned cause = fault - RIO_FAULT_BASE;
        const unsigned ttype = (iotlb->perm & IOMMU_RW) ? RIO_TTYP_UWR :
                ((iotlb->perm & IOMMU_RO) ? RIO_TTYP_URD : RIO_TTYP_ATS);
        ev.reason = set_field(as->devid, RIO_EVENT_MASK_CAUSE, cause);
        ev.reason = set_field(ev.reason, RIO_EVENT_MASK_TTYPE, ttype);
        ev.reason = set_field(ev.reason, RIO_EVENT_PV, enable_pasid);
        ev.reason = set_field(ev.reason, RIO_EVENT_MASK_PID, pasid);
        ev.iova   = iotlb->iova;
        ev.phys   = iotlb->translated_addr;
        ev._rsrvd = 0;
        riscv_iommu_post_event(as, &ev);
    }

    return fault;
}

/* IOMMU Command Interface */

static void riscv_iommu_iodir_inval_ddt(RISCVIOMMUState *s, bool all,
    uint32_t devid)
{
    RISCVIOMMUSpace *as;
    RISCVIOMMUContext *old;

    qemu_mutex_lock(&s->core_lock);
    QLIST_FOREACH(as, &s->spaces, list) {
        if (all || (as->devid == devid)) {
            old = qatomic_xchg(&as->cache, NULL);
            if (old) {
                g_free_rcu(old, rcu);
            }
        }
    }
    qemu_mutex_unlock(&s->core_lock);
}

static void riscv_iommu_iofence(RISCVIOMMUState *s, bool av, uint64_t addr,
    uint32_t data)
{
    if (av) {
        if (dma_memory_write(&address_space_memory, addr, &data, sizeof(data),
                             MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            riscv_iommu_reg_mod(s, RIO_REG_CQ_CONTROL, RIO_CQ_FAULT, 0);
        }
    }
}

static void riscv_iommu_process_cq_db(RISCVIOMMUState *s)
{
    RISCVIOMMUCommand cmd;
    MemTxResult res;
    dma_addr_t addr;
    MemTxAttrs ma = MEMTXATTRS_UNSPECIFIED;
    uint32_t tail;
    uint32_t ctrl = ldl_le_p(&s->regs_rw[RIO_REG_CQ_CONTROL]);
    uint32_t err = 0;

    /* Fetch latest tail position */
    tail = s->cq_mask & ldl_le_p(&s->regs_rw[RIO_REG_CQ_TAIL]);

    /* Check for pending error or queue processing disabled */
    if (!(ctrl & RIO_CQ_ACTIVE) || !!(ctrl & (RIO_CQ_ERROR | RIO_CQ_FAULT))) {
        return;
    }

    while (tail != s->cq_head) {
        addr = s->cq  + s->cq_head * sizeof(cmd);
        res = dma_memory_read(&address_space_memory, addr, &cmd, sizeof(cmd),
                              ma);

        if (res != MEMTX_OK) {
            err = RIO_CQ_FAULT;
            break;
        }

        trace_riscv_iommu_cmd(PCI_BUS_NUM(s->devid), PCI_SLOT(s->devid),
                              PCI_FUNC(s->devid), cmd.request, cmd.address);

        int fun_op = get_field(cmd.request, RIO_CMD_MASK_FUN_OP);

        switch (fun_op) {
        case RIO_CMD_IOFENCE_C:
            riscv_iommu_iofence(s, !!(cmd.request & RIO_IOFENCE_AV),
                    cmd.address,
                    get_field(cmd.request, RIO_IOFENCE_MASK_DATA));
            break;

        case RIO_CMD_IOTINVAL_GVMA:
            /* IOATC not implemented */
            break;

        case RIO_CMD_IOTINVAL_MSI:
            /* IOATC not implemented */
            break;

        case RIO_CMD_IOTINVAL_VMA:
            /* IOATC not implemented */
            break;

        case RIO_CMD_IODIR_INV_DDT:
            riscv_iommu_iodir_inval_ddt(s,
                    !(cmd.request & RIO_IODIR_DID_VALID),
                    get_field(cmd.request, RIO_IODIR_MASK_DID));
            break;

        case RIO_CMD_IODIR_INV_PDT:
            /* PDT/PASID not implemented */
            break;

        case RIO_CMD_IODIR_PRE_PDT:
            /* PDT/PASID not implemented */
            break;

        case RIO_CMD_IODIR_PRE_DDT:
            /* DDT prefetch not implemented */
            break;

        default:
            err = RIO_CQ_ERROR;
            break;
        }

        /* Invalid instruction, keep cq_head at failed instruction index. */
        if (err) {
            break;
        }

        s->cq_head = (s->cq_head + 1) & s->cq_mask;
    }

    stl_le_p(&s->regs_rw[RIO_REG_CQ_HEAD], s->cq_head);

    if (err) {
        riscv_iommu_reg_mod(s, RIO_REG_CQ_CONTROL, err, 0);
    }

    if (ctrl & RIO_CQ_IE) {
        riscv_iommu_irq_assert(s, RIO_INT_CQ);
    }
}

static void riscv_iommu_process_ddtp(RISCVIOMMUState *s)
{
    uint64_t new_ddtp = ldq_le_p(&s->regs_rw[RIO_REG_DDTP]);
    unsigned new_mode = get_field(new_ddtp, RIO_DDTP_MASK_MODE);
    uint64_t old_ddtp = qatomic_read(&s->ddtp);
    unsigned old_mode = get_field(old_ddtp, RIO_DDTP_MASK_MODE);
    bool ok = false;

    /*
     * Allowed DDTP.MODE transitions:
     * {OFF, BARE} -> {OFF, BARE, 1LVL, 2LVL, 3LVL}
     * {1LVL, 2LVL, 3LVL} -> {OFF, BARE}
     */

    if (new_mode == old_mode ||
        new_mode == RIO_DDTP_MODE_OFF ||
        new_mode == RIO_DDTP_MODE_BARE) {
        ok = true;
    } else if (new_mode == RIO_DDTP_MODE_1LVL ||
               new_mode == RIO_DDTP_MODE_2LVL ||
               new_mode == RIO_DDTP_MODE_3LVL) {
        ok = old_mode == RIO_DDTP_MODE_OFF ||
             old_mode == RIO_DDTP_MODE_BARE;
    }

    if (ok) {
        /* clear reserved and busy bits, report back sanitized version */
        new_ddtp = set_field(get_field(new_ddtp, RIO_DDTP_MASK_PPN),
                             RIO_DDTP_MASK_MODE, new_mode);
    } else {
        new_ddtp = old_ddtp;
    }
    qatomic_set(&s->ddtp, new_ddtp);
    stq_le_p(&s->regs_rw[RIO_REG_DDTP], new_ddtp);
}

static void riscv_iommu_process_cq_control(RISCVIOMMUState *s)
{
    uint64_t base;
    uint32_t ctrl_set = ldl_le_p(&s->regs_rw[RIO_REG_CQ_CONTROL]);
    uint32_t ctrl_clr;
    bool enable = !!(ctrl_set & RIO_FQ_EN);
    bool active = !!(ctrl_set & RIO_FQ_ACTIVE);

    if (enable && !active) {
        base = ldq_le_p(&s->regs_rw[RIO_REG_CQ_BASE]);
        s->cq_mask = (2ULL << get_field(base, RIO_CQ_MASK_LOG2SZ)) - 1;
        s->cq = PPN_PHYS(get_field(base, RIO_CQ_MASK_PPN));
        s->cq_head = 0;
        stl_le_p(&s->regs_ro[RIO_REG_CQ_TAIL], ~s->cq_mask);
        stl_le_p(&s->regs_rw[RIO_REG_CQ_HEAD], s->cq_head);
        stl_le_p(&s->regs_rw[RIO_REG_CQ_TAIL], s->cq_head);
        ctrl_set = RIO_CQ_ACTIVE;
        ctrl_clr = RIO_CQ_BUSY | RIO_CQ_FAULT | RIO_CQ_ERROR | RIO_CQ_TIMEOUT;
    } else if (!enable && active) {
        stl_le_p(&s->regs_ro[RIO_REG_CQ_TAIL], ~0);
        ctrl_set = 0;
        ctrl_clr = RIO_CQ_BUSY | RIO_CQ_ACTIVE;
    } else {
        ctrl_set = 0;
        ctrl_clr = RIO_CQ_BUSY;
    }

    riscv_iommu_reg_mod(s, RIO_REG_CQ_CONTROL, ctrl_set, ctrl_clr);
}

static void riscv_iommu_process_fq_control(RISCVIOMMUState *s)
{
    uint64_t base;
    uint32_t ctrl_set = ldl_le_p(&s->regs_rw[RIO_REG_FQ_CONTROL]);
    uint32_t ctrl_clr;
    bool enable = !!(ctrl_set & RIO_FQ_EN);
    bool active = !!(ctrl_set & RIO_FQ_ACTIVE);

    if (enable && !active) {
        base = ldq_le_p(&s->regs_rw[RIO_REG_FQ_BASE]);
        s->fq_mask = (2ULL << get_field(base, RIO_FQ_MASK_LOG2SZ)) - 1;
        s->fq = PPN_PHYS(get_field(base, RIO_FQ_MASK_PPN));
        s->fq_tail = 0;
        stl_le_p(&s->regs_rw[RIO_REG_FQ_HEAD], s->fq_tail);
        stl_le_p(&s->regs_rw[RIO_REG_FQ_TAIL], s->fq_tail);
        stl_le_p(&s->regs_ro[RIO_REG_FQ_HEAD], ~s->fq_mask);
        ctrl_set = RIO_FQ_ACTIVE;
        ctrl_clr = RIO_FQ_BUSY | RIO_FQ_FAULT | RIO_FQ_FULL;
    } else if (!enable && active) {
        stl_le_p(&s->regs_ro[RIO_REG_FQ_HEAD], ~0);
        ctrl_set = 0;
        ctrl_clr = RIO_FQ_BUSY | RIO_FQ_ACTIVE;
    } else {
        ctrl_set = 0;
        ctrl_clr = RIO_FQ_BUSY;
    }

    riscv_iommu_reg_mod(s, RIO_REG_FQ_CONTROL, ctrl_set, ctrl_clr);
}

static void riscv_iommu_process_pq_control(RISCVIOMMUState *s)
{
    uint64_t base;
    uint32_t ctrl_set = ldl_le_p(&s->regs_rw[RIO_REG_PQ_CONTROL]);
    uint32_t ctrl_clr;
    bool enable = !!(ctrl_set & RIO_PQ_EN);
    bool active = !!(ctrl_set & RIO_PQ_ACTIVE);

    if (enable && !active) {
        base = ldq_le_p(&s->regs_rw[RIO_REG_PQ_BASE]);
        s->pq_mask = (2ULL << get_field(base, RIO_PQ_MASK_LOG2SZ)) - 1;
        s->pq = PPN_PHYS(get_field(base, RIO_PQ_MASK_PPN));
        s->pq_tail = 0;
        stl_le_p(&s->regs_rw[RIO_REG_PQ_HEAD], s->pq_tail);
        stl_le_p(&s->regs_rw[RIO_REG_PQ_TAIL], s->pq_tail);
        stl_le_p(&s->regs_ro[RIO_REG_PQ_HEAD], ~s->pq_mask);
        ctrl_set = RIO_PQ_ACTIVE;
        ctrl_clr = RIO_PQ_BUSY | RIO_PQ_FAULT | RIO_PQ_FULL;
    } else if (!enable && active) {
        stl_le_p(&s->regs_ro[RIO_REG_PQ_HEAD], ~0);
        ctrl_set = 0;
        ctrl_clr = RIO_PQ_BUSY | RIO_PQ_ACTIVE;
    } else {
        ctrl_set = 0;
        ctrl_clr = RIO_PQ_BUSY;
    }

    riscv_iommu_reg_mod(s, RIO_REG_PQ_CONTROL, ctrl_set, ctrl_clr);
}

static void *riscv_iommu_core_proc(void* arg)
{
    RISCVIOMMUState *s = arg;
    unsigned exec = 0;
    unsigned mask = 0;

    do {
        mask = (mask ? mask : BIT(RIO_EXEC_LAST)) >> 1;
        switch (exec & mask) {
        case BIT(RIO_EXEC_DDTP):
            riscv_iommu_process_ddtp(s);
            break;
        case BIT(RIO_EXEC_CQ_CONTROL):
            riscv_iommu_process_cq_control(s);
            break;
        case BIT(RIO_EXEC_CQ_DB):
            riscv_iommu_process_cq_db(s);
            break;
        case BIT(RIO_EXEC_FQ_CONTROL):
            riscv_iommu_process_fq_control(s);
            break;
        case BIT(RIO_EXEC_FQ_DB):
            /* NOP */
            break;
        case BIT(RIO_EXEC_PQ_CONTROL):
            riscv_iommu_process_pq_control(s);
            break;
        case BIT(RIO_EXEC_PQ_DB):
            /* NOP */
            break;
        }
        exec &= ~mask;
        exec |= qatomic_xchg(&s->core_exec, 0);
        if (!exec) {
            qemu_mutex_lock(&s->core_lock);
            qemu_cond_wait(&s->core_cond, &s->core_lock);
            qemu_mutex_unlock(&s->core_lock);
        }
    } while (!(exec & BIT(RIO_EXEC_EXIT)));

    return NULL;
}

static void riscv_iommu_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    RISCVIOMMUState *s = opaque;
    uint32_t regb = (addr + size - 1) & ~3;
    uint32_t exec = 0;
    uint32_t busy = 0;

    if (size == 0 || size > 8 || (addr & (size - 1)) != 0) {
        /* Unsupported MMIO alignment or access size */
        return;
    }

    if (addr + size > sizeof(s->regs_rw)) {
        /* Unsupported MMIO access location. */
        return;
    }

    /* Track actionable MMIO write. */
    switch (regb) {
    case RIO_REG_DDTP:
        exec = BIT(RIO_EXEC_DDTP);
        regb = RIO_REG_DDTP_HI;
        busy = RIO_DDTP_HI_BUSY;
        break;

    case RIO_REG_DDTP_HI:
        exec = BIT(RIO_EXEC_DDTP);
        busy = RIO_DDTP_HI_BUSY;
        break;

    case RIO_REG_CQ_TAIL:
        exec = BIT(RIO_EXEC_CQ_DB);
        break;

    case RIO_REG_CQ_CONTROL:
        exec = BIT(RIO_EXEC_CQ_CONTROL);
        busy = RIO_CQ_BUSY;
        break;

    case RIO_REG_FQ_HEAD:
        exec = BIT(RIO_EXEC_FQ_DB);
        break;

    case RIO_REG_FQ_CONTROL:
        exec = BIT(RIO_EXEC_FQ_CONTROL);
        busy = RIO_FQ_BUSY;
        break;

    case RIO_REG_PQ_HEAD:
        exec = BIT(RIO_EXEC_PQ_DB);
        break;

    case RIO_REG_PQ_CONTROL:
        exec = BIT(RIO_EXEC_PQ_CONTROL);
        busy = RIO_PQ_BUSY;
        break;
    }

    qemu_mutex_lock(&s->core_lock);
    if (size == 1) {
        uint8_t ro = s->regs_ro[addr];
        uint8_t wc = s->regs_wc[addr];
        uint8_t rw = s->regs_rw[addr];
        s->regs_rw[addr] = ((rw & ro) | (val & ~ro)) & ~(val & wc);
    } else if (size == 2) {
        uint16_t ro = lduw_le_p(&s->regs_ro[addr]);
        uint16_t wc = lduw_le_p(&s->regs_wc[addr]);
        uint16_t rw = lduw_le_p(&s->regs_rw[addr]);
        stw_le_p(&s->regs_rw[addr], ((rw & ro) | (val & ~ro)) & ~(val & wc));
    } else if (size == 4) {
        uint32_t ro = ldl_le_p(&s->regs_ro[addr]);
        uint32_t wc = ldl_le_p(&s->regs_wc[addr]);
        uint32_t rw = ldl_le_p(&s->regs_rw[addr]);
        stl_le_p(&s->regs_rw[addr], ((rw & ro) | (val & ~ro)) & ~(val & wc));
    } else if (size == 8) {
        uint64_t ro = ldq_le_p(&s->regs_ro[addr]);
        uint64_t wc = ldq_le_p(&s->regs_wc[addr]);
        uint64_t rw = ldq_le_p(&s->regs_rw[addr]);
        stq_le_p(&s->regs_rw[addr], ((rw & ro) | (val & ~ro)) & ~(val & wc));
    }

    /* Busy flag update, MSB 4-byte register. */
    if (busy) {
        uint32_t rw = ldl_le_p(&s->regs_rw[regb]);
        stl_le_p(&s->regs_rw[regb], rw | busy);
    }
    qemu_mutex_unlock(&s->core_lock);

    /* Wakeup core processing thread. */
    if (exec) {
        qatomic_or(&s->core_exec, exec);
        qemu_cond_signal(&s->core_cond);
    }
}

static uint64_t riscv_iommu_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    RISCVIOMMUState *s = opaque;
    uint64_t val = -1;

    if (addr + size > sizeof(s->regs_rw)) {
        return (uint64_t)-1;
    } else if (size == 1) {
        val = (uint64_t) s->regs_rw[addr];
    } else if (size == 2) {
        val = lduw_le_p(&s->regs_rw[addr]);
    } else if (size == 4) {
        val = ldl_le_p(&s->regs_rw[addr]);
    } else if (size == 8) {
        val = ldq_le_p(&s->regs_rw[addr]);
    }

    return val;
}

static const MemoryRegionOps riscv_iommu_mmio_ops = {
    .read = riscv_iommu_mmio_read,
    .write = riscv_iommu_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = true,
    },
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    }
};

static void riscv_iommu_init(RISCVIOMMUState *s)
{
    const uint64_t cap = set_field((
            (s->version & RIO_CAP_REVISION_MASK) |
            (s->enable_s_stage * RIO_CAP_S_ANY) |
            (s->enable_g_stage * RIO_CAP_G_ANY) |
            (s->enable_msi * RIO_CAP_MSI_FLAT) |
            (s->enable_ats * RIO_CAP_ATS)),
            RIO_CAP_PAS_MASK, TARGET_PHYS_ADDR_SPACE_BITS);

    /* Out-of-reset translation mode: OFF (DMA disabled) BARE (passthrough) */
    s->ddtp = set_field(0, RIO_DDTP_MASK_MODE, s->enable_off ?
                        RIO_DDTP_MODE_OFF : RIO_DDTP_MODE_BARE);

    /* Mark all registers read-only */
    memset(s->regs_ro, 0xff, sizeof(s->regs_ro));
    memset(s->regs_rw, 0x00, sizeof(s->regs_rw));
    memset(s->regs_wc, 0x00, sizeof(s->regs_wc));

    /* Set power-on register state */
    stq_le_p(&s->regs_rw[RIO_REG_CAP], cap);
    stq_le_p(&s->regs_ro[RIO_REG_DDTP],
        ~(RIO_DDTP_MASK_PPN | RIO_DDTP_MASK_MODE));
    stq_le_p(&s->regs_ro[RIO_REG_CQ_BASE],
        ~(RIO_CQ_MASK_LOG2SZ | RIO_CQ_MASK_PPN));
    stq_le_p(&s->regs_ro[RIO_REG_FQ_BASE],
        ~(RIO_FQ_MASK_LOG2SZ | RIO_FQ_MASK_PPN));
    stq_le_p(&s->regs_ro[RIO_REG_PQ_BASE],
        ~(RIO_PQ_MASK_LOG2SZ | RIO_PQ_MASK_PPN));
    stl_le_p(&s->regs_wc[RIO_REG_CQ_CONTROL],
        RIO_CQ_FAULT | RIO_CQ_TIMEOUT | RIO_CQ_ERROR);
    stl_le_p(&s->regs_ro[RIO_REG_CQ_CONTROL], RIO_CQ_ACTIVE | RIO_CQ_BUSY);
    stl_le_p(&s->regs_wc[RIO_REG_FQ_CONTROL], RIO_FQ_FAULT | RIO_FQ_FULL);
    stl_le_p(&s->regs_ro[RIO_REG_FQ_CONTROL], RIO_FQ_ACTIVE | RIO_FQ_BUSY);
    stl_le_p(&s->regs_wc[RIO_REG_PQ_CONTROL], RIO_PQ_FAULT | RIO_PQ_FULL);
    stl_le_p(&s->regs_ro[RIO_REG_PQ_CONTROL], RIO_PQ_ACTIVE | RIO_PQ_BUSY);
    stl_le_p(&s->regs_wc[RIO_REG_IPSR], ~0);
    stl_le_p(&s->regs_ro[RIO_REG_IVEC], 0);
    stq_le_p(&s->regs_rw[RIO_REG_DDTP], s->ddtp);

    QLIST_INIT(&s->spaces);
    qemu_cond_init(&s->core_cond);
    qemu_mutex_init(&s->core_lock);
    qemu_thread_create(&s->core_proc, "riscv-iommu-core",
        riscv_iommu_core_proc, s, QEMU_THREAD_JOINABLE);
}

static void riscv_iommu_exit(RISCVIOMMUState *s)
{
    qatomic_or(&s->core_exec, BIT(RIO_EXEC_EXIT));
    qemu_cond_signal(&s->core_cond);
    qemu_thread_join(&s->core_proc);
    qemu_cond_destroy(&s->core_cond);
    qemu_mutex_destroy(&s->core_lock);
}

static AddressSpace *riscv_iommu_find_as(PCIBus *bus, void *opaque, int devfn)
{
    RISCVIOMMUState *s = opaque;
    RISCVIOMMUSpace *as;
    char name[64];
    uint32_t devid = PCI_BUILD_BDF(pci_bus_num(bus), devfn);

    if (s->devid == devid) {
        /* No translation for IOMMU device itself. */
        return &address_space_memory;
    }

    qemu_mutex_lock(&s->core_lock);
    QLIST_FOREACH(as, &s->spaces, list) {
        if (as->devid == devid) {
            break;
        }
    }
    qemu_mutex_unlock(&s->core_lock);

    if (as == NULL) {
        as = g_new0(RISCVIOMMUSpace, 1);

        as->iommu = s;
        as->devid = devid;

        snprintf(name, sizeof(name), "riscv-iommu-%04x:%02x.%d-iova",
            PCI_BUS_NUM(as->devid), PCI_SLOT(as->devid), PCI_FUNC(as->devid));

        memory_region_init_iommu(&as->mr, sizeof(as->mr),
            TYPE_RISCV_IOMMU_MEMORY_REGION,
            OBJECT(as), name, UINT64_MAX);

        address_space_init(&as->as, MEMORY_REGION(&as->mr),
            TYPE_RISCV_IOMMU_PCI);

        qemu_mutex_lock(&s->core_lock);
        QLIST_INSERT_HEAD(&s->spaces, as, list);
        qemu_mutex_unlock(&s->core_lock);

        trace_riscv_iommu_new(PCI_BUS_NUM(s->devid), PCI_SLOT(s->devid),
            PCI_FUNC(s->devid), PCI_BUS_NUM(as->devid), PCI_SLOT(as->devid),
            PCI_FUNC(as->devid));
    }

    return &as->as;
}

/* RISC-V IOMMU PCI Device Emulation */

struct RISCVIOMMUStatePci {
    PCIDevice        pci;     /* Parent PCIe device state */
    MemoryRegion     bar0;    /* PCI BAR (including MSI-x config) */
    MemoryRegion     regs;    /* PCI MMIO interface */
    RISCVIOMMUState  iommu;   /* common IOMMU state */
};

/* interrupt delivery callback */
static void riscv_iommu_pci_notify(RISCVIOMMUState *iommu, unsigned vector)
{
    RISCVIOMMUStatePci *s = container_of(iommu, RISCVIOMMUStatePci, iommu);

    if (msix_enabled(&(s->pci))) {
        msix_notify(&(s->pci), vector);
    }
}

static void riscv_iommu_pci_realize(PCIDevice *dev, Error **errp)
{
    DeviceState *d = DEVICE(dev);
    RISCVIOMMUStatePci *s = RISCV_IOMMU_PCI(d);
    RISCVIOMMUState *iommu = &s->iommu;
    const uint64_t bar_size =
        pow2ceil(QEMU_ALIGN_UP(sizeof(iommu->regs_rw), TARGET_PAGE_SIZE));
    Error *err = NULL;

    iommu->devid = pci_get_bdf(dev);

    riscv_iommu_init(iommu);

    memory_region_init(&s->bar0, OBJECT(s),
            "riscv-iommu-bar0", bar_size);
    memory_region_init_io(&s->regs, OBJECT(s), &riscv_iommu_mmio_ops, iommu,
            "riscv-iommu-regs", sizeof(iommu->regs_rw));
    memory_region_add_subregion(&s->bar0, 0, &s->regs);

    pcie_endpoint_cap_init(dev, 0x80);

    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY |
            PCI_BASE_ADDRESS_MEM_TYPE_64, &s->bar0);

    int ret = msix_init(dev, RIO_INT_COUNT,
                    &s->bar0, 0, RIO_REG_MSI_ADDR_BASE,
                    &s->bar0, 0, RIO_REG_MSI_PBA_BASE, 0, &err);

    if (ret == -ENOTSUP) {
        /*
         * MSI-x is not supported by the platform.
         * Driver should use timer/polling based notification handlers.
         */
        warn_report_err(err);
    } else if (ret < 0) {
        error_propagate(errp, err);
        return;
    } else {
        /* mark all allocated MSIx vectors as used. */
        while (ret-- > 0) {
            msix_vector_use(dev, ret);
        }
        iommu->notify = riscv_iommu_pci_notify;
    }

    /* TODO: find root port bus ranges and use for FDT/ACPI generation. */
    PCIBus *bus = pci_device_root_bus(dev);
    if (!bus) {
        error_setg(errp, "can't find PCIe root port for %02x:%02x.%x",
            pci_bus_num(pci_get_bus(dev)), PCI_SLOT(dev->devfn),
            PCI_FUNC(dev->devfn));
        return;
    }

    pci_setup_iommu(bus, riscv_iommu_find_as, iommu);
}

static void riscv_iommu_pci_exit(PCIDevice *dev)
{
    DeviceState *d = DEVICE(dev);
    RISCVIOMMUStatePci *s = RISCV_IOMMU_PCI(d);

    pci_setup_iommu(pci_device_root_bus(dev), NULL, NULL);
    riscv_iommu_exit(&s->iommu);
}

static const VMStateDescription riscv_iommu_vmstate = {
    .name = "riscv-iommu",
    .unmigratable = 1
};

static Property riscv_iommu_properties[] = {
    DEFINE_PROP_UINT32("version", RISCVIOMMUStatePci, iommu.version, 0x02),
    DEFINE_PROP_BOOL("msi", RISCVIOMMUStatePci, iommu.enable_msi, TRUE),
    DEFINE_PROP_BOOL("ats", RISCVIOMMUStatePci, iommu.enable_ats, TRUE),
    DEFINE_PROP_BOOL("off", RISCVIOMMUStatePci, iommu.enable_off, FALSE),
    DEFINE_PROP_BOOL("s-stage", RISCVIOMMUStatePci, iommu.enable_s_stage, TRUE),
    DEFINE_PROP_BOOL("g-stage", RISCVIOMMUStatePci, iommu.enable_g_stage, TRUE),
    DEFINE_PROP_END_OF_LIST(),
};

static void riscv_iommu_pci_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    device_class_set_props(dc, riscv_iommu_properties);
    k->realize = riscv_iommu_pci_realize;
    k->exit = riscv_iommu_pci_exit;
    k->vendor_id = PCI_VENDOR_ID_RIVOS;
    k->device_id = PCI_DEVICE_ID_RIVOS_IOMMU;
    k->revision = 0;
    k->class_id = PCI_CLASS_SYSTEM_IOMMU;
    dc->desc = "RISCV-IOMMU DMA Remapping device";
    dc->vmsd = &riscv_iommu_vmstate;
    dc->hotpluggable = false;
    dc->user_creatable = true;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo riscv_iommu_pci = {
    .name = TYPE_RISCV_IOMMU_PCI,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(RISCVIOMMUStatePci),
    .class_init = riscv_iommu_pci_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { },
    },
};

static const char *IOMMU_FLAG_STR[] = {
    "NA",
    "RO",
    "WR",
    "RW",
};

/* RISC-V IOMMU Memory Region - Address Translation Space */
static IOMMUTLBEntry riscv_iommu_memory_region_translate(
        IOMMUMemoryRegion *iommu_mr, hwaddr addr,
        IOMMUAccessFlags flag, int iommu_idx)
{
    RISCVIOMMUSpace *as = container_of(iommu_mr, RISCVIOMMUSpace, mr);
    IOMMUTLBEntry iotlb = {
        .iova = addr,
        .target_as = &address_space_memory,
        .addr_mask = ~0ULL,
        .perm = flag,
    };

    if (riscv_iommu_translate(as, &iotlb)) {
        /* Translation fault reported. */
        iotlb.addr_mask = 0;
        iotlb.perm = IOMMU_NONE;
    }

    /* Trace all dma translations with original access flags. */
    trace_riscv_iommu_dma(PCI_BUS_NUM(as->devid), PCI_SLOT(as->devid),
        PCI_FUNC(as->devid), IOMMU_FLAG_STR[flag & IOMMU_RW],
        iotlb.iova, iotlb.translated_addr);

    return iotlb;
}

static int riscv_iommu_memory_region_notify(
    IOMMUMemoryRegion *iommu_mr, IOMMUNotifierFlag old,
    IOMMUNotifierFlag new, Error **errp)
{
    if (new & IOMMU_NOTIFIER_DEVIOTLB_UNMAP) {
        error_setg(errp, "riscv-iommu does not support dev-iotlb");
        return -EINVAL;
    }
    return 0;
}

static void riscv_iommu_memory_region_init(ObjectClass *klass, void *data)
{
    IOMMUMemoryRegionClass *imrc = IOMMU_MEMORY_REGION_CLASS(klass);

    imrc->translate = riscv_iommu_memory_region_translate;
    imrc->notify_flag_changed = riscv_iommu_memory_region_notify;
}

static const TypeInfo riscv_iommu_memory_region_info = {
    .parent = TYPE_IOMMU_MEMORY_REGION,
    .name = TYPE_RISCV_IOMMU_MEMORY_REGION,
    .class_init = riscv_iommu_memory_region_init,
};

static void riscv_iommu_register_types(void)
{
    type_register_static(&riscv_iommu_memory_region_info);
    type_register_static(&riscv_iommu_pci);
}

type_init(riscv_iommu_register_types);
