/*
 * QEMU emulation of an RISC-V IOMMU
 *
 * Copyright (C) 2022-2024 Rivos Inc.
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

#ifndef HW_RISCV_IOMMU_H
#define HW_RISCV_IOMMU_H

#include "qemu/osdep.h"
#include "qom/object.h"

#define TYPE_RISCV_IOMMU "x-riscv-iommu"
OBJECT_DECLARE_SIMPLE_TYPE(RISCVIOMMUState, RISCV_IOMMU)
typedef struct RISCVIOMMUState RISCVIOMMUState;

#define TYPE_RISCV_IOMMU_MEMORY_REGION "x-riscv-iommu-mr"
typedef struct RISCVIOMMUSpace RISCVIOMMUSpace;

#define TYPE_RISCV_IOMMU_PCI "x-riscv-iommu-pci"
OBJECT_DECLARE_SIMPLE_TYPE(RISCVIOMMUStatePci, RISCV_IOMMU_PCI)
typedef struct RISCVIOMMUStatePci RISCVIOMMUStatePci;

#define TYPE_RISCV_IOMMU_SYS "x-riscv-iommu-device"
OBJECT_DECLARE_SIMPLE_TYPE(RISCVIOMMUStateSys, RISCV_IOMMU_SYS)
typedef struct RISCVIOMMUStateSys RISCVIOMMUStateSys;

/* Disable implementation feature bits. QDev 'disable' property. */
enum {
    RISCV_IOMMU_DISABLE_IOTLB,        /* Disable translation cache */
    RISCV_IOMMU_DISABLE_PCI_SEGMENT,  /* Ignore segment number for PCIe endpoints */
    RISCV_IOMMU_DISABLE_DDT_FLUSH,    /* Disable cache clear on DDT update */
    RISCV_IOMMU_DISABLE_DDT_1LVL,     /* Disable 1-level DDT mode */
    RISCV_IOMMU_DISABLE_DDT_2LVL,     /* Disable 2-level DDT mode */
    RISCV_IOMMU_DISABLE_DDT_3LVL,     /* Disable 3-level DDT mode */
};


#endif
