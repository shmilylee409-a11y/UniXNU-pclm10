#!/usr/bin/env python3
"""Prepare a Mu-Silicium (UniXNU/uefi-sheng) checkout and DTB for OPPO Reno Ace (pclm10)."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil


PCLM10_TOML = """\
#
# Boot Shim Configs
#
[boot_shim]
requires_kernel_header = true

#
# UEFI FD Configs
#
[uefi_fd]
base = 0x9FC00000
size = 0x300000

#
# Android Boot Image Kernel Configs
#
[boot_image_kernel]
kernel_compression = "gzip"
append_dtb         = true

#
# Android Boot Image Configs
#
[boot_image]
header_version = 1
"""

PCLM10_DEC = """\
[Defines]
  DEC_SPECIFICATION              = 0x00010005
  PACKAGE_NAME                   = pclm10Pkg
  PACKAGE_GUID                   = 9D2E6F20-1908-4B10-A855-001908100001
  PACKAGE_VERSION                = 0.1
"""

PCLM10_DSC = """\
##
#  SPDX-License-Identifier: BSD-2-Clause-Patent
##

[Defines]
  PLATFORM_NAME                  = pclm10
  PLATFORM_GUID                  = 9D2E6F21-1908-4B10-A855-001908100001
  PLATFORM_VERSION               = 0.1
  DSC_SPECIFICATION              = 0x00010005
  OUTPUT_DIRECTORY               = Build/pclm10Pkg
  SUPPORTED_ARCHITECTURES        = AARCH64
  BUILD_TARGETS                  = RELEASE|DEBUG
  SKUID_IDENTIFIER               = DEFAULT
  FLASH_DEFINITION               = pclm10Pkg/pclm10.fdf
  USE_CUSTOM_DISPLAY_DRIVER      = 0
  SHENG_VENDOR_HW                = 0

  #
  # 0 = SM8150-AB
  # 1 = SM8150-AC / 855+
  # 2 = SM8150-AC / 860
  #
  SOC_TYPE                       = 1

!include HanaPkg/HanaPkg.dsc.inc

[PcdsFixedAtBuild]
  # DDR Memory
  gArmTokenSpaceGuid.PcdSystemMemoryBase|0x80000000

  # UEFI Stack (matches "UEFI Stack" in MemoryMapLib.c)
  gArmPlatformTokenSpaceGuid.PcdCPUCoresStackBase|0x9FFB0000
  gArmPlatformTokenSpaceGuid.PcdCPUCorePrimaryStackSize|0x20000

  # SMBIOS
  gSiliciumPkgTokenSpaceGuid.PcdSmbiosSystemManufacturer|"OPPO"
  gSiliciumPkgTokenSpaceGuid.PcdSmbiosSystemModel|"Reno Ace"
  gSiliciumPkgTokenSpaceGuid.PcdSmbiosSystemRetailModel|"PCLM10"
  gSiliciumPkgTokenSpaceGuid.PcdSmbiosSystemRetailSku|"OPPO_Reno_Ace_PCLM10"
  gSiliciumPkgTokenSpaceGuid.PcdSmbiosSystemBoardModel|"PCLM10"

  # Simple Frame Buffer (1080x2400 @ 0x9D000000)
  gSiliciumPkgTokenSpaceGuid.PcdFrameBufferWidth|1080
  gSiliciumPkgTokenSpaceGuid.PcdFrameBufferHeight|2400
  gSiliciumPkgTokenSpaceGuid.PcdFrameBufferColorDepth|32

  # Platform PEI (skip XBL shared-IMEM HOB walking during first bring-up)
  gQcomPkgTokenSpaceGuid.PcdPlatformType|"LA"
  gQcomPkgTokenSpaceGuid.PcdBuildXblHobs|FALSE

[LibraryClasses]
  MemoryMapLib|pclm10Pkg/Library/MemoryMapLib/MemoryMapLib.inf
  ConfigurationMapLib|pclm10Pkg/Library/ConfigurationMapLib/ConfigurationMapLib.inf
  UniXNULib|UniXNUPkg/Library/UniXNULib/UniXNULib.inf
  FdtLib|MdePkg/Library/BaseFdtLib/BaseFdtLib.inf

[Components]
  UniXNUPkg/Application/UniXNU/UniXNU.inf
  UniXNUPkg/Driver/UniXNUDxe/UniXNUDxe.inf
  shengPkg/Drivers/ShengBootMemoryDxe/ShengBootMemoryDxe.inf
  UniXNUPkg/Driver/UniXNURamDiskDxe/UniXNURamDiskDxe.inf
  shengPkg/Drivers/ShengKeyInputDxe/ShengKeyInputDxe.inf
  DarwinBootPkg/Driver/DarwinBoot.inf
"""

PCLM10_FDF = """\
##
#  SPDX-License-Identifier: BSD-2-Clause-Patent
##

[FD.SILICIUM_UEFI]
BaseAddress   = $(FD_BASE)|gArmTokenSpaceGuid.PcdFdBaseAddress
Size          = $(FD_SIZE)|gArmTokenSpaceGuid.PcdFdSize
ErasePolarity = 1

BlockSize     = 0x1000
NumBlocks     = $(FD_BLOCKS)

0x00000000|$(FD_SIZE)
gArmTokenSpaceGuid.PcdFvBaseAddress|gArmTokenSpaceGuid.PcdFvSize
FV = FVMAIN_COMPACT

[FV.FvMain]
FvNameGuid         = CD8D4C84-20AD-4073-8A28-3400FAE05941
BlockSize          = 0x1000
NumBlocks          = 0
FvAlignment        = 8
ERASE_POLARITY     = 1
MEMORY_MAPPED      = TRUE
STICKY_WRITE       = TRUE
LOCK_CAP           = TRUE
LOCK_STATUS        = TRUE
WRITE_DISABLED_CAP = TRUE
WRITE_ENABLED_CAP  = TRUE
WRITE_STATUS       = TRUE
WRITE_LOCK_CAP     = TRUE
WRITE_LOCK_STATUS  = TRUE
READ_DISABLED_CAP  = TRUE
READ_ENABLED_CAP   = TRUE
READ_STATUS        = TRUE
READ_LOCK_CAP      = TRUE
READ_LOCK_STATUS   = TRUE

  !include Include/APRIORI.inc
  !include Include/DXE.inc

  # RAM Manager
  INF SiliciumPkg/Drivers/RamManagerDxe/RamManagerDxe.inf

  # SmBios
  INF MdeModulePkg/Universal/SmbiosDxe/SmbiosDxe.inf
  INF SiliciumPkg/Drivers/SmBiosTableDxe/SmBiosTableDxe.inf

  # UniXNU
  INF UniXNUPkg/Application/UniXNU/UniXNU.inf
  INF UniXNUPkg/Driver/UniXNUDxe/UniXNUDxe.inf

[FV.FVMAIN_COMPACT]
FvAlignment        = 8
ERASE_POLARITY     = 1
MEMORY_MAPPED      = TRUE
STICKY_WRITE       = TRUE
LOCK_CAP           = TRUE
LOCK_STATUS        = TRUE
WRITE_DISABLED_CAP = TRUE
WRITE_ENABLED_CAP  = TRUE
WRITE_STATUS       = TRUE
WRITE_LOCK_CAP     = TRUE
WRITE_LOCK_STATUS  = TRUE
READ_DISABLED_CAP  = TRUE
READ_ENABLED_CAP   = TRUE
READ_STATUS        = TRUE
READ_LOCK_CAP      = TRUE
READ_LOCK_STATUS   = TRUE

  INF SiliciumPkg/Sec/Sec.inf
  INF QcomPkg/Pei/PlatformPei.inf

  FILE FV_IMAGE = 9E21FD93-9C72-4c15-8C4B-E77F1DB2D792 {
    SECTION GUIDED EE4E5898-3914-4259-9D6E-DC7BD79403CF PROCESSING_REQUIRED = TRUE {
      SECTION FV_IMAGE = FvMain
    }
  }

!include Silicon/Silicium/SiliciumPkg/Include/Rules.inc
"""

PCLM10_APRIORI = """\
APRIORI DXE {
  # First, install the PCD driver and call DxeInit
  INF MdeModulePkg/Core/Dxe/DxeMain.inf
  INF MdeModulePkg/Universal/PCD/Dxe/Pcd.inf

  INF MdeModulePkg/Universal/ReportStatusCodeRouter/RuntimeDxe/ReportStatusCodeRouterRuntimeDxe.inf
  INF MdeModulePkg/Universal/StatusCodeHandler/RuntimeDxe/StatusCodeHandlerRuntimeDxe.inf
  INF MdeModulePkg/Core/RuntimeDxe/RuntimeDxe.inf

  # Next, install cpu protocol and enable the interrupt controller
  INF ArmPkg/Drivers/CpuDxe/CpuDxe.inf
  INF ArmPkg/Drivers/ArmGicDxe/ArmGicDxe.inf

  # Core Drivers
  INF EmbeddedPkg/MetronomeDxe/MetronomeDxe.inf
  INF ArmPkg/Drivers/TimerDxe/TimerDxe.inf
  INF MdeModulePkg/Universal/SecurityStubDxe/SecurityStubDxe.inf

  INF MdeModulePkg/Universal/CapsuleRuntimeDxe/CapsuleRuntimeDxe.inf
  INF EmbeddedPkg/EmbeddedMonotonicCounter/EmbeddedMonotonicCounter.inf
  INF MdeModulePkg/Universal/ResetSystemRuntimeDxe/ResetSystemRuntimeDxe.inf
  INF EmbeddedPkg/RealTimeClockRuntimeDxe/RealTimeClockRuntimeDxe.inf
  INF MdeModulePkg/Universal/DevicePathDxe/DevicePathDxe.inf

  INF MdeModulePkg/Universal/Variable/RuntimeDxe/VariableRuntimeDxe.inf

  INF MdeModulePkg/Universal/WatchdogTimerDxe/WatchdogTimer.inf
}
"""

PCLM10_DXE = """\
  INF MdeModulePkg/Core/Dxe/DxeMain.inf
  INF MdeModulePkg/Universal/PCD/Dxe/Pcd.inf
  INF MdeModulePkg/Universal/ReportStatusCodeRouter/RuntimeDxe/ReportStatusCodeRouterRuntimeDxe.inf
  INF MdeModulePkg/Universal/StatusCodeHandler/RuntimeDxe/StatusCodeHandlerRuntimeDxe.inf
  INF ArmPkg/Drivers/CpuDxe/CpuDxe.inf
  INF MdeModulePkg/Core/RuntimeDxe/RuntimeDxe.inf
  INF MdeModulePkg/Universal/SecurityStubDxe/SecurityStubDxe.inf
  INF MdeModulePkg/Universal/WatchdogTimerDxe/WatchdogTimer.inf
  INF MdeModulePkg/Universal/CapsuleRuntimeDxe/CapsuleRuntimeDxe.inf
  INF MdeModulePkg/Universal/Variable/RuntimeDxe/VariableRuntimeDxe.inf
  INF EmbeddedPkg/EmbeddedMonotonicCounter/EmbeddedMonotonicCounter.inf
  INF MdeModulePkg/Universal/ResetSystemRuntimeDxe/ResetSystemRuntimeDxe.inf
  INF EmbeddedPkg/RealTimeClockRuntimeDxe/RealTimeClockRuntimeDxe.inf
  INF EmbeddedPkg/MetronomeDxe/MetronomeDxe.inf
  INF MdeModulePkg/Universal/HiiDatabaseDxe/HiiDatabaseDxe.inf
  INF ArmPkg/Drivers/ArmGicDxe/ArmGicDxe.inf
  INF ArmPkg/Drivers/TimerDxe/TimerDxe.inf
  INF MdeModulePkg/Universal/DevicePathDxe/DevicePathDxe.inf

  INF shengPkg/Drivers/ShengBootMemoryDxe/ShengBootMemoryDxe.inf
  INF UniXNUPkg/Driver/UniXNURamDiskDxe/UniXNURamDiskDxe.inf
  INF DarwinBootPkg/Driver/DarwinBoot.inf
  INF MdeModulePkg/Universal/Disk/DiskIoDxe/DiskIoDxe.inf
  INF MdeModulePkg/Universal/Disk/PartitionDxe/PartitionDxe.inf
  INF FatPkg/EnhancedFatDxe/Fat.inf
  INF MdeModulePkg/Universal/Disk/UnicodeCollation/EnglishDxe/EnglishDxe.inf
  INF MdeModulePkg/Universal/BdsDxe/BdsDxe.inf

  INF EmbeddedPkg/SimpleTextInOutSerial/SimpleTextInOutSerial.inf
  INF MdeModulePkg/Universal/PrintDxe/PrintDxe.inf
  INF MdeModulePkg/Universal/Console/ConPlatformDxe/ConPlatformDxe.inf
  INF MdeModulePkg/Universal/Console/ConSplitterDxe/ConSplitterDxe.inf
  INF shengPkg/Drivers/ShengKeyInputDxe/ShengKeyInputDxe.inf
  INF MdeModulePkg/Universal/Console/GraphicsConsoleDxe/GraphicsConsoleDxe.inf
  INF SiliciumPkg/Drivers/SimpleFbDxe/SimpleFbDxe.inf

  INF MdeModulePkg/Bus/Usb/UsbBusDxe/UsbBusDxe.inf
  INF MdeModulePkg/Bus/Usb/UsbKbDxe/UsbKbDxe.inf
  INF MdeModulePkg/Bus/Usb/UsbMassStorageDxe/UsbMassStorageDxe.inf
"""

MEMORY_MAP_INF = """\
[Defines]
  INF_VERSION                    = 0x00010005
  BASE_NAME                      = MemoryMapLib
  FILE_GUID                      = 375C4FF0-1905-4BA7-A5CA-7D40D25F5EF3
  MODULE_TYPE                    = BASE
  VERSION_STRING                 = 1.0
  LIBRARY_CLASS                  = MemoryMapLib

[Sources]
  MemoryMapLib.c

[Packages]
  ArmPkg/ArmPkg.dec
  MdePkg/MdePkg.dec
  SiliciumPkg/SiliciumPkg.dec
"""

MEMORY_MAP_C = """\
// SPDX-License-Identifier: BSD-2-Clause-Patent
// Memory map for OPPO Reno Ace (PCLM10, SM8150-AC, oppo,prjversion = 19081)
// Carveouts verified against dts/oppo-pclm10-19081.dtb (/reserved-memory).
#include <Library/MemoryMapLib.h>

STATIC
EFI_MEMORY_REGION_DESCRIPTOR
gMemoryDescriptor[] = {
  // Name, Address, Length, HobOption, ResourceType, ResourceAttribute, MemoryType, ArmAttribute

  // DDR Regions
  {"Kernel",             0x80000000, 0x05700000, AddMem, SYS_MEM, SYS_MEM_CAP, Reserv, WRITE_BACK_XN},
  {"HYP",                0x85700000, 0x00600000, AddMem, SYS_MEM, SYS_MEM_CAP, Reserv, NS_DEVICE},
  {"XBL_AOP",            0x85E00000, 0x00140000, AddMem, MEM_RES, WRITE_COMBINEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"GPU PRR",            0x85F40000, 0x00010000, AddMem, MEM_RES, WRITE_COMBINEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"SMEM",               0x86000000, 0x00200000, AddMem, MEM_RES, WRITE_COMBINEABLE, Reserv, UNCACHED_UNBUFFERED},
  {"Removed Regions",    0x86200000, 0x07500000, AddMem, MEM_RES, UNCACHEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"PIL_REGION",         0x8DC00000, 0x0F400000, AddMem, SYS_MEM, SYS_MEM_CAP, Reserv, UNCACHED_UNBUFFERED_XN},
  {"Display Reserved",   0x9D000000, 0x02400000, AddMem, MEM_RES, SYS_MEM_CAP, Reserv, WRITE_THROUGH_XN},
  {"FV Region",          0x9F800000, 0x00400000, AddMem, SYS_MEM, SYS_MEM_CAP, BsData, WRITE_BACK_XN},
  {"UEFI FD",            0x9FC00000, 0x00300000, AddMem, SYS_MEM, SYS_MEM_CAP, BsData, WRITE_BACK},
  {"SEC Heap",           0x9FF00000, 0x0008C000, AddMem, SYS_MEM, SYS_MEM_CAP, BsData, WRITE_BACK_XN},
  {"MMU PageTables",     0x9FF8D000, 0x00003000, AddMem, SYS_MEM, SYS_MEM_CAP, BsData, WRITE_BACK_XN},
  {"UEFI Stack",         0x9FFB0000, 0x00020000, AddMem, SYS_MEM, SYS_MEM_CAP, BsData, WRITE_BACK_XN},
  {"Log Buffer",         0x9FFF7000, 0x00008000, AddMem, SYS_MEM, SYS_MEM_CAP, RtData, WRITE_BACK_XN},
  {"Info Blk",           0x9FFFF000, 0x00001000, AddMem, SYS_MEM, SYS_MEM_CAP, RtData, WRITE_BACK_XN},
  {"QSEECOM",            0xA0000000, 0x01400000, AddMem, MEM_RES, UNCACHEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"CDSP_SEC",           0xA4C00000, 0x00C00000, AddMem, MEM_RES, UNCACHEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"DXE Heap",           0xA5800000, 0x1A800000, AddMem, SYS_MEM, SYS_MEM_CAP, Conv,   WRITE_BACK_XN},

  // Other Memory Regions
  {"AOP_SS_MSG_RAM",     0x0C300000, 0x00100000, NoHob,  MMAP_IO, INITIALIZED, Conv,   NS_DEVICE},
  {"IMEM Base",          0x14680000, 0x00040000, NoHob,  MMAP_IO, INITIALIZED, Conv,   NS_DEVICE},

  // Register Regions (SM8150 / HanaPkg)
  {"GCC_CLK_CTL",        0x00100000, 0x00200000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"MMCX_CPR3",          0x00630000, 0x00004000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"SECURITY CONTROL",   0x00780000, 0x00007000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PRNG_CFG_PRNG",      0x00790000, 0x00010000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"QUPV3_0_GSI",        0x00800000, 0x000D0000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"QUPV3_1_GSI",        0x00A00000, 0x000D0000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"QUPV3_2_GSI",        0x00C00000, 0x000D0000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PCIE_0_WRAPPER_AHB", 0x01C00000, 0x00008000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PCIE_1_WRAPPER_AHB", 0x01C08000, 0x00008000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PCIE_2_WRAPPER_AHB", 0x01C10000, 0x00008000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PCIE_3_WRAPPER_AHB", 0x01C18000, 0x00008000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"UFS2 UFS REGS",      0x01D60000, 0x00020000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"UFS UFS REGS",       0x01D80000, 0x00020000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"CRYPTO0 CRYPTO",     0x01DC0000, 0x00040000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TCSR_TCSR_REGS",     0x01FC0000, 0x00030000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"GPU_GMU_CX_BLK",     0x02C7D000, 0x00002000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"GPU_CC",             0x02C90000, 0x0000A000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"GPU_CPR",            0x02C9A000, 0x00004000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TLMM_WEST",          0x03100000, 0x00300000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TLMM_EAST",          0x03500000, 0x00300000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TLMM_NORTH",         0x03900000, 0x00300000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TLMM_SOUTH",         0x03D00000, 0x00300000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"GPMU_RGMU_BASE",     0x05060000, 0x0000D000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"SYS_NOC",            0x08580000, 0x00014000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"QFPROM_CORR",        0x00784000, 0x00007000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PERIPH_SS",          0x08800000, 0x00200000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"MCCC_MCCC_MSTR",     0x090B0000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"USB30_PRIM",         0x0A600000, 0x0011B000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"USB_RUMI",           0x0A720000, 0x00010000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"USB30_SEC",          0x0A800000, 0x0011B000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"VIDEO_CC",           0x0AB00000, 0x00020000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TITAN_SS_TITAN_1",   0x0AC00000, 0x0004A000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TITAN_A_CCI",        0x0AC4A000, 0x00004000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TITAN_SS_TITAN_2",   0x0AC4E000, 0x000B2000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TITAN_CAM_CC",       0x0AD00000, 0x00020000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"MDSS",               0x0AE00000, 0x00134000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"DISP_CC",            0x0AF00000, 0x00020000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PDC_DISPLAY",        0x0B290000, 0x00020000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PDC_DISP_SEQ",       0x0B490000, 0x00020000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"RPMH_BCM_BCM_TOP",   0x0BA00000, 0x00200000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"RPMH_CPRF_CPRF",     0x0C200000, 0x00010000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"SLP_CNTR",           0x0C221000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TSENS0",             0x0C222000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TSENS1",             0x0C223000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TSENS0_TM",          0x0C263000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PSHOLD",             0x0C264000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TSENS1_TM",          0x0C265000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PMIC ARB SPMI",      0x0C400000, 0x02800000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"SMMU",               0x15000000, 0x000D0000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"APSS_GIC500_GICD",   0x17A00000, 0x00010000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"APSS_GIC500_GICR",   0x17A60000, 0x00100000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"APSS_IPC_INT",       0x17C00000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"APSS_WDT_TMR1",      0x17C10000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"QTIMER",             0x17C20000, 0x00110000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"APSS_RSC_APPS_RSCC", 0x18200000, 0x00030000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"SILVER_CLK_CTL",     0x18280000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"GOLD_CLK_CTL",       0x18282000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"L3_CLK_CTL",         0x18284000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"GOLDPLUS_CLK_CTL",   0x18286000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"SILVER_ACD",         0x18290000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"GOLD_ACD",           0x18292000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"L3_ACD",             0x18294000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"GOLDPLUS_ACD",       0x18296000, 0x00001000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"APSS_ACTPM_WRAP",    0x18300000, 0x000B0000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PCIE_1_WRAPPER_AXI", 0x40000000, 0x02000000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PCIE_0_WRAPPER_AXI", 0x60000000, 0x02000000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PCIE_2_WRAPPER_AXI", 0x68000000, 0x02000000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PCIE_3_WRAPPER_AXI", 0x70000000, 0x02000000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE}
};

VOID
GetMemoryMap (
  OUT EFI_MEMORY_REGION_DESCRIPTOR **MemoryDescriptor,
  OUT UINT8                         *MemoryDescriptorCount)
{
  *MemoryDescriptor      = gMemoryDescriptor;
  *MemoryDescriptorCount = ARRAY_SIZE (gMemoryDescriptor);
}
"""

CONFIG_MAP_INF = """\
[Defines]
  INF_VERSION                    = 0x00010005
  BASE_NAME                      = ConfigurationMapLib
  FILE_GUID                      = D7DDE228-62E4-43E3-B5A1-AB29FB9B0A35
  MODULE_TYPE                    = BASE
  VERSION_STRING                 = 1.0
  LIBRARY_CLASS                  = ConfigurationMapLib

[Sources]
  ConfigurationMapLib.c

[Packages]
  MdePkg/MdePkg.dec
  QcomPkg/QcomPkg.dec
"""

CONFIG_MAP_C = """\
#include <Library/ConfigurationMapLib.h>

STATIC
EFI_CONFIGURATION_ENTRY_DESCRIPTOR
gConfigurationDescriptor[] = {
  {"AllowNonPersistentVarsInRetail", 0x1},
  {"DDRInfoNotifyFlag", 0x0},
  {"DRAM_CLK_PERIOD_ADDR", 0x240BA050},
  {"DetectRetailUserAttentionHotkey", 0x0},
  {"DetectRetailUserAttentionHotkeyCode", 0x17},
  {"DloadCookieAddr", 0x1FD9000},
  {"DloadCookieValue", 0x30},
  {"EarlyInitCoreCnt", 0x1},
  {"EnableACPIFallback", 0x0},
  {"EnableDisplayImageFv", 0x0},
  {"EnableDisplayThread", 0x0},
  {"EnableLogFsSyncInRetail", 0x0},
  {"EnableMultiCoreFvDecompression", 0x0},
  {"EnableMultiThreading", 0x0},
  {"EnableOEMSetupAppInRetail", 0x0},
  {"EnableSDHCSwitch", 0x1},
  {"EnableShell", 0x1},
  {"EnableUefiSecAppDebugLogDump", 0x0},
  {"EnableUfsIOC", 0x1},
  {"EnableVariablePolicyEngine", 0x0},
  {"LoadAutoImageInPILFlag", 0x1},
  {"MaxCoreCnt", 0x8},
  {"MaxLogFileSize", 0x400000},
  {"NumActiveCores", 0x1},
  {"NumCpus", 0x8},
  {"PilSubsysDbgCookieAddr", 0x146AA6DC},
  {"PilSubsysDbgCookieVal", 0x53444247},
  {"PwrBtnShutdownFlag", 0x0},
  {"Sdc1GpioConfigOff", 0xA00},
  {"Sdc1GpioConfigOn", 0x1E92},
  {"Sdc2GpioConfigOff", 0xA00},
  {"Sdc2GpioConfigOn", 0x1E92},
  {"SecPagePoolCount", 0x800},
  {"SecurityFlag", 0xC4},
  {"SharedIMEMBaseAddr", 0x146AA000},
  {"ShmBridgememSize", 0xA00000},
  {"USBHS1_Config", 0x0},
  {"UefiMemUseThreshold", 0xE1},
  {"UfsSmmuConfigForOtherBootDev", 0x1},
  {"UsbFnIoRevNum", 0x10001}
};

VOID
GetConfigurationMap (
  OUT EFI_CONFIGURATION_ENTRY_DESCRIPTOR **ConfigurationDescriptor,
  OUT UINT8                               *ConfigurationDescriptorCount)
{
  *ConfigurationDescriptor      = gConfigurationDescriptor;
  *ConfigurationDescriptorCount = ARRAY_SIZE (gConfigurationDescriptor);
}
"""


def write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--uefi",
        type=Path,
        default=Path(os.environ["UNIXNU_UEFI"]) if "UNIXNU_UEFI" in os.environ else None,
        help="Path to Mu-Silicium (UniXNU/uefi-sheng) checkout",
    )
    parser.add_argument(
        "--device",
        type=Path,
        default=Path(os.environ["UNIXNU_DEVICE"])
        if "UNIXNU_DEVICE" in os.environ
        else Path(__file__).resolve().parent.parent,
        help="Path to UniXNU-pclm10 device repository",
    )
    parser.add_argument(
        "--tree",
        type=Path,
        default=Path.cwd(),
        help="Path to UniXNU root repository (where build/pclm10/pclm10.dtb is staged)",
    )
    args = parser.parse_args()

    device_dir = args.device.resolve()
    src_dtb = device_dir / "dts" / "oppo-pclm10-19081.dtb"
    if not src_dtb.is_file():
        raise SystemExit(f"Missing device DTB: {src_dtb}")

    # Stage build/pclm10/pclm10.dtb under the active build tree
    tree_dir = args.tree.resolve()
    dst_dtb = tree_dir / "build" / "pclm10" / "pclm10.dtb"
    dst_dtb.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src_dtb, dst_dtb)
    print(f"Staged DTB: {dst_dtb} ({dst_dtb.stat().st_size} bytes)")

    if args.uefi is not None:
        uefi_dir = args.uefi.resolve()
        if not (uefi_dir / "Resources" / "Scripts" / "mkbootimg.py").is_file():
            raise SystemExit(f"Not a Mu-Silicium checkout: {uefi_dir}")

        uefi_dtb = uefi_dir / "Resources" / "DTBs" / "pclm10.dtb"
        uefi_dtb.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src_dtb, uefi_dtb)

        write_text(uefi_dir / "Resources" / "Configs" / "pclm10.toml", PCLM10_TOML)
        pkg_dir = uefi_dir / "Platforms" / "OPPO" / "pclm10Pkg"
        write_text(pkg_dir / "pclm10.dec", PCLM10_DEC)
        write_text(pkg_dir / "pclm10.dsc", PCLM10_DSC)
        write_text(pkg_dir / "pclm10.fdf", PCLM10_FDF)
        write_text(pkg_dir / "Include" / "APRIORI.inc", PCLM10_APRIORI)
        write_text(pkg_dir / "Include" / "DXE.inc", PCLM10_DXE)
        write_text(pkg_dir / "Library" / "MemoryMapLib" / "MemoryMapLib.inf", MEMORY_MAP_INF)
        write_text(pkg_dir / "Library" / "MemoryMapLib" / "MemoryMapLib.c", MEMORY_MAP_C)
        write_text(
            pkg_dir / "Library" / "ConfigurationMapLib" / "ConfigurationMapLib.inf",
            CONFIG_MAP_INF,
        )
        write_text(
            pkg_dir / "Library" / "ConfigurationMapLib" / "ConfigurationMapLib.c",
            CONFIG_MAP_C,
        )
        print(f"Injected pclm10Pkg into {uefi_dir}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
