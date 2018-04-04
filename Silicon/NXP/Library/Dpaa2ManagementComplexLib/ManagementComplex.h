/** ManagementComplex.h
  DPAA2 Management Complex private declarations

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __MANAGEMENT_COMPLEX_H__
#define __MANAGEMENT_COMPLEX_H__

#include <Bitops.h>
#include <Library/Dpaa2McIo.h>
#include <Uefi.h>

#define DPAA2_MC_CCSR_BASE_ADDR 0x8340000ULL
#define GSR_FS_MASK             0x3fffffff
#define DPAA2_MC_PORTAL_STRIDE  0x10000

#define DPAA2_MC_PORTAL_ADDR(PortalId) \
  ((EFI_PHYSICAL_ADDRESS)FixedPcdGet64 (PcdDpaa2McPortalBaseAddr) + \
   (PortalId) * DPAA2_MC_PORTAL_STRIDE)

#define MC_PORTAL_OFFSET_TO_PORTAL_ID(PortalOffset) \
  ((PortalOffset) / DPAA2_MC_PORTAL_STRIDE)

#define MC_RAM_BASE_ADDR_ALIGNMENT  (512UL * 1024 * 1024)
#define MC_RAM_BASE_ADDR_ALIGNMENT_MASK \
        (~(MC_RAM_BASE_ADDR_ALIGNMENT - 1))

#define MC_RAM_SIZE_ALIGNMENT      (256UL * 1024 * 1024)

#define MC_LOG_MAGIC_WORD_PREFIX_MASK  \
        (((UINT32)'M' << 24) |  ((UINT32)'C' << 16))

#define MC_FIXED_SIZE_512MB 0x20000000

// MC_ADDR should be the top 512MB from memory region allocated for MC
#if FixedPcdGetBool (PcdMcHighMemSupport)
#define MC_ADDR     FixedPcdGet64 (PcdDram2BaseAddr) + FixedPcdGet64 (PcdDramMemSize) - MC_FIXED_SIZE_512MB
#else
#define MC_ADDR     FixedPcdGet64 (PcdDram1BaseAddr) - FixedPcdGet64 (PcdDpaa2McRamSize)
#endif

/**
 * DPAA2 Management complex CCSR registers
 */
typedef volatile struct _DPAA2_MC_CCSR {
  UINT32 Gcr1;
  UINT32 Reserved1;
  UINT32 Gsr;
# define GCR1_P1_STOP      BIT(31)
# define GCR1_P2_STOP      BIT(30)
# define GCR1_P1_DE_RST    BIT(23)
# define GCR1_P2_DE_RST    BIT(22)
# define GCR1_M1_DE_RST    BIT(15)
# define GCR1_M2_DE_RST    BIT(14)
# define GCR1_M_ALL_DE_RST  (GCR1_M1_DE_RST | GCR1_M2_DE_RST)

  UINT32 Reserved2;
  UINT32 Sicbalr;
  UINT32 Sicbahr;
  UINT32 Sicapr;
  UINT32 Reserved3;
  UINT32 Mcfbalr;
  UINT32 Mcfbahr;
  UINT32 Mcfapr;
  UINT32 Reserved4[0x2f1];
  UINT32 Psr;
  UINT32 Reserved5;
  UINT32 Brr[2];
  UINT32 Reserved6[0x80];
  UINT32 Error[];
} DPAA2_MC_CCSR;

/**
 * DPAA2 Management Complex in-RAM log header
 */
typedef volatile struct _DPAA2_MC_LOG_HEADER {
  /**
   * 32-bit little-endian magic word 'MC<log-version>'
   */
  UINT32 MagicWord;

  /**
   * Reserved word
   */
  UINT32 Reserved1;

  /**
   * 32-bit little-endian offset of the start of the buffer.
   * (Offset is from the beginning of the MC private memory block)
   */
  UINT32 BufStart;

  /**
   * 32-bit little-endian length of buffer (in bytes) - not including header
   */
  UINT32 BufLength;

  /**
   * 31-bit little-endian offset (from BufStart) of the byte after last
   * written byte.
   * The MSBit of this field indicates a buffer wraparound.
   */
  UINT32 LastByte;

  /**
   * Reserved space for future extensions
   */
  char Reserved2[44];
} DPAA2_MC_LOG_HEADER;

typedef enum _DPAA2_MC_LOG_LEVEL {
  MC_LOG_LEVEL_DEBUG =       0x01,
  MC_LOG_LEVEL_INFO =        0x02,
  MC_LOG_LEVEL_WARNING =     0x03,
  MC_LOG_LEVEL_ERROR =       0x04,
  MC_LOG_LEVEL_CRITICAL =    0x05,
  MC_LOG_LEVEL_ASSERT =      0x06,

  /*
   * Special value to indicate that the default
   * value in the DPC is to be used:
   */
  MC_LOG_LEVEL_DPC_DEFAULT = 0xff,
} DPAA2_MC_LOG_LEVEL;

/**
 * Control block for the DPPA2 Management Complex (MC)
 */
typedef struct _DPAA2_MANAGEMENT_COMPLEX {
  /**
   * Flag indicating if the MC has firmware has been booted
   */
  BOOLEAN McBooted;

  /**
   * Flag indicating if the MC DPL has been deployed
   */
  BOOLEAN McDplDeployed;

  /**
   * Last "wrap-around" flag value for the Mc log
   */
  BOOLEAN McLogLastWraparoundFlag;

  /**
   * Pointer to the MC's CCSR registers
   */
  DPAA2_MC_CCSR *McCcsrRegs;

  /**
   * Base physical address for the MC's private memory block
   */
  EFI_PHYSICAL_ADDRESS McPrivateMemoryBaseAddr;

  /**
   * MC's private memory block size in bytes
   */
  UINT32 McPrivateMemorySize;

  /**
   * Pointer to the MC's log in DRAM (located in the MC's private
   * memory block)
   */
  DPAA2_MC_LOG_HEADER *McLog;

  /**
   * Pointer to the beginning of the MC's log buffer
   */
  CHAR8 *McLogBufferStartPtr;

  /**
   * Pointer to the end of the MC's log buffer
   */
  CHAR8 *McLogBufferEnd;

  /**
   * MC boot status
   */
  EFI_STATUS McBootStatus;

  /**
   * MC command portal I/O object for the MC's root DPRC
   */
  DPAA2_MC_IO RootDprcMcIo;

  /**
   * MC's root DPRC handle
   */
  UINT16 RootDprcHandle;

} DPAA2_MANAGEMENT_COMPLEX;

/**
 * Supported storage sources for the MC firmware images
 */
typedef enum _DPAA2_MC_FW_SOURCE {
  MC_IMAGES_IN_NOR_FLASH =  0x01,
  MC_IMAGES_IN_QSPI_FLASH = 0x02,
} DPAA2_MC_FW_SOURCE;

VOID DumpMcLogTail (
  DPAA2_MANAGEMENT_COMPLEX *Mc,
  UINTN NumTailLines
  );

extern VOID CleanDcacheRange (
  UINT64 StartAddr,
  UINT64 EndAddr
  );

extern DPAA2_MANAGEMENT_COMPLEX gManagementComplex;

#endif /* __MANAGEMENT_COMPLEX_H__ */
