/** @file
  Header Defining The MMC Memory Controller Constants, Function Prototype, Structures Etc

  Copyright 2017 NXP

  This Program And The Accompanying Materials
  Are Licensed And Made Available Under The Terms And Conditions Of The BSD
  License Which Accompanies This Distribution. The Full Text Of The License
  May Be Found At
  Http://Opensource.Org/Licenses/Bsd-License.Php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __MMC_INTERNAL_H__
#define __MMC_INTERNAL_H__

#include <Library/DmaLib.h>

#define MIN_CLK_FREQUENCY          400000
#define MAX_CLK_FREQUENCY          52000000

#define ENABLE_CACHE_SNOOPING      0x00000040
#define CMD_STOP_TRANSMISSION      12

/**
  Timeout counter
**/
#define TIMEOUT                    10000
#define TRANSFER_TIMEOUT           10000000
/**
  Pre Div factor for DDR mode
**/
#define DIV_2                      2
#define DIV_1                      1

/**
  SDXC-Specific Constants
**/
#define SYSCTL                     0x0002e02c
#define SYSCTL_INITA               0x08000000
#define SYSCTL_TIMEOUT_MASK        0x000f0000
#define SYSCTL_CLOCK_MASK          0x0000fff0
#define SYSCTL_CKEN                0x00000008
#define SYSCTL_PEREN               0x00000004
#define SYSCTL_HCKEN               0x00000002
#define SYSCTL_IPGEN               0x00000001
#define SYSCTL_RSTA                0x01000000
#define SYSCTL_RSTC                0x02000000
#define SYSCTL_RSTD                0x04000000

/**
  Host Capabilities
**/
#define SDXC_HOSTCAPBLT_VS18       0x04000000
#define SDXC_HOSTCAPBLT_VS30       0x02000000
#define SDXC_HOSTCAPBLT_VS33       0x01000000
#define SDXC_HOSTCAPBLT_SRS        0x00800000
#define SDXC_HOSTCAPBLT_DMAS       0x00400000
#define SDXC_HOSTCAPBLT_HSS        0x00200000

/**
  VDD Voltage Range
**/
#define MMC_VDD_165_195            0x00000080    // VDD Voltage 1.65 - 1.95
#define MMC_VDD_20_21              0x00000100    // VDD Voltage 2.0 ~ 2.1
#define MMC_VDD_21_22              0x00000200    // VDD Voltage 2.1 ~ 2.2
#define MMC_VDD_22_23              0x00000400    // VDD Voltage 2.2 ~ 2.3
#define MMC_VDD_23_24              0x00000800    // VDD Voltage 2.3 ~ 2.4
#define MMC_VDD_24_25              0x00001000    // VDD Voltage 2.4 ~ 2.5
#define MMC_VDD_25_26              0x00002000    // VDD Voltage 2.5 ~ 2.6
#define MMC_VDD_26_27              0x00004000    // VDD Voltage 2.6 ~ 2.7
#define MMC_VDD_27_28              0x00008000    // VDD Voltage 2.7 ~ 2.8
#define MMC_VDD_28_29              0x00010000    // VDD Voltage 2.8 ~ 2.9
#define MMC_VDD_29_30              0x00020000    // VDD Voltage 2.9 ~ 3.0
#define MMC_VDD_30_31              0x00040000    // VDD Voltage 3.0 ~ 3.1
#define MMC_VDD_31_32              0x00080000    // VDD Voltage 3.1 ~ 3.2
#define MMC_VDD_32_33              0x00100000    // VDD Voltage 3.2 ~ 3.3
#define MMC_VDD_33_34              0x00200000    // VDD Voltage 3.3 ~ 3.4
#define MMC_VDD_34_35              0x00400000    // VDD Voltage 3.4 ~ 3.5
#define MMC_VDD_35_36              0x00800000    // VDD Voltage 3.5 ~ 3.6

/**
  MMC Operating Modes
**/
#define MMC_MODE_HS                (1 << 0)
#define MMC_MODE_HS_52MHz          (1 << 1)
#define MMC_MODE_4_BIT             (1 << 2)
#define MMC_MODE_8_BIT             (1 << 3)
#define MMC_MODE_SPI               (1 << 4)
#define MMC_MODE_HC                (1 << 5)
#define MMC_MODE_DDR_52MHz         (1 << 6)

#define MMC_DATA_READ              1
#define MMC_DATA_WRITE             2

/**
  Maximum Block Size for MMC
**/
#define MMC_MAX_BLOCK_LEN          512

#define WML_RD_MAX                 0x10
#define WML_WR_MAX                 0x80
#define WML_RD_MAX_VAL             0x00
#define WML_WR_MAX_VAL             0x80
#define WML_RD_MASK                0xff
#define WML_WR_MASK                0xff0000

#define XFERTYPE                   0x0002e00c
#define XFERTYPE_CMD(X)            ((X & 0x3f) << 24)
#define XFERTYPE_CMDTYP_NORMAL     0x0
#define XFERTYPE_CMDTYP_SUSPEND    0x00400000
#define XFERTYPE_CMDTYP_RESUME     0x00800000
#define XFERTYPE_CMDTYP_ABORT      0x00c00000
#define XFERTYPE_DPSEL             0x00200000
#define XFERTYPE_CICEN             0x00100000
#define XFERTYPE_CCCEN             0x00080000
#define XFERTYPE_RSPTYP_NONE       0
#define XFERTYPE_RSPTYP_136        0x00010000
#define XFERTYPE_RSPTYP_48         0x00020000
#define XFERTYPE_RSPTYP_48_BUSY    0x00030000
#define XFERTYPE_MSBSEL            0x00000020
#define XFERTYPE_DTDSEL            0x00000010
#define XFERTYPE_AC12EN            0x00000004
#define XFERTYPE_BCEN              0x00000002
#define XFERTYPE_DMAEN             0x00000001

/**
  IRQSTAT bits
**/
#define IRQSTAT                    (0x0002e030)
#define IRQSTATE_DMAE              (0x10000000)
#define IRQSTATE_AC12E             (0x01000000)
#define IRQSTATE_DEBE              (0x00400000)
#define IRQSTATE_DCE               (0x00200000)
#define IRQSTATE_DTOE              (0x00100000)
#define IRQSTATE_CIE               (0x00080000)
#define IRQSTATE_CEBE              (0x00040000)
#define IRQSTATE_CCE               (0x00020000)
#define IRQSTATE_CTOE              (0x00010000)
#define IRQSTATE_CINT              (0x00000100)
#define IRQSTATE_CRM               (0x00000080)
#define IRQSTATE_CINS              (0x00000040)
#define IRQSTATE_BRR               (0x00000020)
#define IRQSTATE_BWR               (0x00000010)
#define IRQSTATE_DINT              (0x00000008)
#define IRQSTATE_BGE               (0x00000004)
#define IRQSTATE_TC                (0x00000002)
#define IRQSTATE_CC                (0x00000001)

#define CMD_ERR                    (IRQSTATE_CIE | IRQSTATE_CEBE | IRQSTATE_CCE)
#define DATA_ERR                   (IRQSTATE_DEBE | IRQSTATE_DCE | IRQSTATE_DTOE | \
                                   IRQSTATE_DMAE)
#define DATA_COMPLETE              (IRQSTATE_TC | IRQSTATE_DINT)

#define IRQSTATE_EN                (0x0002e034)
#define IRQSTATE_EN_DMAE           (0x10000000)
#define IRQSTATE_EN_AC12E          (0x01000000)
#define IRQSTATE_EN_DEBE           (0x00400000)
#define IRQSTATE_EN_DCE            (0x00200000)
#define IRQSTATE_EN_DTOE           (0x00100000)
#define IRQSTATE_EN_CIE            (0x00080000)
#define IRQSTATE_EN_CEBE           (0x00040000)
#define IRQSTATE_EN_CCE            (0x00020000)
#define IRQSTATE_EN_CTOE           (0x00010000)
#define IRQSTATE_EN_CINT           (0x00000100)
#define IRQSTATE_EN_CRM            (0x00000080)
#define IRQSTATE_EN_CINS           (0x00000040)
#define IRQSTATE_EN_BRR            (0x00000020)
#define IRQSTATE_EN_BWR            (0x00000010)
#define IRQSTATE_EN_DINT           (0x00000008)
#define IRQSTATE_EN_BGE            (0x00000004)
#define IRQSTATE_EN_TC             (0x00000002)
#define IRQSTATE_EN_CC             (0x00000001)

#define PRSSTATE                   (0x0002e024)
#define PRSSTATE_DAT0              (0x01000000)
#define PRSSTATE_CLSL              (0x00800000)
#define PRSSTATE_WPSPL             (0x00080000)
#define PRSSTATE_CDPL              (0x00040000)
#define PRSSTATE_CINS              (0x00010000)
#define PRSSTATE_BREN              (0x00000800)
#define PRSSTATE_BWEN              (0x00000400)
#define PRSSTATE_DLA               (0x00000004)
#define PRSSTATE_CICHB             (0x00000002)
#define PRSSTATE_CIDHB             (0x00000001)

#define PRCTL                      0x0002e028
#define PRCTL_INIT                 0x00000020
#define PRCTL_DTW_4                0x00000002
#define PRCTL_DTW_8                0x00000004
#define PRCTL_BE                   0x00000030

typedef struct {
  UINT32 Dsaddr;         // SDMA System Address Register
  UINT32 Blkattr;        // Block Attributes Register
  UINT32 CmdArg;         // Command Argument Register
  UINT32 Xfertype;       // Transfer Type Register
  UINT32 Rspns0;         // Command Response 0 Register
  UINT32 Rspns1;         // Command Response 1 Register
  UINT32 Rspns2;         // Command Response 2 Register
  UINT32 Rspns3;         // Command Response 3 Register
  UINT32 Datport;        // Buffer Data Port Register
  UINT32 Prsstat;        // Present State Register
  UINT32 Proctl;         // Protocol Control Register
  UINT32 Sysctl;         // System Control Register
  UINT32 Irqstat;        // Interrupt Status Register
  UINT32 Irqstaten;      // Interrupt Status Enable Register
  UINT32 Irqsigen;       // Interrupt Signal Enable Register
  UINT32 Autoc12err;     // Auto CMD Error Status Register
  UINT32 Hostcapblt;     // Host Controller Capabilities Register
  UINT32 Wml;            // Watermark Level Register
  UINT32 Mixctrl;        // for USDHC
  CHAR8  Reserved1[4];   // Reserved
  UINT32 Fevt;           // Force Event Register
  UINT32 Admaes;         // ADMA Error Status Register
  UINT32 Adsaddr;        // ADMA System Address Register
  CHAR8  Reserved2[100]; // Reserved
  UINT32 VendorSpec;     //Vendor Specific Register
  CHAR8  Reserved3[56];  // Reserved
  UINT32 Hostver;        // Host Controller Version Register
  CHAR8  Reserved4[4];   // Reserved
  UINT32 Dmaerraddr;     // DMA Error Address Register
  CHAR8  Reserved5[4];   // Reserved
  UINT32 Dmaerrattr;     // DMA Error Attribute Register
  CHAR8  Reserved6[4];   // Reserved
  UINT32 Hostcapblt2;    // Host Controller Capabilities Register 2
  CHAR8  Reserved7[8];   // Reserved
  UINT32 Tcr;            // Tuning Control Register
  CHAR8  Reserved8[28];  // Reserved
  UINT32 Sddirctl;       // SD Direction Control Register
  CHAR8  Reserved9[712]; // Reserved
  UINT32 Scr;            // SDXC Control Register
} SDXC_REGS;

typedef struct {
  VOID *DmaAddr;
  UINTN Bytes;
  VOID *Mapping;
  DMA_MAP_OPERATION MapOperation;
} DMA_DATA;

typedef struct {
  UINT32 SdhcClk;
  UINT32 HostCaps;
  UINT32 HasInit;
  UINT32 FMin;
  UINT32 FMax;
  UINT32 BusWidth;
  UINT32 Clock;
  UINT32 CardCaps;
  INT32  DdrMode;
} MMC;

/**
  Function to Initialize MMC host controller

  @param  Mmc       Pointer to MMC data structure

**/
EFI_STATUS
SdxcInit (
  IN  MMC    *Mmc
  );

/**
  Function to reset MMC host controller

  @param   Regs     Pointer to MMC host Controller

**/
VOID
SdxcReset (
  IN  SDXC_REGS *Regs
  );

/**
  Function to create dma map for read/write operation

  @param   DmaData  Pointer to Dma data Structure

  @retval           Address of dma map

**/
VOID *
GetDmaBuffer (
  IN  DMA_DATA  *DmaData
  );

/**
  Function to free dma map

  @param   DmaData  Pointer to Dma data Structure

  @retval           Address of dma map

**/
EFI_STATUS
FreeDmaBuffer (
  IN  DMA_DATA  *DmaData
  );

/**
  Function to set up MMC data (timeout value,watermark level,
  system address,block attributes etc.)

  @param   Data     Pointer to MMC data

**/
EFI_STATUS
SdxcSetupData (
  IN  SD_DATA   *Data
  );

/**
  Function to select the transfer type flags depending upon given
  command and data packet

  @param   Cmd      Pointer to MMC command
  @param   Data     Pointer to MMC data

  @retval           Returns the XFERTYPE flags

**/
UINT32
SdxcXfertype (
  IN  SD_CMD    *Cmd,
  IN  SD_DATA   *Data
  );

/**
  Function to peform reset of MMC command and data

**/
VOID
ResetCmdFailedData (
  IN  SDXC_REGS *Regs,
  IN  UINT8     Data
  );

/**
  Function to set MMC host controller bus width

  @param  Mmc       Pointer to MMC data structure
  @param  BWidth    Bus width to be set

**/
VOID
SdxcSetBusWidth (
  IN  MMC       *Mmc,
  IN  UINT32    BWidth
  );

/**
  Function to read from MMC depending upon pcd

  @param   Address  MMC register to read

  @retval           Read Value from register

**/
UINT32
EFIAPI
MmcRead (
  IN  UINTN     Address
  );

/**
  Function to write on MMC depeding upon pcd

  @param   Address  MMC register to write

**/
UINT32
EFIAPI
MmcWrite (
  IN  UINTN     Address,
  IN  UINT32    Value
  );

/**
  Function to call MmioAndThenOr32 depending upon pcd

  @param   Address  MMC register
  @param   AndData  The value to AND with the read value from the MMC register
  @param   OrData   The value to OR with the result of the AND operation.

  @retval           Value written back to register

**/
UINT32
EFIAPI
MmcAndThenOr (
  IN  UINTN     Address,
  IN  UINT32    AndData,
  IN  UINT32    OrData
  );

/**
  Function to call MmioOr32 depending upon pcd

  @param   Address  MMC register
  @param   OrData   The value to OR with the result of the AND operation.

  @retval           Value written back to register

**/
UINT32
EFIAPI
MmcOr (
  IN  UINTN     Address,
  IN  UINT32    OrData
  );

/**
  Function to call MmioAnd32 depending upon pcd

  @param   Address  MMC register
  @param   AndData  The value to AND with the read value from the MMC register

  @retval           Value written back to register

**/
UINT32
EFIAPI
MmcAnd (
  IN  UINTN     Address,
  IN  UINT32    AndData
  );

/**
  Function to do MMC read/write transfer using DMA and checks
  whether transfer is completed or not

**/
EFI_STATUS
Transfer (
  IN  VOID
  );

/**
  Function to set MMC host controller system control register

  @param  Clock     Clock value for setting the register

**/
VOID
SetSysctl (
  UINT32 Clock
  );

#endif
