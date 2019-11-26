/** Soc.h
*  Header defining the Base addresses, sizes, flags etc for chassis 1
*
*  Copyright 2018-2019 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#ifndef __SOC_H__
#define __SOC_H__

#define MAX_CPUS                    16
#define FSL_CLK_GRPA_ADDR           0x01300000
#define FSL_CLK_GRPB_ADDR           0x01310000
#define NUM_CC_PLLS                 4

#define FSL_CLUSTER_CLOCKS          { 0, 0, 0, 0, 2, 2, 2, 2} /* LX2160 */
#define TP_CLUSTER_EOC_MASK         0x80000000      /* Mask for End of clusters */
#define CHECK_CLUSTER(Cluster)      ((Cluster & TP_CLUSTER_EOC_MASK) != TP_CLUSTER_EOC_MASK)

// RCW SERDES MACRO
#define RCWSR_INDEX                  28
#define RCWSR_SRDS1_PRTCL_MASK       0x001f0000
#define RCWSR_SRDS1_PRTCL_SHIFT      16
#define RCWSR_SRDS2_PRTCL_MASK       0x03E00000
#define RCWSR_SRDS2_PRTCL_SHIFT      21
#define RCWSR_SRDS3_PRTCL_MASK       0x7C000000
#define RCWSR_SRDS3_PRTCL_SHIFT      26

// SMMU Defintions
#define SMMU_BASE_ADDR               0x05000000
#define SMMU_REG_SCR0                (SMMU_BASE_ADDR + 0x0)
#define SMMU_REG_SACR                (SMMU_BASE_ADDR + 0x10)
#define SMMU_REG_IDR1                (SMMU_BASE_ADDR + 0x24)
#define SMMU_REG_NSCR0               (SMMU_BASE_ADDR + 0x400)
#define SMMU_REG_NSACR               (SMMU_BASE_ADDR + 0x410)

/* Interrupt Sampling Control */
#define INT_SAMPLING_CTRL_BASE        0x01F70000
#define IRQCR_OFFSET                  0x14

#define SACR_PAGESIZE_MASK           0x00010000
#define SCR0_CLIENTPD_MASK           0x00000001
#define SCR0_USFCFG_MASK             0x00000400

#define DDR_GEN2_PHY_MULTIPLIER      4
typedef struct {
  UINTN FreqProcessor[MAX_CPUS];
  UINTN FreqSystemBus;
  UINTN FreqDdrBus;
  UINTN FreqDdrBus2;
  UINTN FreqLocalBus;
  UINTN FreqSdhc;
  UINTN FreqFman[1];
  UINTN FreqQman;
  UINTN FreqPme;
}SYS_INFO;

///
/// Device Configuration and Pin Control
///
typedef struct {
  UINT32    PorSr1;           // POR status register 1
  UINT32    PorSr2;           // POR status register 2
  UINT8     Res008[0x20-0x8];
  UINT32    GppOrCr1;         // General-purpose POR configuration register
  UINT32    GppOrCr2;         // General-purpose POR configuration register 2
  UINT32    DcfgFuseSr;       // Fuse status register */
  UINT32    GppOrCr3;
  UINT32    GppOrCr4;
  UINT8     Res034[0x70-0x34];
  UINT32    DevDisr;          // Device disable control register
  UINT32    DevDisr2;         // Device disable control register 2
  UINT32    DevDisr3;         // Device disable control register 3
  UINT32    DevDisr4;         // Device disable control register 4
  UINT32    DevDisr5;         // Device disable control register 5
  UINT32    DevDisr6;         // Device disable control register 6
  UINT32    DevDisr7;         // Device disable control register 7
  UINT8     Res08c[0x90-0x8c];
  UINT32    CoreDisrUpper;    // CORE DISR Uppper for support of 64 cores
  UINT32    CoreDisrLower;    // CORE DISR Lower for support of 64 cores
  UINT8     Res098[0xa0-0x98];
  UINT32    Pvr;              // Processor version
  UINT32    Svr;              // System version
  UINT32    Mvr;              // Manufacturing version
  UINT8     Res0ac[0x100-0xac];
  UINT32    RcwSr[32];        // Reset control word status
#define CHASSIS3_RCWSR_0_SYS_PLL_RAT_SHIFT    2
#define CHASSIS3_RCWSR_0_SYS_PLL_RAT_MASK     0x1f
#define CHASSIS3_RCWSR_0_MEM_PLL_RAT_SHIFT    10
#define CHASSIS3_RCWSR_0_MEM_PLL_RAT_MASK     0x3f
#define CHASSIS3_RCWSR_0_MEM2_PLL_RAT_SHIFT   18
#define CHASSIS3_RCWSR_0_MEM2_PLL_RAT_MASK    0x3f
#define CHASSIS3_RCWSR_0_MEM_PLL_CFG_SHIFT    8
#define CHASSIS3_RCWSR_0_MEM_PLL_CFG_MASK     0x3
  UINT8     Res180[0x200-0x180];
  UINT32    ScratchRw[32];    // Scratch Read/Write
  UINT8     Res280[0x300-0x280];
  UINT32    ScratchW1R[4];    // Scratch Read (Write once)
  UINT8     Res310[0x400-0x310];
  UINT32    BootLocPtrL;      // Low addr : Boot location pointer
  UINT32    BootLocPtrH;      // High addr : Boot location pointer
  UINT8     Res408[0x520-0x408];
  UINT32    Usb1Amqr;
  UINT32    Usb2Amqr;
  UINT8     Res528[0x530-0x528];
  UINT32    Sdmm1Amqr;
  UINT32    Sdmm2Amqr;
  UINT8     Res538[0x550-0x538];
  UINT32    Sata1Amqr;
  UINT32    Sata2Amqr;
  UINT32    Sata3Amqr;
  UINT32    Sata4Amqr;
  UINT8     Res560[0x740-0x560];
  UINT32    TpItyp[64];
  struct {
    UINT32     Upper;
    UINT32     Lower;
  } TpCluster[8];
  UINT8     Res880[0x900-0x880];
  UINT32    FlexSPICR1;
  UINT8     Res904[0x1000-0x904];
} CCSR_GUR;

///
/// Clocking
///
typedef struct {
  struct {
    UINT32 Csr;                 // core cluster n clock control status
    UINT8  Res04[0x20-0x04];
  } ClkCnCsr[8];
} CCSR_CLT_CTRL;

///
/// Clock Cluster
///
typedef struct {
  struct {
    UINT8      Res00[0x10];
    UINT32     Csr;             // core cluster n clock control status
    UINT8      Res14[0x20-0x14];
  } HwnCsr[4];
  struct {
    UINT32     Gsr;             // core cluster n clock general status
    UINT8      Res84[0xa0-0x84];
  } PllnGsr[2];
  UINT8      Rese0[0x100-0xe0];
} CCSR_CLK_CLUSTER;

VOID
GetSysInfo (
  OUT SYS_INFO *
  );

UINT32
EFIAPI
GurRead (
  IN  UINTN     Address
  );

VOID
EFIAPI
GurWrite (
  IN  UINTN    Address,
  IN  UINT32   Value
  );

#endif /* __SOC_H__ */
