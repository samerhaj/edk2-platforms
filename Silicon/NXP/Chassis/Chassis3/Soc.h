/** Soc.h
*  Header defining the Base addresses, sizes, flags etc for chassis 1
*
*  Copyright 2017-2019 NXP
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
#define NUM_CC_PLLS                 FixedPcdGet8 (PcdNumCcPlls) // 3 LS1028A
                                                                // 6 LS208x
#define FSL_CLUSTER_CLOCKS          { 1, 1, 4, 4 } // LS208x
                                                   // { 1, 1 } LS1028A
#define TP_CLUSTER_EOC_MASK         0x80000000      /* Mask for End of clusters */
#define CHECK_CLUSTER(Cluster)      ((Cluster & TP_CLUSTER_EOC_MASK) != TP_CLUSTER_EOC_MASK)

// RCW SERDES MACRO
#define RCWSR_INDEX                  28
#define RCWSR_SRDS1_PRTCL_MASK       FixedPcdGet32 (PcdCh3Srds1PrtclMask) // 0x00ff0000 LS208x
                                                                          // 0xffff0000 LS1028A
#define RCWSR_SRDS1_PRTCL_SHIFT      16
#define RCWSR_SRDS2_PRTCL_MASK       0xff000000
#define RCWSR_SRDS2_PRTCL_SHIFT      24

// SMMU Defintions
#define SMMU_BASE_ADDR               0x05000000
#define SMMU_REG_SCR0                (SMMU_BASE_ADDR + 0x0)
#define SMMU_REG_SACR                (SMMU_BASE_ADDR + 0x10)
#define SMMU_REG_IDR1                (SMMU_BASE_ADDR + 0x24)
#define SMMU_REG_NSCR0               (SMMU_BASE_ADDR + 0x400)
#define SMMU_REG_NSACR               (SMMU_BASE_ADDR + 0x410)

#define SCFG_BASE_ADDR               0x1FC0000
#define SCFG_USB3PRM1CR              0x000
#define SCFG_USB3PRM2CR              0x004
#define SCFG_USB3PRM1CR_INIT         0x27672b2a
#define USB_TXVREFTUNE               0x9
#define USB_SQRXTUNE                 0xFC7FFFFF
#define USB_PCSTXSWINGFULL           0x47
#define DCSR_BASE                    0x700000000ULL
#define DCSR_USB_PHY1                0x4600000
#define DCSR_USB_PHY2                0x4610000
#define DCSR_USB_PHY_RX_OVRD_IN_HI   0x1006
#define USB_PHY_RX_EQ_VAL_1          0x0000
#define USB_PHY_RX_EQ_VAL_2          0x0080
#define USB_PHY_RX_EQ_VAL_3          0x0380
#define USB_PHY_RX_EQ_VAL_4          0x0b80

/* Interrupt Sampling Control */
#define INT_SAMPLING_CTRL_BASE        0x01F70000
#define IRQCR_OFFSET                  0x14


#define SACR_PAGESIZE_MASK           0x00010000
#define SCR0_CLIENTPD_MASK           0x00000001
#define SCR0_USFCFG_MASK             0x00000400

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
  UINT8     Res180[0x200-0x180];
  UINT32    ScratchRw[32];    // Scratch Read/Write
  UINT8     Res280[0x300-0x280];
  UINT32    ScratchW1R[4];    // Scratch Read (Write once)
  UINT8     Res310[0x400-0x310];
  UINT32    BootLocPtrL;      // Low addr : Boot location pointer
  UINT32    BootLocPtrH;      // High addr : Boot location pointer
  UINT8     Res408[0x500-0x408];
  UINT8     Res500[0x740-0x500];
  UINT32    TpItyp[64];
  struct {
    UINT32     Upper;
    UINT32     Lower;
  } TpCluster[3];
  UINT8      Res858[0x900-0x858];
  UINT32     QspiCR1;
  UINT8      Res904[0x1000-0x904];
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
  } HwnCsr[3];
  UINT8      Res60[0x80-0x60];
  struct {
    UINT32     Gsr;             // core cluster n clock general status
    UINT8      Res84[0xa0-0x84];
  } PllnGsr[3];
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

#endif /* __SOC_H__ */
