/** FpgaLib.h
*  Header defining the LS2160a Fpga specific constants (Base addresses, sizes, flags)
*
*  Copyright 2019 SolidRun
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

#ifndef __LX2160A_FPGA_H__
#define __LX2160A_FPGA_H__

typedef enum {
  CLK_66,
  CLK_83,
  CLK_100,
  CLK_125,
  CLK_133
} SYSTEM_CLOCK;

/*
 * FPGA register set of LS2160ACEX7 board-specific.
 */
typedef struct {
  UINT8 Id;           // ID value uniquely identifying each QorIQ board type
  UINT8 Arch;         // Board Version
  UINT8 Ver;          // FPGA Version
  UINT8 Model;        // Programming Model
  UINT8 Minor;        // Minor Revision Number
  UINT8 CtlSys;
  UINT8 Aux;
  UINT8 ClkSpd;
  UINT8 StatDut;
  UINT8 StatSys;
  UINT8 StatAlrm;
  UINT8 Present;
  UINT8 Present2;
  UINT8 RcwCtl;
  UINT8 CtlLed;
  UINT8 I2cBlk;
  UINT8 RcfgCtl;
  UINT8 RcfgSt;
  UINT8 DcmAd;
  UINT8 DcmDa;
  UINT8 Dcmd;
  UINT8 Dmsg;
  UINT8 Gdc;
  UINT8 Gdd;
  UINT8 Dmack;
  UINT8 Res1[6];
  UINT8 Watch;
  UINT8 PwrCtl[2];
  UINT8 Res2[2];
  UINT8 PwrStat[4];
  UINT8 Res3[8];
  UINT8 ClkSpd2[2];
  UINT8 Res4[2];
  UINT8 Sclk[3];
  UINT8 Res5;
  UINT8 Dclk[3];
  UINT8 Res6;
  UINT8 ClkDspd[3];
  UINT8 Res7;
  UINT8 RstCtl;
  UINT8 RstStat;
  UINT8 RstRsn;
  UINT8 RstFrc[2];
  UINT8 Res8[11];
  UINT8 BrdCfg[16];
  UINT8 DutCfg[16];
  UINT8 RcwAd[2];
  UINT8 RcwData;
  UINT8 Res9[5];
  UINT8 PostCtl;
  UINT8 PostStat;
  UINT8 PostDat[2];
  UINT8 Pid[4];
  UINT8 GpioIo[4];
  UINT8 GpioDir[4];
  UINT8 Res10[20];
  UINT8 RjtagCtl;
  UINT8 RjtagDat;
  UINT8 Res11[2];
  UINT8 TrigSrc[4];
  UINT8 TrigDst[4];
  UINT8 TrigStat;
  UINT8 Res12[3];
  UINT8 TrigCtr[4];
  UINT8 Res13[16];
  UINT8 ClkFreq[6];
  UINT8 ResC6[8];
  UINT8 ClkBase[2];
  UINT8 ResD0[8];
  UINT8 Cms[2];
  UINT8 ResC0[6];
  UINT8 Aux2[4];
  UINT8 Res14[10];
  UINT8 AuxAd;
  UINT8 AuxDa;
  UINT8 Res15[16];
} FFPGA_REG_SET;

/**
   Function to read FPGA register.
**/
UINT8
FpgaRead (
  UINTN  Reg
  );

/**
   Function to write FPGA register.
**/
VOID
FpgaWrite (
  UINTN  Reg,
  UINT8  Value
  );

/**
   Function to initialize FPGA timings.
**/
VOID
FpgaInit (
  VOID
  );

/**
   Function to get system clock frequency.
**/
UINTN
GetBoardSysClk (
  VOID
  );

/**
   Function to print board personality.
**/
VOID
PrintBoardPersonality (
  VOID
  );

#define QIXIS_BASE_I2C_BUS            0
#define QIXIS_BASE_I2C_ADR            0x66

//SYSCLK
#define FPGA_CLK_MASK            0x03     // FPGA Clock Mask
#define SYSCLK_100_MHZ           100000000

#define FPGA_VBANK_MASK          0x07
#define FPGA_CS_MASK             0x08

#define FPGA_READ(Reg)           FpgaRead (OFFSET_OF (FPGA_REG_SET, Reg))
#define FPGA_WRITE(Reg, Value)   FpgaWrite (OFFSET_OF (FPGA_REG_SET, Reg), Value)

#endif // __LS2160A_FPGA_H__
