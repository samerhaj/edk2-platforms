/** @file

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __USB_HCD__
#define __USB_HCD__

/* Global constants */
#define DWC3_GSNPSID_MASK                      0xffff0000
#define DWC3_SYNOPSIS_ID                       0x55330000
#define DWC3_RELEASE_MASK                      0xffff
#define DWC3_REG_OFFSET                        0xC100
#define DWC3_RELEASE_190a                      0x190a

/* Global Configuration Register */
#define DWC3_GCTL_U2RSTECN                     BIT(16)
#define DWC3_GCTL_PRTCAPDIR(n)                 ((n) << 12)
#define DWC3_GCTL_PRTCAP_HOST                  1
#define DWC3_GCTL_PRTCAP_OTG                   3
#define DWC3_GCTL_CORESOFTRESET                BIT(11)
#define DWC3_GCTL_SCALEDOWN(n)                 ((n) << 4)
#define DWC3_GCTL_SCALEDOWN_MASK               DWC3_GCTL_SCALEDOWN(3)
#define DWC3_GCTL_DISSCRAMBLE                  BIT(3)
#define DWC3_GCTL_DSBLCLKGTNG                  BIT(0)

/* Global HWPARAMS1 Register */
#define DWC3_GHWPARAMS1_EN_PWROPT(n)           (((n) & (3 << 24)) >> 24)
#define DWC3_GHWPARAMS1_EN_PWROPT_CLK          1

/* Global USB2 PHY Configuration Register */
#define DWC3_GUSB2PHYCFG_PHYSOFTRST            BIT(31)

/* Global USB3 PIPE Control Register */
#define DWC3_GUSB3PIPECTL_PHYSOFTRST           BIT(31)

/* Global Frame Length Adjustment Register */
#define GFLADJ_30MHZ_REG_SEL                   BIT(7)
#define GFLADJ_30MHZ(n)                        ((n) & 0x3f)
#define GFLADJ_30MHZ_DEFAULT                   0x20

/* Default to the FSL XHCI defines */
#define USB3_ENABLE_BEAT_BURST                 0xF
#define USB3_ENABLE_BEAT_BURST_MASK            0xFF
#define USB3_SET_BEAT_BURST_LIMIT              0xF00

typedef struct {
  UINT32 GEvntAdrLo;
  UINT32 GEvntAdrHi;
  UINT32 GEvntSiz;
  UINT32 GEvntCount;
} GEventBuffer;

typedef struct {
  UINT32 DDepCmdPar2;
  UINT32 DDepCmdPar1;
  UINT32 DDepCmdPar0;
  UINT32 DDepCmd;
} DPhysicalEndpoint;

typedef struct {
  UINT32 GSBusCfg0;
  UINT32 GSBusCfg1;
  UINT32 GTxThrCfg;
  UINT32 GRxThrCfg;
  UINT32 GCtl;
  UINT32 Res1;
  UINT32 GSts;
  UINT32 Res2;
  UINT32 GSnpsId;
  UINT32 GGpio;
  UINT32 GUid;
  UINT32 GUctl;
  UINT64 GBusErrAddr;
  UINT64 GPrtbImap;
  UINT32 GHwParams0;
  UINT32 GHwParams1;
  UINT32 GHwParams2;
  UINT32 GHwParams3;
  UINT32 GHwParams4;
  UINT32 GHwParams5;
  UINT32 GHwParams6;
  UINT32 GHwParams7;
  UINT32 GDbgFifoSpace;
  UINT32 GDbgLtssm;
  UINT32 GDbgLnmcc;
  UINT32 GDbgBmu;
  UINT32 GDbgLspMux;
  UINT32 GDbgLsp;
  UINT32 GDbgEpInfo0;
  UINT32 GDbgEpInfo1;
  UINT64 GPrtbImapHs;
  UINT64 GPrtbImapFs;
  UINT32 Res3[28];
  UINT32 GUsb2PhyCfg[16];
  UINT32 GUsb2I2cCtl[16];
  UINT32 GUsb2PhyAcc[16];
  UINT32 GUsb3PipeCtl[16];
  UINT32 GTxFifoSiz[32];
  UINT32 GRxFifoSiz[32];
  GEventBuffer GEvntBuf[32];
  UINT32 GHwParams8;
  UINT32 Res4[11];
  UINT32 GFLAdj;
  UINT32 Res5[51];
  UINT32 DCfg;
  UINT32 DCtl;
  UINT32 DEvten;
  UINT32 DSts;
  UINT32 DGCmdPar;
  UINT32 DGCmd;
  UINT32 Res6[2];
  UINT32 DAlepena;
  UINT32 Res7[55];
  DPhysicalEndpoint DPhyEpCmd[32];
  UINT32 Res8[128];
  UINT32 OCfg;
  UINT32 OCtl;
  UINT32 OEvt;
  UINT32 OEvtEn;
  UINT32 OSts;
  UINT32 Res9[3];
  UINT32 AdpCfg;
  UINT32 AdpCtl;
  UINT32 AdpEvt;
  UINT32 AdpEvten;
  UINT32 BcCfg;
  UINT32 Res10;
  UINT32 BcEvt;
  UINT32 BcEvten;
} Dwc3;

#endif
