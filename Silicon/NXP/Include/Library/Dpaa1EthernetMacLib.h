/** Dpaa1EthernetMacLib.h
  DPAA1 Ethernet MAC library interface

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __DPAA1_ETHERTNET_MAC_LIB_H__
#define __DPAA1_ETHERTNET_MAC_LIB_H__

#include <Library/BaseLib.h>
#include <Library/Dpaa1EthernetPhyLib.h>
#include <Library/UefiLib.h>
#include <Uefi.h>

#define PARAM_MODE_GLOBAL		0x20000000
#define PARAM_MODE_GRACEFUL_STOP	0x00800000

/* COMMAND_CONFIG - command and configuration register */
#define MEMAC_CMD_CFG_RX_EN		0x00000002 /* MAC Rx path enable */
#define MEMAC_CMD_CFG_TX_EN		0x00000001 /* MAC Tx path enable */
#define MEMAC_CMD_CFG_RXTX_EN	(MEMAC_CMD_CFG_RX_EN | MEMAC_CMD_CFG_TX_EN)
#define MEMAC_CMD_CFG_NO_LEN_CHK	0x20000 /* Payload length check disable */
#define MEMAC_CMD_CFG_SWR		0x00001000 /* software rest, sel clearing bit */
#define MEMAC_CMD_CFG_CLR_STATS	0x00000004 /* statistics clearing bit */
#define MEMAC_CMD_CFG_PAD		BIT5 /*Frame padding removal in receive path enable*/

/* FMAN_BM_TCFG - Tx configuration */
#define FMAN_BM_TCFG_EN 		0x80000000 /* port is enabled to transmit data */
#define FMAN_BM_TCFG_IM 		0x01000000 /* independent mode enable */

/* FMAN_BM_TST - Tx status */
#define FMAN_BM_TST_BSY        0x80000000 /* Tx port is busy */

/* FMAN_BM_TFCA - Tx frame command attributes */
#define FMAN_BM_TFCA_ORDER     0x80000000
#define FMAN_BM_TFCA_MR_MASK   0x003f0000
#define FMAN_BM_TFCA_MR(x)     ((x << 16) & FMAN_BM_TFCA_MR_MASK)

/* FMAN_BM_TSTC - Tx statistics counters */
#define FMAN_BM_TSTC_EN        0x80000000

/* FMAN_BM_RCFG - Rx configuration */
#define FMAN_BM_RCFG_EN        0x80000000 /* port is enabled to receive data */
#define FMAN_BM_RCFG_FDOVR     0x02000000 /* frame discard override */
#define FMAN_BM_RCFG_IM        0x01000000 /* independent mode */

/* FMAN_BM_RST - Rx status */
#define FMAN_BM_RST_BSY        0x80000000 /* Rx port is busy */

/* FMAN_BM_RFCA - Rx frame command attributes */
#define FMAN_BM_RFCA_ORDER     0x80000000
#define FMAN_BM_RFCA_MR_MASK   0x003f0000
#define FMAN_BM_RFCA_MR(x)     ((x << 16) & FMAN_BM_RFCA_MR_MASK)

/* FMAN_BM_RSTC - Rx statistics */
#define FMAN_BM_RSTC_EN        0x80000000 /* statistics counters enable */

/* NIA - next invoked action */
#define NIA_ENG_RISC        0x00000000
#define NIA_ENG_MASK        0x007c0000

/* action code */
#define NIA_RISC_AC_CC      0x00000006
#define NIA_RISC_AC_IM_TX   0x00000008 /* independent mode Tx */
#define NIA_RISC_AC_IM_RX   0x0000000a /* independent mode Rx */
#define NIA_RISC_AC_HC      0x0000000c

/**
 * DPAA1 FMAN MEMAC IDs
 */
typedef enum _FMAN_MEMAC_ID {
  INVALID_FMAN_MEMAC_ID = 0x0,
  FM1_DTSEC_1,
  FM1_DTSEC_2,
  FM1_DTSEC_3,
  FM1_DTSEC_4,
  FM1_DTSEC_5,
  FM1_DTSEC_6,
  FM1_DTSEC_9 = 0x9,
  FM1_DTSEC_10,
  /*
   * New values must added above this entry
   */
  NUM_FMAN_MEMACS,
} FMAN_MEMAC_ID;

typedef struct _ENET_MAC {
  VOID *Base; /* MAC controller base address */
  VOID *PhyRegs;
} ENET_MAC;

typedef enum _FMAN_ETH_TYPE {
       FM_ETH_1G,
       FM_ETH_10G,
}FMAN_ETH_TYPE;

typedef struct _BMI_TX_PORT {
       UINT32 FmanBmTcfg;      /* Tx configuration */
       UINT32 FmanBmTst; 	 /* Tx status */
       UINT32 FmanBmTda; 	 /* Tx DMA attributes */
       UINT32 FmanBmTfp; 	 /* Tx FIFO parameters */
       UINT32 FmanBmTfed;      /* Tx frame end data */
       UINT32 FmanBmTicp;      /* Tx internal context parameters */
       UINT32 FmanBmTfne;      /* Tx frame next engine */
       UINT32 FmanBmTfca;      /* Tx frame command attributes */
       UINT32 FmanBmTcfqid;	 /* Tx confirmation frame queue ID */
       UINT32 FmanBmTfeqid;	 /* Tx error frame queue ID */
       UINT32 FmanBmTfene;     /* Tx frame enqueue next engine */
       UINT32 FmanBmTrlmts;	 /* Tx rate limiter scale */
       UINT32 FmanBmTrlmt;     /* Tx rate limiter */
       UINT32 Res0[0x73];
       UINT32 FmanBmTstc;       /* Tx statistics counters */
       UINT32 FmanBmTfrc;       /* Tx frame counter */
       UINT32 FmanBmTfdc;       /* Tx frames discard counter */
       UINT32 FmanBmTfledc;     /* Tx frame length error discard counter */
       UINT32 FmanBmTfufdc;     /* Tx frame unsupported format discard counter */
       UINT32 FmanBmTbdc;       /* Tx buffers deallocate counter */
       UINT32 Res1[0x1a];
       UINT32 FmanBmTpc;        /* Tx performance counters */
       UINT32 FmanBmTpcp;       /* Tx performance count parameters */
       UINT32 FmanBmTccn;       /* Tx cycle counter */
       UINT32 FmanBmTtuc;       /* Tx tasks utilization counter */
       UINT32 FmanBmTtcquc;     /* Tx transmit confirm queue utilization counter */
       UINT32 FmanBmTduc;       /* Tx DMA utilization counter */
       UINT32 FmanBmTfuc;       /* Tx FIFO utilization counter */
       UINT32 Res2[0x19];
       UINT32 FmanBmTdcfg;      /* Tx debug configuration */
} BMI_TX_PORT;

typedef struct _BMI_RX_PORT {
       UINT32 FmanBmRcfg;       /* Rx configuration */
       UINT32 FmanBmRst; 	   /* Rx status */
       UINT32 FmanBmRda;        /* Rx DMA attributes */
       UINT32 FmanBmRfp;        /* Rx FIFO parameters */
       UINT32 FmanBmRfed;       /* Rx frame end data */
       UINT32 FmanBmRicp;       /* Rx internal context parameters */
       UINT32 FmanBmRim; 	   /* Rx internal margins */
       UINT32 FmanBmRebm;       /* Rx external buffer margins */
       UINT32 FmanBmRfne;       /* Rx frame next engine */
       UINT32 FmanBmRfca;       /* Rx frame command attributes */
       UINT32 FmanBmRfpne;      /* Rx frame parser next engine */
       UINT32 FmanBmRpso;       /* Rx parse start offset */
       UINT32 FmanBmRpp;        /* Rx policer profile */
       UINT32 FmanBmRccb;       /* Rx coarse classification base */
       UINT32 Res1[0x2];
       UINT32 FmanBmRprai[0x8]; /* Rx parse results array Initialization */
       UINT32 FmanBmRfqid;             /* Rx frame queue ID */
       UINT32 FmanBmRefqid;     /* Rx error frame queue ID */
       UINT32 FmanBmRfsdm;             /* Rx frame status discard mask */
       UINT32 FmanBmRfsem;             /* Rx frame status error mask */
       UINT32 FmanBmRfene;             /* Rx frame enqueue next engine */
       UINT32 Res2[0x23];
       UINT32 FmanBmEbmpi[0x8]; /* buffer manager pool information */
       UINT32 FmanBmAcnt[0x8];  /* allocate counter */
       UINT32 Res3[0x8];
       UINT32 FmanBmCgm[0x8];   /* congestion group map */
       UINT32 FmanBmMpd;        /* BMan pool depletion */
       UINT32 Res4[0x1F];
       UINT32 FmanBmRstc;              /* Rx statistics counters */
       UINT32 FmanBmRfrc;              /* Rx frame counters */
       UINT32 FmanBmRfbc;              /* Rx bad frames counter */
       UINT32 FmanBmRlfc;              /* Rx large frames counter */
       UINT32 FmanBmRffc;              /* Rx filter frames counter */
       UINT32 FmanBmRfdc;              /* Rx frame discard counter */
       UINT32 FmanBmRfldec;     /* Rx frames list DMA error counter */
       UINT32 FmanBmRodc;              /* Rx out of buffers discard counter */
       UINT32 FmanBmRbdc;              /* Rx buffers deallocate counter */
       UINT32 Res5[0x17];
       UINT32 FmanBmRpc;        /* Rx performance counters */
       UINT32 FmanBmRpcp;              /* Rx performance count parameters */
       UINT32 FmanBmRccn;              /* Rx cycle counter */
       UINT32 FmanBmRtuc;              /* Rx tasks utilization counter */
       UINT32 FmanBmRrquc;             /* Rx receive queue utilization counter */
       UINT32 FmanBmRduc;              /* Rx DMA utilization counter */
       UINT32 FmanBmRfuc;              /* Rx FIFO utilization counter */
       UINT32 FmanBmRpac;              /* Rx pause activation counter */
       UINT32 Res6[0x18];
       UINT32 FmanBmRdbg;              /* Rx debug configuration */
} BMI_RX_PORT;

/* Rx/Tx queue descriptor */
typedef struct _FMAN_QUEUE_DESC {
       UINT16 Gen;
       UINT16 BdRingBaseHi;
       UINT32 BdRingBaseLo;
       UINT16 BdRingSize;
       UINT16 OffsetIn;
       UINT16 OffsetOut;
       UINT16 Res0;
       UINT32 Res1[0x4];
} FMAN_QUEUE_DESC;

/* IM global parameter RAM */
typedef struct _FMAN_GLOBAL_PARAM {
       UINT32 Mode;     /* independent mode register */
       UINT32 RxqdPtr; /* Rx queue descriptor pointer */
       UINT32 TxqdPtr; /* Tx queue descriptor pointer */
       UINT16 Mrblr;    /* max Rx buffer length */
       UINT16 RxqdBsyCnt;    /* RxQD busy counter, should be cleared */
       UINT32 Res0[0x4];
       FMAN_QUEUE_DESC Rxqd;     /* Rx queue descriptor */
       FMAN_QUEUE_DESC Txqd;     /* Tx queue descriptor */
       UINT32 Res1[0x28];
} FMAN_GLOBAL_PARAM;

/* Fman ethernet private struct */
typedef struct _ETH_DEVICE {
       INT32 FmIndex;               /* Fman index */
       UINT32 Num;                    /* 0..n-1 for give type */
       BMI_TX_PORT *TxPort;
       BMI_RX_PORT *RxPort;
       FMAN_ETH_TYPE Type;             /* 1G or 10G ethernet */
       ENET_MAC *Mac;   /* MAC controller */
       FMAN_GLOBAL_PARAM *RxPram; /* Rx parameter table */
       FMAN_GLOBAL_PARAM *TxPram; /* Tx parameter table */
       VOID *RxBdRing;           /* Rx BD ring base */
       VOID *CurRxbd;                    /* current Rx BD */
       VOID *RxBuf;               /* Rx buffer base */
       VOID *TxBdRing;           /* Tx BD ring base */
       INT8 CurPendingTxbdId;    /*latest Transmission Pending BD ID : Rear of Circular TxBD Queue*/
       INT8 CurUsedTxbdId;       /*oldest used(transmitted) BD Id : Front of Circular TxBD Queue*/
       UINT8 TotalPendingTxbd;   /*Total Buffers pending to be transmitted : Size of Circular TxBD Queue*/
       EFI_LOCK TxSyncLock;		/* TPL-based lock to serialize access to Transmit BD Ring */
       EFI_LOCK RxSyncLock;		/* TPL-based lock to serialize access to Receive BD Ring */
} ETH_DEVICE;

/**
 * State information kept for a FMAN MEMAC
 */
typedef struct _FMAN_MEMAC {
  /**
   * Signature for run-time type checking
   */
  UINT32 Signature;
# define FMAN_MEMAC_SIGNATURE SIGNATURE_32('M', 'E', 'M', 'A')

  /**
   * Memac ID
   */
  FMAN_MEMAC_ID Id;

  /**
   * Flag indicating if this Memac is enabled (corresponding bit in the
   * CCSR DEVDISR2 register is off)
   */
  BOOLEAN Enabled;

  /**
   * Node in a linked list of enabled MEMACs
   */
  LIST_ENTRY ListNode;

  /**
   * Ethernet PHY associated with this MEMAC
   */
  DPAA1_PHY Phy;
} FMAN_MEMAC;

#define IMASK_MASK_ALL		0x00000000
#define IEVENT_CLEAR_ALL		0xffffffff

/* MAXFRM - maximum frame length */
#define MAXFRM_MASK			0x0000ffff
#define MEMAC_MAXFRM			1518

typedef struct _Memac {
       /* memac general control and status registers */
       UINT32    Res_0[2];
       UINT32    CommandConfig;      	/* Control and configuration register */
       UINT32    MacAddr_0;   		/* Lower 32 bits of 48-bit MAC address */
       UINT32    MacAddr_1;   		/* Upper 16 bits of 48-bit MAC address */
       UINT32    Maxfrm;              	/* Maximum frame length register */
       UINT32    Res_18;
       UINT32    RxFifo;              	/* RX FIFO register */
       UINT32    TxFifo;              	/* TX FIFI register */
       UINT32    Res_24[2];
       UINT32    HashtableCtrl;      	/* Hash table control register */
       UINT32    Res_30[4];
       UINT32    Ievent;              	/* Interrupt event register */
       UINT32    TxIpgLength;       	/* Transmitter inter-packet-gap register */
       UINT32    Res_48;
       UINT32    Imask;        		/* interrupt mask register */
       UINT32    Res_50;
       UINT32    ClPauseQuanta[4]; 	/* CL01-CL67 pause quanta register */
       UINT32    ClPauseThresh[4]; 	/* CL01-CL67 pause thresh register */
       UINT32    RxPauseStatus;     	/* Receive pause status register */
       UINT32    Res_78[2];
       UINT32    MacAddr[14]; 		/* MAC address */
       UINT32    LpwakeTimer; 		/* EEE low power wakeup timer register */
       UINT32    SleepTimer;  		/* Transmit EEE Low Power Timer register */
       UINT32    ResC0[8];
       UINT32    StatnConfig; 		/* Statistics configuration register */
       UINT32    ResE4[7];

       /* memac statistics counter registers */
       UINT32    RxEoctL;    /* Rx ethernet octests lower */
       UINT32    RxEoctU;    /* Rx ethernet octests upper */
       UINT32    RxOctL;     /* Rx octests lower */
       UINT32    RxOctU;     /* Rx octests upper */
       UINT32    RxAlignErrL;      	/* Rx alignment error lower */
       UINT32    RxAlignErrU;      	/* Rx alignment error upper */
       UINT32    RxPauseFrameL; 	/* Rx valid pause frame upper */
       UINT32    RxPauseFrameU; 	/* Rx valid pause frame upper */
       UINT32    RxFrameL;   		/* Rx frame counter lower */
       UINT32    RxFrameU;   		/* Rx frame counter upper */
       UINT32    RxFrameCrcErrL; 	/* Rx frame check sequence error lower */
       UINT32    RxFrameCrcErrU; 	/* Rx frame check sequence error upper */
       UINT32    RxVlanL;    /* Rx VLAN frame lower */
       UINT32    RxVlanU;    /* Rx VLAN frame upper */
       UINT32    RxErrL;     /* Rx frame error lower */
       UINT32    RxErrU;     /* Rx frame error upper */
       UINT32    RxUniL;     /* Rx unicast frame lower */
       UINT32    RxUniU;     /* Rx unicast frame upper */
       UINT32    RxMultiL;   /* Rx multicast frame lower */
       UINT32    RxMultiU;   /* Rx multicast frame upper */
       UINT32    RxBrdL;     /* Rx broadcast frame lower */
       UINT32    RxBrdU;     /* Rx broadcast frame upper */
       UINT32    RxDropL;    /* Rx dropped packets lower */
       UINT32    RxDropU;    /* Rx dropped packets upper */
       UINT32    RxPktL;     /* Rx packets lower */
       UINT32    RxPktU;     /* Rx packets upper */
       UINT32    RxUndszL;   /* Rx undersized packet lower */
       UINT32    RxUndszU;   /* Rx undersized packet upper */
       UINT32    Rx_64L;      /* Rx 64 oct packet lower */
       UINT32    Rx_64U;      /* Rx 64 oct packet upper */
       UINT32    Rx_127L;     /* Rx 65 to 127 oct packet lower */
       UINT32    Rx_127U;     /* Rx 65 to 127 oct packet upper */
       UINT32    Rx_255L;     /* Rx 128 to 255 oct packet lower */
       UINT32    Rx_255U;     /* Rx 128 to 255 oct packet upper */
       UINT32    Rx_511L;     /* Rx 256 to 511 oct packet lower */
       UINT32    Rx_511U;     /* Rx 256 to 511 oct packet upper */
       UINT32    Rx_1023L;    /* Rx 512 to 1023 oct packet lower */
       UINT32    Rx_1023U;    /* Rx 512 to 1023 oct packet upper */
       UINT32    Rx_1518L;    /* Rx 1024 to 1518 oct packet lower */
       UINT32    Rx_1518U;    /* Rx 1024 to 1518 oct packet upper */
       UINT32    Rx_1519L;    /* Rx 1519 to max oct packet lower */
       UINT32    Rx_1519U;    /* Rx 1519 to max oct packet upper */
       UINT32    RxOverszL;  /* Rx oversized packet lower */
       UINT32    RxOverszU;  /* Rx oversized packet upper */
       UINT32    RxJabberL;  /* Rx Jabber packet lower */
       UINT32    RxJabberU;  /* Rx Jabber packet upper */
       UINT32    RxFragL;    /* Rx Fragment packet lower */
       UINT32    RxFragU;    /* Rx Fragment packet upper */
       UINT32    RxCnpL;     /* Rx control packet lower */
       UINT32    RxCnpU;     /* Rx control packet upper */
       UINT32    RxDrntpL;   /* Rx dripped not truncated packet lower */
       UINT32    RxDrntpU;   /* Rx dripped not truncated packet upper */
       UINT32    Res_1d0[0xc];

       UINT32    TxEoctL;    /* Tx ethernet octests lower */
       UINT32    TxEoctU;    /* Tx ethernet octests upper */
       UINT32    TxOctL;     /* Tx octests lower */
       UINT32    TxOctU;     /* Tx octests upper */
       UINT32    Res_210[0x2];
       UINT32    TxPauseFrameL; /* Tx valid pause frame lower */
       UINT32    TxPauseFrameU; /* Tx valid pause frame upper */
       UINT32    TxFrameL;   /* Tx frame counter lower */
       UINT32    TxFrameU;   /* Tx frame counter upper */
       UINT32    TxFrameCrcErrL; /* Tx frame check sequence error lower */
       UINT32    TxFrameCrcErrU; /* Tx frame check sequence error upper */
       UINT32    TxVlanL;    /* Tx VLAN frame lower */
       UINT32    TxVlanU;    /* Tx VLAN frame upper */
       UINT32    TxFrameErrL;      /* Tx frame error lower */
       UINT32    TxFrameErrU;      /* Tx frame error upper */
       UINT32    TxUniL;     /* Tx unicast frame lower */
       UINT32    TxUniU;     /* Tx unicast frame upper */
       UINT32    TxMultiL;   /* Tx multicast frame lower */
       UINT32    TxMultiU;   /* Tx multicast frame upper */
       UINT32    TxBrdL;     /* Tx broadcast frame lower */
       UINT32    TxBrdU;     /* Tx broadcast frame upper */
       UINT32    Res_258[0x2];
       UINT32    TxPktL;     /* Tx packets lower */
       UINT32    TxPktU;     /* Tx packets upper */
       UINT32    TxUndszL;   /* Tx undersized packet lower */
       UINT32    TxUndszU;   /* Tx undersized packet upper */
       UINT32    Tx_64L;      /* Tx 64 oct packet lower */
       UINT32    Tx_64U;      /* Tx 64 oct packet upper */
       UINT32    Tx_127L;     /* Tx 65 to 127 oct packet lower */
       UINT32    Tx_127U;     /* Tx 65 to 127 oct packet upper */
       UINT32    Tx_255L;     /* Tx 128 to 255 oct packet lower */
       UINT32    Tx_255U;     /* Tx 128 to 255 oct packet upper */
       UINT32    Tx_511L;     /* Tx 256 to 511 oct packet lower */
       UINT32    Tx_511U;     /* Tx 256 to 511 oct packet upper */
       UINT32    Tx_1023L;    /* Tx 512 to 1023 oct packet lower */
       UINT32    Tx_1023U;    /* Tx 512 to 1023 oct packet upper */
       UINT32    Tx_1518L;    /* Tx 1024 to 1518 oct packet lower */
       UINT32    Tx_1518U;    /* Tx 1024 to 1518 oct packet upper */
       UINT32    Tx_1519L;    /* Tx 1519 to max oct packet lower */
       UINT32    Tx_1519U;    /* Tx 1519 to max oct packet upper */
       UINT32    Res_2a8[0x6];
       UINT32    TxCnpL;     /* Tx control packet lower */
       UINT32    TxCnpU;     /* Tx control packet upper */
       UINT32    Res_2c8[0xe];

       /* Line interface control register */
       UINT32 IfMode;         /* interface mode control */
       UINT32 IfStatus;              /* interface status */
       UINT32 Res_308[0xe];

       /* HiGig/2 Register */
       UINT32 HgConfig;       /* HiGig2 control and configuration */
       UINT32 Res_344[0x3];
       UINT32 HgPauseQuanta; /* HiGig2 pause quanta */
       UINT32 Res_354[0x3];
       UINT32 HgPauseThresh; /* HiGig2 pause quanta threshold */
       UINT32 Res_364[0x3];
       UINT32 HgrxPauseStatus;      /* HiGig2 rx pause quanta status */
       UINT32 HgFifosStatus; /* HiGig2 fifos status */
       UINT32 Rhm;      /* Rx HiGig2 message counter register */
       UINT32 Thm;/* Tx HiGig2 message counter register */
       UINT32 Res_380[0x320];
} Memac;

VOID
FmanMemacInit (FMAN_MEMAC_ID MemacId,
              PHY_INTERFACE_TYPE PhyInterfaceType,
              DPAA1_PHY_MDIO_BUS *MdioBus,
              UINT8 PhyAddress,
              LIST_ENTRY *EnabledDmpacsList);

CONST CHAR8 *
PhyInterfaceTypeToString(PHY_INTERFACE_TYPE PhyInterfaceType);

extern CONST CHAR8 *CONST gFmanMemacStrings[];

VOID InitializeMac (
  IN  ENET_MAC *Mac
  );

VOID SetInterface (
  IN  ENET_MAC *Mac,
  IN  PHY_INTERFACE_TYPE Type,
  IN  INT32 Speed
  );

VOID
SetMacAddr (
  IN  ENET_MAC *Mac,
  IN  EFI_MAC_ADDRESS MAddr
  );

VOID
DisablePorts (
  IN  ETH_DEVICE *FmanEthDevice
  );

VOID
EnablePorts (
  IN  ETH_DEVICE *FmanEthDevice
  );

VOID
TxGracefulStopDisable (
  IN ETH_DEVICE *FmanEthDevice
  );

VOID
TxGracefulStopEnable (
  IN ETH_DEVICE *FmanEthDevice
  );

VOID
DumpMacStats (
  ENET_MAC *Mac
  );

VOID
DumpMac (
  Memac *Reg
  );

BOOLEAN
IsMemacEnabled (
  IN  FMAN_MEMAC_ID MemacId
  );

/**
  Fix the Memac state in Fdt (Flattened Device tree)
  Update the status field to "okay" or "disabled"
  Update the phy-interface-type field.

  @param[in]  Fdt   Fdt blob to fixup

  @retval EFI_SUCCESS       Successfully fixed the device tree
  @retval EFI_DEVICE_ERROR  Failed to fix the device tree
**/
EFI_STATUS
FdtMemacFixup (
  IN  VOID  *Fdt
);
#endif /* __DPAA1_ETHERTNET_MAC_LIB_H__ */
