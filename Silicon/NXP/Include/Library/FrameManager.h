/** @file
  DPAA Frame manager declarations

  Copyright (c) 2016, Freescale Semiconductor, Inc. All rights reserved.

  This Program And The Accompanying Materials
  Are Licensed And Made Available Under The Terms And Conditions Of The BSD
  License Which Accompanies This Distribution. The Full Text Of The License
  May Be Found At
  http://Opensource.Org/Licenses/Bsd-License.Php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __FRAME_MANAGER_H__
#define __FRAME_MANAGER_H__

#include <Base.h>
#include <Library/Dpaa1EthernetMacLib.h>
#include <Library/Dpaa1Lib.h>
#include <Library/Utils.h>
#include <Uefi.h>

#define FMAN_MEM_RES_SIZE	    	(4 * 1024U)
#define FMAN_FREE_POOL_SIZE	    	(128 * 1024U) /* bytes */
#define FMAN_FREE_POOL_ALIGNMENT   256

#define FMAN_PRAM_SIZE        	sizeof(FMAN_GLOBAL_PARAM)
#define FMAN_PRAM_ALIGN            256

//
// FMan hardware ports
//
#define FMAN_OFFLINE_PORT_ID_BASE  	0x01
#define FMAN_MAX_NUM_OFFLINE_PORTS 	7   /* O/H ports */

#define FMAN_RX_1G_PORT_ID_BASE	    	0x08
#define FMAN_MAX_NUM_RX_1G_PORTS    	8

#define FMAN_RX_10G_PORT_ID_BASE    	0x10

#define FMAN_TX_1G_PORT_ID_BASE     	0x28
#define FMAN_MAX_NUM_TX_1G_PORTS    	8

#define FMAN_TX_10G_PORT_ID_BASE    	0x30

#define FMAN_MIIM_TIMEOUT           	0xFFFF

#define RX_RING_SIZE            	8
#define TX_RING_SIZE            	8
#define MAX_RXBUF_LOG2             	11
#define MAX_RXBUF_LEN              	BIT(MAX_RXBUF_LOG2)

/* Rx/Tx buffer descriptor */
typedef struct _BD {
       UINT16 Status;
       UINT16 Len;
       UINT32 Res0;
       UINT16 Res1;
       UINT16 BufPtrHi;
       UINT32 BufPtrLo;
} BD;

//
// FMan private memory area
//
typedef struct _FMAN_MEMORY {
        VOID *Base;
        VOID *Top;
        UINT32 Size;
	 VOID *AllocCursor;
} FMAN_MEMORY;

//
// FMan controller configuration registers
//
typedef struct _FMAN_CONTROLLER_CONFIG {
        UINT32 Address;
        UINT32 Data;
        UINT32 TimingConfig;
        UINT32 DataReady;
        UINT8  Reserved[0xff0];
} FMAN_CONTROLLER_CONFIG;

//
// FMAN Buffer manager interface (BMI) common registers
//
typedef struct _FMAN_BMI_COMMON {
  UINT32    Init;           /* BMI initialization */
# define    BMI_INIT_START  0x80000000 /* init internal buffers */

  UINT32    Cfg1;           /* BMI configuration1 */
# define    BMI_CFG1_FBPS_MASK   0x03ff0000 /* Free buffer pool size */
# define    BMI_CFG1_FBPS_SHIFT  16
# define    BMI_CFG1_FBPO_MASK   0x000003ff /* Free buffer pool offset */
# define    BMI_CFG1_FBPO_SHIFT  0

  UINT32    Cfg2;           /* BMI configuration2 */
  UINT32    Reserved[0x5];
  UINT32    Ievr;           /* interrupt event register */
# define    BMI_IEVR_PEC    0x80000000 /* pipeline table ECC err detected */
# define    BMI_IEVR_LEC    0x40000000 /* linked list RAM ECC error */
# define    BMI_IEVR_SEC    0x20000000 /* statistics count RAM ECC error */
# define    BMI_IEVR_ALL    (BMI_IEVR_PEC | BMI_IEVR_LEC | BMI_IEVR_SEC)

  UINT32    Ier;            /* interrupt enable register */
  UINT32    Ifr;            /* interrupt force register */
  UINT32    Reserved1[0x5];
  UINT32    Arb[0x8];       /* BMI arbitration */
  UINT32    Reserved2[0x28];
  UINT32    Gde;            /* global debug enable */
  UINT32    Pp[0x3f];       /* BMI port parameters */
# define    BMI_PP_MXT_MASK	0x3f000000 /* Max # tasks */
# define    BMI_PP_MXT_SHIFT    24
# define    BMI_PP_MXD_MASK	0x00000f00 /* Max DMA */
# define    BMI_PP_MXD_SHIFT    8

  UINT32    Reserved3;
  UINT32    Pfs[0x3f];      /* BMI port FIFO size */
# define    BMI_PFS_IFSZ_MASK   0x000003ff /* Internal Fifo Size */
# define    BMI_PFS_IFSZ_SHIFT  0

  UINT32    Reserved4;
  UINT32    Ppid[0x3f];     /* port partition ID */
} FMAN_BMI_COMMON;

//
// FMAN queue manager interface (BMI) common registers
//
typedef struct _FMAN_QMI_COMMON {
  UINT32        Gc;         /* general configuration register */
# define        QMI_GC_ENQ_EN   0x80000000 /* enqueue enable */
# define        QMI_GC_DEQ_EN   0x40000000 /* dequeue enable */
# define        QMI_GC_STEN	    0x10000000 /* enable global stat counters */

  UINT32        Reserved0;
  UINT32        Eie;        /* error interrupt event register */
# define        QMI_EIE_DEE	    0x80000000 /* double-bit ECC error */
# define        QMI_EIE_DFUPE   0x40000000 /* dequeue from unknown PortID */
# define        QMI_EIE_ALL     (QMI_EIE_DEE | QMI_EIE_DFUPE)

  UINT32        Eien;       /* error interrupt enable register */
# define        QMI_EIEN_DEEN	0x80000000 /* double-bit ECC error */
# define        QMI_EIEN_DFUPEN	0x40000000 /* dequeue from unknown PortID */

  UINT32        Eif;        /* error interrupt force register */
  UINT32        Ie;         /* interrupt event register */
# define        QMI_IE_SEE  0x80000000 /* single-bit ECC error detected */
# define        QMI_IE_ALL  (QMI_IE_SEE)

  UINT32        Ien;        /* interrupt enable register */
# define        QMI_IEN_SEE 0x80000000 /* single-bit ECC err IRQ enable */

  UINT32        If;         /* interrupt force register */
  UINT32        Gs;         /* global status register */
  UINT32        Ts;         /* task status register */
  UINT32        Etfc;       /* enqueue total frame counter */
  UINT32        Dtfc;       /* dequeue total frame counter */
  UINT32        Dc0;        /* dequeue counter 0 */
  UINT32        Dc1;        /* dequeue counter 1 */
  UINT32        Dc2;        /* dequeue counter 2 */
  UINT32        Dc3;        /* dequeue counter 3 */
  UINT32        Dfnoc;      /* dequeue FQID not override counter */
  UINT32        Dfcc;       /* dequeue FQID from context counter */
  UINT32        Dffc;       /* dequeue FQID from FD counter */
  UINT32        Dcc;        /* dequeue confirm counter */
  UINT32        Reserved1[0xc];
  UINT32        Dtrc;       /* debug trap configuration register */
  UINT32        Efddd;      /* enqueue frame descriptor dynamic debug */
  UINT32        Reserved3[0x2];
  UINT32        Reserved4[0xdc];        /* missing debug regs */
} FMAN_QMI_COMMON;

//
// FMan DMA registers
//
typedef struct _FMAN_DMA {
  UINT32	Sr;		/* status register */
# define        DMA_SR_CMDQNE	0x10000000 /* command queue not empty */
# define        DMA_SR_BER	    	0x08000000 /* bus err event occurred on bus */
# define        DMA_SR_RDB_ECC	0x04000000 /* read buffer ECC error */
# define        DMA_SR_WRB_SECC	0x02000000 /* write buf ECC err sys side */
# define        DMA_SR_WRB_FECC	0x01000000 /* write buf ECC err Fman side */
# define        DMA_SR_DPEXT_SECC  0x00800000 /* DP external ECC err sys side */
# define        DMA_SR_DPEXT_FECC  0x00400000 /* DP external ECC err Fman side */
# define        DMA_SR_DPDAT_SECC  0x00200000 /* DP data ECC err on sys side */
# define        DMA_SR_DPDAT_FECC  0x00100000 /* DP data ECC err on Fman side */
# define        DMA_SR_SPDAT_FECC  0x00080000 /* SP data ECC error Fman side */
#define         DMA_SR_ALL_ERRORS  (DMA_SR_BER | DMA_SR_RDB_ECC \
				     | DMA_SR_WRB_SECC | DMA_SR_WRB_FECC \
				     | DMA_SR_DPEXT_SECC | DMA_SR_DPEXT_FECC \
				     | DMA_SR_DPDAT_SECC | DMA_SR_DPDAT_FECC \
				     | DMA_SR_SPDAT_FECC)

  UINT32	Mr;		/* mode register */
# define        DMA_MR_SBER	0x10000000 /* stop the DMA if a bus error */

  UINT32	Tr;		/* bus threshold register */
  UINT32	Hy;		/* bus hysteresis register */
  UINT32	Setr;	        /* SOS emergency threshold register */
  UINT32	Tah;	        /* transfer bus address high register */
  UINT32	Tal;	        /* transfer bus address low register */
  UINT32	Tcid;	        /* transfer bus communication ID register */
  UINT32	Ra;		/* DMA bus internal ram address register */
  UINT32	Rd;		/* DMA bus internal ram data register */
  UINT32	Reserved0[0xb];
  UINT32	Dcr;	        /* debug counter */
  UINT32	Emsr;	        /* emrgency smoother register */
  UINT32	Reserved1;
  UINT32	Plr[32];	/* FM DMA PID-LIODN # register */
  UINT32	Reserved2[0x3c8];
} FMAN_DMA;

//
// FMan frame processing manager registers
//
typedef struct _FMAN_FPM {
  UINT32      Tnc;		/* TNUM control */
  UINT32	Prc;		/* Port_ID control */
# define        FPM_PRC_PORTID_MASK         0x3f000000
# define        FPM_PRC_PORTID_SHIFT	  24
# define        FPM_PRC_ORA_SHIFT	         16
# define        FPM_PRC_FMAN_CONTROLLER1    0x00000001
# define        FPM_PRC_FMAN_CONTROLLER2    0x00000002

  UINT32	Reserved0;
  UINT32	Flc;		/* flush control */
# define        FPM_FLC_DISPATCH_LIMIT_NONE	0x0

  UINT32	Dist1;	        /* dispatch thresholds1 */
  UINT32	Dist2;	        /* dispatch thresholds2 */
  UINT32	Epi;		/* error pending interrupts */
  UINT32	Rie;		/* rams interrupt enable */
  UINT32	Fcev[0x4];      /* FMan controller event 0-3 */
  UINT32	Reserved1[0x4];
  UINT32	Fcmask[0x4];	/* FMan controller mask 0-3 */
  UINT32	Reserved2[0x4];
  UINT32	Tsc1;	        /* timestamp control1 */
  UINT32	Tsc2;	        /* timestamp control2 */
  UINT32	Tsp;		/* time stamp */
  UINT32	Tsf;		/* time stamp fraction */
  UINT32	Rcr;		/* rams control and event */
# define        FPM_RCR_MDEC    0x00008000 /* FMan internal memory double ECC error */
# define        FPM_RCR_IDEC    0x00004000 /* FMan Controller config memory double ECC error */

  UINT32	Reserved3[0x3];
  UINT32	Drd[0x4];	/* data_ram data 0-3 */
  UINT32	Reserved4[0xc];
  UINT32	Dra;		/* data ram access */
  UINT32	IpRev1;	        /* IP block revision 1 */
  UINT32	IpRev2;	        /* IP block revision 2 */
  UINT32	Rstc;		/* reset command */
  UINT32	Cldc;		/* classifier debug control */
  UINT32	Npi;		/* normal pending interrupts */
  UINT32	Reserved5;
  UINT32	FpEe;		/* event and enable */
# define        FPM_FPEE_DECC       0x80000000 /* double ECC err on FPM ram */
# define        FPM_FPEE_STL        0x40000000 /* stall of task ... */
# define        FPM_FPEE_SECC       0x20000000 /* single ECC error */
# define        FPM_FPEE_RFM        0x00010000 /* release FMan */
# define        FPM_FPEE_DECC_EN    0x00008000 /* double ECC interrupt enable */
# define        FPM_FPEE_STL_EN     0x00004000 /* stall of task interrupt enable */
# define        FPM_FPEE_SECC_EN    0x00002000 /* single ECC err interrupt enable */
# define        FPM_FPEE_EHM	 0x00000008 /* external halt enable */
# define        FPM_FPEE_UEC	 0x00000004 /* FMan is not halted */
# define        FPM_FPEE_CER	 0x00000002 /* only errornous task stalled */
# define        FPM_FPEE_DER	 0x00000001 /* DMA error is just reported */
# define        FPM_FPEE_ALL_EVENTS (FPM_FPEE_DECC | FPM_FPEE_STL | \
                                     FPM_FPEE_SECC | FPM_FPEE_EHM | \
                                     FPM_FPEE_UEC | FPM_FPEE_CER | \
				     FPM_FPEE_DER | FPM_FPEE_RFM)

  UINT32	Cev[0x4];	/* CPU event 0-3 */
  UINT32	Reserved6[0x4];
  UINT32	Ps[0x40];	/* port status */
  UINT32	Reserved7[0x260];
  UINT32	Ts[0x80];	/* task status */
  UINT32	Reserved8[0xa0];
} FMAN_FPM;

//
// FMan MDIO registers
//
typedef struct _FMAN_MDIO {
  UINT8	        Reserved0[0x120];
  UINT32	Miimcfg;	/* MII management configuration reg */
  UINT32	Miimcom;	/* MII management command reg */
  UINT32	Miimadd;	/* MII management address reg */
  UINT32	Miimcon;	/* MII management control reg */
  UINT32	Miimstat;	/* MII management status reg  */
  UINT32	Miimind;	/* MII management indication reg */
  UINT8         Reserved1[0x1000 - 0x138];
} FMAN_MDIO;

//
// frame manager (FMan) CCSR space layout
//
typedef struct _FMAN_CCSR {
  UINT8             FmanMemory[0x80000]; /* 512KB : 384 (Internal Memory) + */
  FMAN_BMI_COMMON   BmiCommon;
  FMAN_QMI_COMMON   QmiCommon;
  UINT8             Reserved[2048];
  struct {
    UINT8           FmanBmi[1024];
    UINT8           FmanQmi[1024];
    UINT8           FmanParser[1024];
    UINT8           Reserved[1024];
  } port[63];

  UINT8             Policer[4*1024];
  UINT8             KeyGen[4*1024];
  FMAN_DMA          FmanDma;
  FMAN_FPM          FmanFpm;
  FMAN_CONTROLLER_CONFIG FmanControllerCfg;
  UINT8             Reserved1[8*1024];
  UINT8             FmanSoftParser[4*1024];
  UINT8             Reserved2[96*1024];
  struct {
    UINT8           FmanMemac[4*1024];
    UINT8           FmanMemacMdio[4*1024];
  } memac[10];
  UINT8             Reserved4[32*1024];
  UINT8             FmanDedicatedMdio[2][4*1024];
  UINT8             Fman1588[4*1024];
  UINT8             Reserved5[4*1024];
} FMAN_CCSR;

//
// FMan firmware image header
//
typedef struct _FMAN_FW_HEADER {
  UINT32 Length;    /* Length of firmware image in bytes */
  UINT8 Magic[3];   /* Set to { 'Q', 'E', 'F' } */
  UINT8 Version;    /* Version of this layout. */
} FMAN_FW_HEADER;


//
// FMan firmware microcode header
//
typedef struct _FMAN_FW_MICROCODE_HEADER {
  UINT8     Id[32];        /* Null-terminated identifier */
  UINT32    Traps[16];        /* Trap addresses, 0 == ignore */
  UINT32    Eccr;
  UINT32    IRamOffset; /* Offset into I-RAM for the code */
  UINT32    Count;        /* Number of 32-bit words of code */
  UINT32    CodeOffset; /* Offset of the actual microcode */
  UINT8     Major;        /* The microcode version major */
  UINT8     Minor;        /* The microcode version minor */
  UINT8     Revision;        /* The microcode version revision */
  UINT8     Padding;        /* Reserved, for alignment */
  UINT8     Reserved[4];
} FMAN_FW_MICROCODE_HEADER;

//
// Fman firmware image layout
//
typedef struct _FMAN_FIRMWARE {
  FMAN_FW_HEADER Header;
  UINT8     Id[62]; /* Null-terminated identifier string */
  UINT8     Split;  /* 0 = shared I-RAM, 1 = split I-RAM */
  UINT8     Count;  /* Number of entries of MicrocodeHeaders[] */
  /* SoC model and revision major-minor number */
  UINT16    SocModel;
  UINT8     SocMajor;
  UINT8     SocMinor;
  UINT8     Padding[4];
  UINT64    ExtendedModes;
  UINT32    Vtraps[8];  /* Virtual trap addresses */
  UINT8     reserved[4];
  FMAN_FW_MICROCODE_HEADER MicrocodeHeaders[]; /* variable-size array */
  /* CRC32 should be located after the last byte of microcode */

  //
  // NOTE Since this struct has a UINT64 field, unless we use
  // '__attribute__ ((packed))', the compiler will round up the size of
  // the struct to a multiple of 8, which adds unwanted padding
  // at the end of the struct.
  //
} __attribute__ ((packed)) FMAN_FIRMWARE;

/**
   Initialize a given frame manager (FMan) instance

   @param[in]     FManIndex FMan instance index
   @param[in]     FManCcsrBase Base address of the FMan instance's CCSR registers
                  memory

   @retval EFI_SUCCESS   on success
   @retval !EFI_SUCCESS  on failure.

 **/
EFI_STATUS
FmanInit (
  INTN  FmanIndex,
  FMAN_CCSR *FmanCcsr
  );

UINT32
ComputeCrc32(
  IN  CONST VOID    *Data,
  IN  UINTN         DataSize
  );

#endif /* __FRAME_MANAGER_H__ */
