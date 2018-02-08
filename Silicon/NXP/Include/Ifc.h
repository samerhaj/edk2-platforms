/** @Ifc.h

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __IFC_H__
#define __IFC_H__

#include <Library/BaseLib.h>
#include <Uefi.h>

#define IFC_BANK_COUNT        4

#define IFC_CSPR_REG_LEN      148
#define IFC_AMASK_REG_LEN     144
#define IFC_CSOR_REG_LEN      144
#define IFC_FTIM_REG_LEN      576

#define IFC_CSPR_USED_LEN     sizeof (IFC_CSPR) * \
                              IFC_BANK_COUNT

#define IFC_AMASK_USED_LEN    sizeof (IFC_AMASK) * \
                              IFC_BANK_COUNT

#define IFC_CSOR_USED_LEN     sizeof (IFC_CSOR) * \
                              IFC_BANK_COUNT

#define IFC_FTIM_USED_LEN     sizeof (IFC_FTIM) * \
                              IFC_BANK_COUNT

/* List of commands */
#define IFC_NAND_CMD_RESET        0xFF
#define IFC_NAND_CMD_READID       0x90
#define IFC_NAND_CMD_STATUS       0x70
#define IFC_NAND_CMD_READ0        0x00
#define IFC_NAND_CMD_READSTART    0x30
#define IFC_NAND_CMD_ERASE1       0x60
#define IFC_NAND_CMD_ERASE2       0xD0
#define IFC_NAND_CMD_SEQIN        0x80
#define IFC_NAND_CMD_PAGEPROG     0x10
#define MAX_RETRY_COUNT           150000


#define IFC_NAND_SEQ_STRT_FIR_STRT  0x80000000

/*
 * NAND Event and Error Status Register (NAND_EVTER_STAT)
 */

/* Operation Complete */
#define IFC_NAND_EVTER_STAT_OPC     0x80000000

/* Flash Timeout Error */
#define IFC_NAND_EVTER_STAT_FTOER   0x08000000

/* Write Protect Error */
#define IFC_NAND_EVTER_STAT_WPER    0x04000000

/* ECC Error */
#define IFC_NAND_EVTER_STAT_ECCER   0x02000000

/*
 * NAND Flash Instruction Registers (NAND_FIR0/NAND_FIR1/NAND_FIR2)
 */

/* NAND Machine specific opcodes OP0-OP14*/
#define IFC_NAND_FIR0_OP0           0xFC000000
#define IFC_NAND_FIR0_OP0_SHIFT     26
#define IFC_NAND_FIR0_OP1           0x03F00000
#define IFC_NAND_FIR0_OP1_SHIFT     20
#define IFC_NAND_FIR0_OP2           0x000FC000
#define IFC_NAND_FIR0_OP2_SHIFT     14
#define IFC_NAND_FIR0_OP3           0x00003F00
#define IFC_NAND_FIR0_OP3_SHIFT     8
#define IFC_NAND_FIR0_OP4           0x000000FC
#define IFC_NAND_FIR0_OP4_SHIFT     2
#define IFC_NAND_FIR1_OP5           0xFC000000
#define IFC_NAND_FIR1_OP5_SHIFT     26
#define IFC_NAND_FIR1_OP6           0x03F00000
#define IFC_NAND_FIR1_OP6_SHIFT     20
#define IFC_NAND_FIR1_OP7           0x000FC000
#define IFC_NAND_FIR1_OP7_SHIFT     14
#define IFC_NAND_FIR1_OP8           0x00003F00
#define IFC_NAND_FIR1_OP8_SHIFT     8
#define IFC_NAND_FIR1_OP9           0x000000FC
#define IFC_NAND_FIR1_OP9_SHIFT     2
#define IFC_NAND_FIR2_OP10          0xFC000000
#define IFC_NAND_FIR2_OP10_SHIFT    26
#define IFC_NAND_FIR2_OP11          0x03F00000
#define IFC_NAND_FIR2_OP11_SHIFT    20
#define IFC_NAND_FIR2_OP12          0x000FC000
#define IFC_NAND_FIR2_OP12_SHIFT    14
#define IFC_NAND_FIR2_OP13          0x00003F00
#define IFC_NAND_FIR2_OP13_SHIFT    8
#define IFC_NAND_FIR2_OP14          0x000000FC
#define IFC_NAND_FIR2_OP14_SHIFT    2

/*
 * NAND Flash Command Registers (NAND_FCR0/NAND_FCR1)
 */

/* General purpose FCM flash command bytes CMD0-CMD7 */
#define IFC_NAND_FCR0_CMD0          0xFF000000
#define IFC_NAND_FCR0_CMD0_SHIFT    24
#define IFC_NAND_FCR0_CMD1          0x00FF0000
#define IFC_NAND_FCR0_CMD1_SHIFT    16
#define IFC_NAND_FCR0_CMD2          0x0000FF00
#define IFC_NAND_FCR0_CMD2_SHIFT    8
#define IFC_NAND_FCR0_CMD3          0x000000FF
#define IFC_NAND_FCR0_CMD3_SHIFT    0
#define IFC_NAND_FCR1_CMD4          0xFF000000
#define IFC_NAND_FCR1_CMD4_SHIFT    24
#define IFC_NAND_FCR1_CMD5          0x00FF0000
#define IFC_NAND_FCR1_CMD5_SHIFT    16
#define IFC_NAND_FCR1_CMD6          0x0000FF00
#define IFC_NAND_FCR1_CMD6_SHIFT    8
#define IFC_NAND_FCR1_CMD7          0x000000FF
#define IFC_NAND_FCR1_CMD7_SHIFT    0

/* Timing registers for NAND Flash */

#define IFC_FTIM0_NAND_TCCST_SHIFT  25
#define IFC_FTIM0_NAND_TCCST(n)     ((n) << IFC_FTIM0_NAND_TCCST_SHIFT)
#define IFC_FTIM0_NAND_TWP_SHIFT    16
#define IFC_FTIM0_NAND_TWP(n)       ((n) << IFC_FTIM0_NAND_TWP_SHIFT)
#define IFC_FTIM0_NAND_TWCHT_SHIFT  8
#define IFC_FTIM0_NAND_TWCHT(n)     ((n) << IFC_FTIM0_NAND_TWCHT_SHIFT)
#define IFC_FTIM0_NAND_TWH_SHIFT    0
#define IFC_FTIM0_NAND_TWH(n)       ((n) << IFC_FTIM0_NAND_TWH_SHIFT)
#define IFC_FTIM1_NAND_TADLE_SHIFT  24
#define IFC_FTIM1_NAND_TADLE(n)     ((n) << IFC_FTIM1_NAND_TADLE_SHIFT)
#define IFC_FTIM1_NAND_TWBE_SHIFT   16
#define IFC_FTIM1_NAND_TWBE(n)      ((n) << IFC_FTIM1_NAND_TWBE_SHIFT)
#define IFC_FTIM1_NAND_TRR_SHIFT    8
#define IFC_FTIM1_NAND_TRR(n)       ((n) << IFC_FTIM1_NAND_TRR_SHIFT)
#define IFC_FTIM1_NAND_TRP_SHIFT    0
#define IFC_FTIM1_NAND_TRP(n)       ((n) << IFC_FTIM1_NAND_TRP_SHIFT)
#define IFC_FTIM2_NAND_TRAD_SHIFT   21
#define IFC_FTIM2_NAND_TRAD(n)      ((n) << IFC_FTIM2_NAND_TRAD_SHIFT)
#define IFC_FTIM2_NAND_TREH_SHIFT   11
#define IFC_FTIM2_NAND_TREH(n)      ((n) << IFC_FTIM2_NAND_TREH_SHIFT)
#define IFC_FTIM2_NAND_TWHRE_SHIFT  0
#define IFC_FTIM2_NAND_TWHRE(n)     ((n) << IFC_FTIM2_NAND_TWHRE_SHIFT)
#define IFC_FTIM3_NAND_TWW_SHIFT    24
#define IFC_FTIM3_NAND_TWW(n)       ((n) << IFC_FTIM3_NAND_TWW_SHIFT)

/*
 * Flash ROW and COL Address Register (ROWn, COLn)
 */

/* Main/spare region locator */
#define IFC_NAND_COL_MS         0x80000000

/* Column Address */
#define IFC_NAND_COL_CA_MASK    0x00000FFF

#define NAND_STATUS_WP          0x80

/*
 * NAND Event and Error Enable Register (NAND_EVTER_EN)
 */

/* Operation complete event enable */
#define IFC_NAND_EVTER_EN_OPC_EN      0x80000000

/* Page read complete event enable */
#define IFC_NAND_EVTER_EN_PGRDCMPL_EN 0x20000000

/* Flash Timeout error enable */
#define IFC_NAND_EVTER_EN_FTOER_EN    0x08000000

/* Write Protect error enable */
#define IFC_NAND_EVTER_EN_WPER_EN     0x04000000

/* ECC error logging enable */
#define IFC_NAND_EVTER_EN_ECCER_EN    0x02000000

/*
 * CSPR - Chip Select Property Register
 */

#define IFC_CSPR_BA               0xFFFF0000
#define IFC_CSPR_BA_SHIFT         16
#define IFC_CSPR_PORT_SIZE        0x00000180
#define IFC_CSPR_PORT_SIZE_SHIFT  7

// Port Size 8 bit
#define IFC_CSPR_PORT_SIZE_8      0x00000080

// Port Size 16 bit
#define IFC_CSPR_PORT_SIZE_16     0x00000100

// Port Size 32 bit
#define IFC_CSPR_PORT_SIZE_32     0x00000180

// Write Protect
#define IFC_CSPR_WP           0x00000040
#define IFC_CSPR_WP_SHIFT     6

// Machine Select
#define IFC_CSPR_MSEL         0x00000006
#define IFC_CSPR_MSEL_SHIFT   1

// NOR
#define IFC_CSPR_MSEL_NOR     0x00000000

/* NAND */
#define IFC_CSPR_MSEL_NAND    0x00000002

/* GPCM */
#define IFC_CSPR_MSEL_GPCM    0x00000004

// Bank Valid
#define IFC_CSPR_V            0x00000001
#define IFC_CSPR_V_SHIFT      0

/*
 * Chip Select Option Register - NOR Flash Mode
 */

// Enable Address shift Mode
#define IFC_CSOR_NOR_ADM_SHFT_MODE_EN 0x80000000

// Page Read Enable from NOR device
#define IFC_CSOR_NOR_PGRD_EN          0x10000000

// AVD Toggle Enable during Burst Program
#define IFC_CSOR_NOR_AVD_TGL_PGM_EN   0x01000000

// Address Data Multiplexing Shift
#define IFC_CSOR_NOR_ADM_MASK         0x0003E000
#define IFC_CSOR_NOR_ADM_SHIFT_SHIFT  13
#define IFC_CSOR_NOR_ADM_SHIFT(n)     ((n) << IFC_CSOR_NOR_ADM_SHIFT_SHIFT)

// Type of the NOR device hooked
#define IFC_CSOR_NOR_NOR_MODE_AYSNC_NOR 0x00000000
#define IFC_CSOR_NOR_NOR_MODE_AVD_NOR   0x00000020

// Time for Read Enable High to Output High Impedance
#define IFC_CSOR_NOR_TRHZ_MASK    0x0000001C
#define IFC_CSOR_NOR_TRHZ_SHIFT   2
#define IFC_CSOR_NOR_TRHZ_20      0x00000000
#define IFC_CSOR_NOR_TRHZ_40      0x00000004
#define IFC_CSOR_NOR_TRHZ_60      0x00000008
#define IFC_CSOR_NOR_TRHZ_80      0x0000000C
#define IFC_CSOR_NOR_TRHZ_100     0x00000010

// Buffer control disable
#define IFC_CSOR_NOR_BCTLD        0x00000001

/*
 * Chip Select Option Register IFC_NAND Machine
 */

/* Enable ECC Encoder */
#define IFC_CSOR_NAND_ECC_ENC_EN    0x80000000
#define IFC_CSOR_NAND_ECC_MODE_MASK 0x30000000

/* 4 bit correction per 520 Byte sector */
#define IFC_CSOR_NAND_ECC_MODE_4  0x00000000

/* 8 bit correction per 528 Byte sector */
#define IFC_CSOR_NAND_ECC_MODE_8  0x10000000

/* Enable ECC Decoder */
#define IFC_CSOR_NAND_ECC_DEC_EN  0x04000000

/* Row Address Length */
#define IFC_CSOR_NAND_RAL_MASK  0x01800000
#define IFC_CSOR_NAND_RAL_SHIFT 20
#define IFC_CSOR_NAND_RAL_1     0x00000000
#define IFC_CSOR_NAND_RAL_2     0x00800000
#define IFC_CSOR_NAND_RAL_3     0x01000000
#define IFC_CSOR_NAND_RAL_4     0x01800000

/* Page Size 512b, 2k, 4k */
#define IFC_CSOR_NAND_PGS_MASK  0x00180000
#define IFC_CSOR_NAND_PGS_SHIFT 16
#define IFC_CSOR_NAND_PGS_512   0x00000000
#define IFC_CSOR_NAND_PGS_2K    0x00080000
#define IFC_CSOR_NAND_PGS_4K    0x00100000
#define IFC_CSOR_NAND_PGS_8K    0x00180000

/* Spare region Size */
#define IFC_CSOR_NAND_SPRZ_MASK     0x0000E000
#define IFC_CSOR_NAND_SPRZ_SHIFT    13
#define IFC_CSOR_NAND_SPRZ_16       0x00000000
#define IFC_CSOR_NAND_SPRZ_64       0x00002000
#define IFC_CSOR_NAND_SPRZ_128      0x00004000
#define IFC_CSOR_NAND_SPRZ_210      0x00006000
#define IFC_CSOR_NAND_SPRZ_218      0x00008000
#define IFC_CSOR_NAND_SPRZ_224      0x0000A000
#define IFC_CSOR_NAND_SPRZ_CSOR_EXT 0x0000C000

/* Pages Per Block */
#define IFC_CSOR_NAND_PB_MASK     0x00000700
#define IFC_CSOR_NAND_PB_SHIFT    8
#define IFC_CSOR_NAND_PB(n)       (n-5) << IFC_CSOR_NAND_PB_SHIFT

/* Time for Read Enable High to Output High Impedance */
#define IFC_CSOR_NAND_TRHZ_MASK   0x0000001C
#define IFC_CSOR_NAND_TRHZ_SHIFT  2
#define IFC_CSOR_NAND_TRHZ_20     0x00000000
#define IFC_CSOR_NAND_TRHZ_40     0x00000004
#define IFC_CSOR_NAND_TRHZ_60     0x00000008
#define IFC_CSOR_NAND_TRHZ_80     0x0000000C
#define IFC_CSOR_NAND_TRHZ_100    0x00000010

/*
 * FTIM0 - NOR Flash Mode
 */
#define IFC_FTIM0_NOR               0xF03F3F3F
#define IFC_FTIM0_NOR_TACSE_SHIFT   28
#define IFC_FTIM0_NOR_TACSE(n)      ((n) << IFC_FTIM0_NOR_TACSE_SHIFT)
#define IFC_FTIM0_NOR_TEADC_SHIFT   16
#define IFC_FTIM0_NOR_TEADC(n)      ((n) << IFC_FTIM0_NOR_TEADC_SHIFT)
#define IFC_FTIM0_NOR_TAVDS_SHIFT   8
#define IFC_FTIM0_NOR_TAVDS(n)      ((n) << IFC_FTIM0_NOR_TAVDS_SHIFT)
#define IFC_FTIM0_NOR_TEAHC_SHIFT   0
#define IFC_FTIM0_NOR_TEAHC(n)      ((n) << IFC_FTIM0_NOR_TEAHC_SHIFT)

/*
 * FTIM1 - NOR Flash Mode
 */
#define IFC_FTIM1_NOR                   0xFF003F3F
#define IFC_FTIM1_NOR_TACO_SHIFT        24
#define IFC_FTIM1_NOR_TACO(n)           ((n) << IFC_FTIM1_NOR_TACO_SHIFT)
#define IFC_FTIM1_NOR_TRAD_NOR_SHIFT    8
#define IFC_FTIM1_NOR_TRAD_NOR(n)       ((n) << IFC_FTIM1_NOR_TRAD_NOR_SHIFT)
#define IFC_FTIM1_NOR_TSEQRAD_NOR_SHIFT 0
#define IFC_FTIM1_NOR_TSEQRAD_NOR(n)    ((n) << IFC_FTIM1_NOR_TSEQRAD_NOR_SHIFT)

/*
 * FTIM2 - NOR Flash Mode
 */
#define IFC_FTIM2_NOR                   0x0F3CFCFF
#define IFC_FTIM2_NOR_TCS_SHIFT         24
#define IFC_FTIM2_NOR_TCS(n)            ((n) << IFC_FTIM2_NOR_TCS_SHIFT)
#define IFC_FTIM2_NOR_TCH_SHIFT         18
#define IFC_FTIM2_NOR_TCH(n)            ((n) << IFC_FTIM2_NOR_TCH_SHIFT)
#define IFC_FTIM2_NOR_TWPH_SHIFT        10
#define IFC_FTIM2_NOR_TWPH(n)           ((n) << IFC_FTIM2_NOR_TWPH_SHIFT)
#define IFC_FTIM2_NOR_TWP_SHIFT         0
#define IFC_FTIM2_NOR_TWP(n)            ((n) << IFC_FTIM2_NOR_TWP_SHIFT)

/*
 * FTIM0 - Normal GPCM Mode
 */
#define IFC_FTIM0_GPCM                  0xF03F3F3F
#define IFC_FTIM0_GPCM_TACSE_SHIFT      28
#define IFC_FTIM0_GPCM_TACSE(n)         ((n) << IFC_FTIM0_GPCM_TACSE_SHIFT)
#define IFC_FTIM0_GPCM_TEADC_SHIFT      16
#define IFC_FTIM0_GPCM_TEADC(n)         ((n) << IFC_FTIM0_GPCM_TEADC_SHIFT)
#define IFC_FTIM0_GPCM_TAVDS_SHIFT      8
#define IFC_FTIM0_GPCM_TAVDS(n)         ((n) << IFC_FTIM0_GPCM_TAVDS_SHIFT)
#define IFC_FTIM0_GPCM_TEAHC_SHIFT      0
#define IFC_FTIM0_GPCM_TEAHC(n)         ((n) << IFC_FTIM0_GPCM_TEAHC_SHIFT)

/*
 * FTIM1 - Normal GPCM Mode
 */
#define IFC_FTIM1_GPCM                  0xFF003F00
#define IFC_FTIM1_GPCM_TACO_SHIFT       24
#define IFC_FTIM1_GPCM_TACO(n)          ((n) << IFC_FTIM1_GPCM_TACO_SHIFT)
#define IFC_FTIM1_GPCM_TRAD_SHIFT       8
#define IFC_FTIM1_GPCM_TRAD(n)          ((n) << IFC_FTIM1_GPCM_TRAD_SHIFT)

/*
 * FTIM2 - Normal GPCM Mode
 */
#define IFC_FTIM2_GPCM                  0x0F3C00FF
#define IFC_FTIM2_GPCM_TCS_SHIFT        24
#define IFC_FTIM2_GPCM_TCS(n)           ((n) << IFC_FTIM2_GPCM_TCS_SHIFT)
#define IFC_FTIM2_GPCM_TCH_SHIFT        18
#define IFC_FTIM2_GPCM_TCH(n)           ((n) << IFC_FTIM2_GPCM_TCH_SHIFT)
#define IFC_FTIM2_GPCM_TWP_SHIFT        0
#define IFC_FTIM2_GPCM_TWP(n)           ((n) << IFC_FTIM2_GPCM_TWP_SHIFT)

/* Convert an address into the right format for the CSPR Registers */
#define IFC_CSPR_PHYS_ADDR(x)   (((UINTN)x) & 0xffff0000)

/*
 * Address Mask Register
 */
#define IFC_AMASK_MASK      0xFFFF0000
#define IFC_AMASK_SHIFT     16
#define IFC_AMASK(n)        (IFC_AMASK_MASK << \
                            (HighBitSet32(n) - IFC_AMASK_SHIFT))

typedef enum {
  IFC_CS0 = 0,
  IFC_CS1,
  IFC_CS2,
  IFC_CS3,
  IFC_CS4,
  IFC_CS5,
  IFC_CS6,
  IFC_CS7,
  IFC_CS_MAX,
} IFC_CHIP_SEL;

typedef struct {
  UINT32 Ftim[IFC_BANK_COUNT];
  UINT32 CsprExt;
  UINT32 Cspr;
  UINT32 Csor;
  UINT32 Amask;
  UINT8 CS;
} IFC_TIMINGS;

#endif //__IFC_H__
