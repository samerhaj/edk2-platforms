/** IfcBoardSpecificLib.h

  IFC Flash Board Specific Macros and structure

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/
#ifndef __IFC__BOARD_SPECIFIC_H__
#define __IFC__BOARD_SPECIFIC_H__

#include <Ifc.h>

// On board flash support
#define IFC_NAND_BUF_BASE    0x7E800000

// On board Inegrated flash Controller chip select configuration
#define IFC_NOR_CS    IFC_CS0
#define IFC_NAND_CS   IFC_CS1
#define IFC_FPGA_CS   IFC_CS2

// On board NAND flash properties
#define NAND_PG_SZ        SIZE_2KB //NAND flash page size
#define NAND_SP_SZ        BIT6 //NAND flash spare area 64Bytes
#define NAND_BK_SZ        SIZE_128KB //NAND flash block size
#define NAND_LAST_BLOCK   (SIZE_2KB - 1)

// board-specific NAND timing
#define NAND_FTIM0    (IFC_FTIM0_NAND_TCCST(0x7) | \
                      IFC_FTIM0_NAND_TWP(0x18)   | \
                      IFC_FTIM0_NAND_TWCHT(0x7) | \
                      IFC_FTIM0_NAND_TWH(0xa))

#define NAND_FTIM1    (IFC_FTIM1_NAND_TADLE(0x32) | \
                      IFC_FTIM1_NAND_TWBE(0x39)  | \
                      IFC_FTIM1_NAND_TRR(0xe)   | \
                      IFC_FTIM1_NAND_TRP(0x18))

#define NAND_FTIM2    (IFC_FTIM2_NAND_TRAD(0xf) | \
                      IFC_FTIM2_NAND_TREH(0xa) | \
                      IFC_FTIM2_NAND_TWHRE(0x1e))

#define NAND_FTIM3    0x0

#define NAND_CSPR   (IFC_CSPR_PHYS_ADDR(IFC_NAND_BUF_BASE) \
                            | IFC_CSPR_PORT_SIZE_8 \
                            | IFC_CSPR_MSEL_NAND \
                            | IFC_CSPR_V)

#define NAND_CSPR_EXT   0x0
#define NAND_AMASK      0xFFFF0000

#define NAND_CSOR     (IFC_CSOR_NAND_ECC_ENC_EN /* ECC on encode */ \
                      | IFC_CSOR_NAND_ECC_DEC_EN /* ECC on decode */ \
                      | IFC_CSOR_NAND_ECC_MODE_4 /* 4-bit ECC */ \
                      | IFC_CSOR_NAND_RAL_3       /* RAL = 3 Bytes */ \
                      | IFC_CSOR_NAND_PGS_2K      /* Page Size = 2K */ \
                      | IFC_CSOR_NAND_SPRZ_64     /* Spare size = 64 */ \
                      | IFC_CSOR_NAND_PB(6))     /* 2^6 Pages Per Block */

// board-specific NOR timing
#define NOR_FTIM0     (IFC_FTIM0_NOR_TACSE(0x1) | \
                      IFC_FTIM0_NOR_TEADC(0x1) | \
                      IFC_FTIM0_NOR_TAVDS(0x0) | \
                      IFC_FTIM0_NOR_TEAHC(0xc))
#define NOR_FTIM1     (IFC_FTIM1_NOR_TACO(0x1c) | \
                      IFC_FTIM1_NOR_TRAD_NOR(0xb) |\
                      IFC_FTIM1_NOR_TSEQRAD_NOR(0x9))
#define NOR_FTIM2     (IFC_FTIM2_NOR_TCS(0x1) | \
                      IFC_FTIM2_NOR_TCH(0x4) | \
                      IFC_FTIM2_NOR_TWPH(0x8) | \
                      IFC_FTIM2_NOR_TWP(0x10))
#define NOR_FTIM3     0x0

#define NOR_CSPR      (IFC_CSPR_PHYS_ADDR(FixedPcdGet64 (PcdIfcRegion1BaseAddr)) \
                      | IFC_CSPR_PORT_SIZE_16 \
                      | IFC_CSPR_MSEL_NOR        \
                      | IFC_CSPR_V)

#define NOR_CSPR_EXT  0x0
#define NOR_AMASK     IFC_AMASK(128*1024*1024)
#define NOR_CSOR      (IFC_CSOR_NOR_ADM_SHIFT(4) | \
                      IFC_CSOR_NOR_TRHZ_80)

// board-specific fpga timing
#define FPGA_BASE_PHYS  0x7fb00000
#define FPGA_CSPR_EXT   0x0
#define FPGA_CSPR       (IFC_CSPR_PHYS_ADDR(FPGA_BASE_PHYS) | \
                        IFC_CSPR_PORT_SIZE_8 | \
                        IFC_CSPR_MSEL_GPCM | \
                        IFC_CSPR_V)

#define FPGA_AMASK      IFC_AMASK(64 * 1024)
#define FPGA_CSOR       (IFC_CSOR_NOR_ADM_SHIFT(4) | \
                        IFC_CSOR_NOR_NOR_MODE_AVD_NOR | \
                        IFC_CSOR_NOR_TRHZ_80)

#define FPGA_FTIM0      (IFC_FTIM0_GPCM_TACSE(0xf) | \
                        IFC_FTIM0_GPCM_TEADC(0xf) | \
                        IFC_FTIM0_GPCM_TEAHC(0xf))
#define FPGA_FTIM1      (IFC_FTIM1_GPCM_TACO(0xff) | \
                        IFC_FTIM1_GPCM_TRAD(0x3f))
#define FPGA_FTIM2      (IFC_FTIM2_GPCM_TCS(0xf) | \
                        IFC_FTIM2_GPCM_TCH(0xf) | \
                        IFC_FTIM2_GPCM_TWP(0xff))
#define FPGA_FTIM3      0x0

#endif //__IFC__BOARD_SPECIFIC_H__
