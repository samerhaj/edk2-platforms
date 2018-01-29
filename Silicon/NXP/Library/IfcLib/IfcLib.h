/** @IfcLib.h

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __IFC_LIB_H__
#define __IFC_LIB_H__

#include <Ifc.h>
#include <Uefi.h>

#define IFC_NAND_RESERVED_SIZE      FixedPcdGet32 (PcdIfcNandReservedSize)

typedef enum {
  IFC_FTIM0 = 0,
  IFC_FTIM1,
  IFC_FTIM2,
  IFC_FTIM3,
} IFC_FTIMS;

typedef struct {
  UINT32 CsprExt;
  UINT32 Cspr;
  UINT32 Res;
} IFC_CSPR;

typedef struct {
  UINT32 Amask;
  UINT32 Res[0x2];
} IFC_AMASK;

typedef struct {
  UINT32 Csor;
  UINT32 CsorExt;
  UINT32 Res;
} IFC_CSOR;

typedef struct {
  UINT32 Ftim[4];
  UINT32 Res[0x8];
}IFC_FTIM ;

typedef struct {
  UINT32 Ncfgr;
  UINT32 Res1[0x4];
  UINT32 NandFcr0;
  UINT32 NandFcr1;
  UINT32 Res2[0x8];
  UINT32 Row0;
  UINT32 Res3;
  UINT32 Col0;
  UINT32 Res4;
  UINT32 Row1;
  UINT32 Res5;
  UINT32 Col1;
  UINT32 Res6;
  UINT32 Row2;
  UINT32 Res7;
  UINT32 Col2;
  UINT32 Res8;
  UINT32 Row3;
  UINT32 Res9;
  UINT32 Col3;
  UINT32 Res10[0x24];
  UINT32 NandFbcr;
  UINT32 Res11;
  UINT32 NandFir0;
  UINT32 NandFir1;
  UINT32 nandFir2;
  UINT32 Res12[0x10];
  UINT32 NandCsel;
  UINT32 Res13;
  UINT32 NandSeqStrt;
  UINT32 Res14;
  UINT32 NandEvterStat;
  UINT32 Res15;
  UINT32 PgrdcmplEvtStat;
  UINT32 Res16[0x2];
  UINT32 NandEvterEn;
  UINT32 Res17[0x2];
  UINT32 NandEvterIntrEn;
  UINT32 Res18[0x2];
  UINT32 NandErattr0;
  UINT32 NandErattr1;
  UINT32 Res19[0x10];
  UINT32 NandFsr;
  UINT32 Res20;
  UINT32 NandEccstat[4];
  UINT32 Res21[0x20];
  UINT32 NanNdcr;
  UINT32 Res22[0x2];
  UINT32 NandAutobootTrgr;
  UINT32 Res23;
  UINT32 NandMdr;
  UINT32 Res24[0x5C];
} IFC_NAND;

/*
 * IFC controller NOR Machine registers
 */
typedef struct {
  UINT32 NorEvterStat;
  UINT32 Res1[0x2];
  UINT32 NorEvterEn;
  UINT32 Res2[0x2];
  UINT32 NorEvterIntrEn;
  UINT32 Res3[0x2];
  UINT32 NorErattr0;
  UINT32 NorErattr1;
  UINT32 NorErattr2;
  UINT32 Res4[0x4];
  UINT32 NorCr;
  UINT32 Res5[0xEF];
} IFC_NOR;

/*
 * IFC controller GPCM Machine registers
 */
typedef struct  {
  UINT32 GpcmEvterStat;
  UINT32 Res1[0x2];
  UINT32 GpcmEvterEn;
  UINT32 Res2[0x2];
  UINT32 gpcmEvterIntrEn;
  UINT32 Res3[0x2];
  UINT32 GpcmErattr0;
  UINT32 GpcmErattr1;
  UINT32 GcmErattr2;
  UINT32 GpcmStat;
} IFC_GPCM;

/*
 * IFC Controller Registers
 */
typedef struct {
  UINT32      IfcRev;
  UINT32      Res1[0x2];
  IFC_CSPR    CsprCs[IFC_BANK_COUNT];
  UINT8       Res2[IFC_CSPR_REG_LEN - IFC_CSPR_USED_LEN];
  IFC_AMASK   AmaskCs[IFC_BANK_COUNT];
  UINT8       Res3[IFC_AMASK_REG_LEN - IFC_AMASK_USED_LEN];
  IFC_CSOR    CsorCs[IFC_BANK_COUNT];
  UINT8       Res4[IFC_CSOR_REG_LEN - IFC_CSOR_USED_LEN];
  IFC_FTIM    FtimCs[IFC_BANK_COUNT];
  UINT8       Res5[IFC_FTIM_REG_LEN - IFC_FTIM_USED_LEN];
  UINT32      RbStat;
  UINT32      RbMap;
  UINT32      WpMap;
  UINT32      IfcGcr;
  UINT32      Res7[0x2];
  UINT32      CmEvter_stat;
  UINT32      Res8[0x2];
  UINT32      CmEvterEn;
  UINT32      Res9[0x2];
  UINT32      CmEvterIntrEn;
  UINT32      Res10[0x2];
  UINT32      CmErattr0;
  UINT32      CmErattr1;
  UINT32      Res11[0x2];
  UINT32      IfcCcr;
  UINT32      IfcCsr;
  UINT32      DdrCcrLow;
  UINT32      Res12[IFC_NAND_RESERVED_SIZE];
  IFC_NAND    IfcNand;
  IFC_NOR     IfcNor;
  IFC_GPCM    IfcGpcm;
} IFC_REGS;

extern VOID GetIfcNorFlashTimings (IFC_TIMINGS * NorIfcTimings);

extern VOID GetIfcFpgaTimings (IFC_TIMINGS  *FpgaIfcTimings);

extern VOID GetIfcNandFlashTimings (IFC_TIMINGS * NandIfcTimings);

#endif //__IFC_LIB_H__
