/** @file
  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/
#ifndef _SPI_NOR_FLASH_OPS_H
#define _SPI_NOR_FLASH_OPS_H

/* Include necessary header files here */
#include <Pi/PiSpi.h>
#include <Protocol/SpiIo.h>

#define SFDP_PARAM_PAGE_SIZE(ParamTable)    (1 << ParamTable->PageSize)
#define SFDP_PARAM_FLASH_SIZE(ParamTable)    \
          ((ParamTable->Density & BIT31) ? (1 << (ParamTable->Density & ~BIT31)) : (ParamTable->Density + 1))
#define SFDP_PARAM_ERASE_SIZE(ParamTable)    \
          (1 << ParamTable->Erase_Size_Command[SpiNorParams->EraseIndex].Size)
#define PARAMETER_REV(Major, Minor)         ((Major << 8) | Minor)
#define PARAMETER_ID(Msb, Lsb)              ((Msb << 8) | Lsb)

#pragma pack (1)
typedef struct _SFDP_HEADER {
  UINT32 Signature;
  UINT8  MinorRev;
  UINT8  MajorRev;
  UINT8  NumTableHeader;
  UINT8  AccessProtocol;
} SFDP_HEADER;

typedef struct _SFDP_TABLE_HEADER {
  UINT8  IdLsb;
  UINT8  MinorRev;
  UINT8  MajorRev;
  UINT8  Length;
  UINT32 Pointer : 24;
  UINT8  IdMsb;
} SFDP_TABLE_HEADER;

typedef struct _SFDP_FLASH_PARAM {
  UINT8   EraseSizes : 2;
  UINT8   WriteGranularity : 1;
  UINT8   Reserved1_3 : 5;
  UINT8   Erase4K_Command;
  UINT8   Read_1_1_2 : 1;
  UINT8   AddressBytes : 2;
  UINT8   Dtr : 1;
  UINT8   Read_1_2_2 : 1;
  UINT8   Read_1_4_4 : 1;
  UINT8   Read_1_1_4 : 1;
  UINT16  Reserved1_23 : 9;
  UINT32  Density;
  UINT8   Read_1_4_4_Dummy : 5;
  UINT8   Read_1_4_4_Mode : 3;
  UINT8   Read_1_4_4_Command;
  UINT8   Read_1_1_4_Dummy : 5;
  UINT8   Read_1_1_4_Mode : 3;
  UINT8   Read_1_1_4_Command;
  UINT8   Read_1_1_2_Dummy : 5;
  UINT8   Read_1_1_2_Mode : 3;
  UINT8   Read_1_1_2_Command;
  UINT8   Read_1_2_2_Dummy : 5;
  UINT8   Read_1_2_2_Mode : 3;
  UINT8   Read_1_2_2_Command;
  UINT32  Reserved5[3];
  struct {
    UINT8   Size;
    UINT8   Command;
  } Erase_Size_Command[4];
  UINT8   EraseTimeMultiplier : 4;
  UINT8   Erase1_TypCount : 5;
  UINT8   Erase1_TypUnit : 2;
  UINT8   Erase2_TypCount : 5;
  UINT8   Erase2_TypUnit : 2;
  UINT8   Erase3_TypCount : 5;
  UINT8   Erase3_TypUnit : 2;
  UINT8   Erase4_TypCount : 5;
  UINT8   Erase4_TypUnit : 2;
  UINT8   PageProgramMultipler : 4;
  UINT8   PageSize : 4;
  UINT8   PageProgramTypCount : 5;
  UINT8   PageProgramTypUnit : 1;
  UINT16  Reserved11_14 : 10;
  UINT8   ChipEraseTypCount : 5;
  UINT8   ChipEraseTypUnit : 2;
  UINT8   Reserved11_31 : 1;
  UINT32  Reserved12[2];
  UINT8   Reserved14_0 : 2;
  UINT8   StatusRegPolling : 6;
  UINT32  Reserved14_8 : 24;
  UINT16  Reserved15_0 : 9;
  UINT8   ModeSupported_0_4_4 : 1;
  UINT8   ModeExit_0_4_4 : 6;
  UINT8   ModeEntry_0_4_4 : 4;
  UINT8   QuadEnableReq : 3;
  UINT16  Reserved15_23 : 9;
  UINT8   WriteEnable : 7;
  UINT8   Reserved16_7 : 1;
  UINT8   SoftReset : 6;
  UINT16  Exit4Byte : 10;
  UINT8   Enter4Byte;
  UINT8   Read_1_8_8_Dummy : 5;
  UINT8   Read_1_8_8_Mode : 3;
  UINT8   Read_1_8_8_Command;
  UINT8   Read_1_1_8_Dummy : 5;
  UINT8   Read_1_1_8_Mode : 3;
  UINT8   Read_1_1_8_Command;
  UINT32  Reserved18;
  UINT16  Reserved19_0 : 9;
  UINT8   ModeSupported_0_8_8 : 1;
  UINT8   ModeExit_0_8_8 : 6;
  UINT8   ModeEntry_0_8_8 : 4;
  UINT8   OctalEnableReq : 3;
  UINT16  Reserved19_23 : 9;
} SFDP_FLASH_PARAM;

typedef struct _SFDP_SECTOR_MAP {
  UINT8   SequenceEnd : 1;
  UINT8   DescriptorType : 1;
  UINT8   Reserved1_2 : 6;
  union {
    struct {
      UINT8   Command;
      UINT8   ReadLatency : 4;
      UINT8   Reserved1_20 : 2;
      UINT8   AddressLength : 2;
      UINT8   Mask;
      UINT32  Address;
    } Config;
    struct {
      UINT8   ConfigId;
      UINT8   RegionCount;
      UINT8   Reserved1_24;
      UINT8   Erase_Support : 4;
      UINT8   Reserved2_4 : 4;
      UINT32  RegionSize : 24;
    } Map;
  };
} SFDP_SECTOR_MAP;
#pragma pack()

typedef enum {
  SPI_NOR_REQUEST_TYPE_SFDP_READ,
  SPI_NOR_REQUEST_TYPE_READ,
  SPI_NOR_REQUEST_TYPE_WRITE,
  SPI_NOR_REQUEST_TYPE_ERASE,
  SPI_NOR_REQUEST_TYPE_WRITE_ENABLE,
  SPI_NOR_REQUEST_TYPE_READ_STATUS,
  SPI_NOR_REQUEST_TYPE_MAX
} SPI_NOR_REQUEST_TYPE;

typedef struct _SPI_NOR_PARAMS {
  EFI_SPI_REQUEST_PACKET    *RequestPackets;
  UINT8                     Commands[SPI_NOR_REQUEST_TYPE_MAX];
  UINT32                    FlashOffset;
  UINT8                     Register[2];
  UINT8                     EraseIndex;
  SFDP_FLASH_PARAM          *ParamTable;
  SFDP_TABLE_HEADER         *ParamHeader;
} SPI_NOR_PARAMS;

/**
  This function fill the SPI NOR Flash parameters after parsing SFDP_FLASH_PARAM Table

  @param[in]  ParamTable  Pointer to SFDP parameter table read from flash
  @param[in]  ParamHeader Pointer to SFDP parameter table Header read from flash
  @param[out] SpiNorParam Pointer to SPI_NOR_PARAMS
**/
EFI_STATUS
FillFlashParam (
  IN  EFI_SPI_IO_PROTOCOL           *SpiIo,
  IN  SPI_NOR_PARAMS                *SpiNorParams
);

EFI_STATUS
ReadSfdpParameterTable (
  IN  EFI_SPI_IO_PROTOCOL           *SpiIo,
  IN  SPI_NOR_PARAMS                *SpiNorParams,
  IN  UINT16                        ParameterId,
  IN  UINT16                        ParameterRev,
  OUT SFDP_TABLE_HEADER             **TableHeader,
  OUT VOID                          **Table,
  IN  BOOLEAN                       Runtime
  );

EFI_STATUS
ReadFlashData (
  IN  EFI_SPI_IO_PROTOCOL           *SpiIo,
  IN  SPI_NOR_PARAMS                *SpiNorParams,
  IN  UINTN                         From,
  IN  UINTN                         Length,
  IN  UINT8                         *ReadBuf
  );

EFI_STATUS
WriteFlashData (
  IN  EFI_SPI_IO_PROTOCOL           *SpiIo,
  IN  SPI_NOR_PARAMS                *SpiNorParams,
  IN  UINTN                         To,
  IN  UINTN                         Length,
  IN  UINT8                         *WriteBuf
  );

EFI_STATUS
EraseFlashBlock (
  IN  EFI_SPI_IO_PROTOCOL           *SpiIo,
  IN  SPI_NOR_PARAMS                *SpiNorParams,
  IN  UINTN                         Offset
  );

UINT64
GetMaxTimeout (
  IN  SPI_NOR_PARAMS          *SpiNorParams,
  IN  SPI_NOR_REQUEST_TYPE    RequestType
  );

#endif // _SPI_NOR_FLASH_OPS_H
