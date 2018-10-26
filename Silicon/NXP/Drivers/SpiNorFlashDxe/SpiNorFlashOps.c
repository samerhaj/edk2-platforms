/** @file
 The EFI_SPI_NOR_FLASH_PROTOCOL exists in the SPI peripheral layer.
 This protocol manipulates the SPI NOR flash parts using a common set
 of commands. The board layer provides the interconnection and
 configuration details for the SPI NOR flash part. The SPI NOR flash
 driver uses this configuration data to expose a generic interface
 which provides the following APls:
   •Read manufacture and device ID
   •Read data
   •Read data using low frequency
   •Read status
   •Write data
   •Erase 4 KiB blocks
   •Erase 32 or 64 KiB blocks
   •Write status

 Copyright 2018 NXP.

 This program and the accompanying materials
 are licensed and made available under the terms and conditions of
 the BSD License which accompanies this distribution. The full
 text of the license may be found at
 http://opensource.org/licenses/bsd-license.php

 THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS"
 BASIS, WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER
 EXPRESS OR IMPLIED.

 @par Specification Reference:
   - PI 1.6, Chapter 18, Spi Protocol Stack
**/
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>

#include "SpiNorFlashOps.h"

EFI_SPI_REQUEST_PACKET*
SpiNorGetRequestPacket (
  IN  SPI_NOR_PARAMS        *SpiNorParams,
  IN  SPI_NOR_REQUEST_TYPE  RequestType
  )
{
  SPI_NOR_REQUEST_TYPE    Index;
  EFI_SPI_REQUEST_PACKET  *RequestPacket;

  RequestPacket = SpiNorParams->RequestPackets;
  for (Index = SPI_NOR_REQUEST_TYPE_SFDP_READ;
       Index < RequestType;
       Index++) {
    RequestPacket = (VOID *)RequestPacket +
                    (RequestPacket->TransactionCount * sizeof (EFI_SPI_BUS_TRANSACTION) + sizeof (UINTN));
  }

  return RequestPacket;
}

EFI_STATUS
FillRequestPacketData (
  IN  SPI_NOR_REQUEST_TYPE    RequestType,
  IN  SPI_NOR_PARAMS          *SpiNorParams,
  IN  EFI_SPI_REQUEST_PACKET  *RequestPacket,
  IN  UINT32                  Address,
  IN  VOID                    *Data,
  IN  UINT32                  DataLength
  )
{
  UINTN   Index;

  for (Index = 0; Index < RequestPacket->TransactionCount; Index++) {
    switch (RequestPacket->Transaction[Index].TransactionType) {
      case SPI_TRANSACTION_ADDRESS:
        SpiNorParams->FlashOffset = Address;
        RequestPacket->Transaction[Index].WriteBuffer = (UINT8 *)&SpiNorParams->FlashOffset;
        break;
      case SPI_TRANSACTION_DATA:
        switch (RequestType) {
          case SPI_NOR_REQUEST_TYPE_SFDP_READ:
          case SPI_NOR_REQUEST_TYPE_READ:
            RequestPacket->Transaction[Index].ReadBuffer = Data;
            break;
          case SPI_NOR_REQUEST_TYPE_WRITE:
            RequestPacket->Transaction[Index].WriteBuffer = Data;
            break;
          default:
            break;
        }
        RequestPacket->Transaction[Index].Length = DataLength;
        break;
      default:
        break;
    }
  }

  return EFI_SUCCESS;
}

UINT64
GetMaxTimeout (
  IN  SPI_NOR_PARAMS          *SpiNorParams,
  IN  SPI_NOR_REQUEST_TYPE    RequestType
  )
{
  UINT64                TimeUsec;
  SFDP_FLASH_PARAM      *ParamTable;

  ParamTable = SpiNorParams->ParamTable;
  TimeUsec = 0;
  switch (RequestType) {
    case SPI_NOR_REQUEST_TYPE_WRITE:
      if (ParamTable->PageProgramTypUnit) {
        TimeUsec = 64;
      } else {
        TimeUsec = 8;
      }
      TimeUsec *= (ParamTable->PageProgramTypCount + 1);
      TimeUsec *= 2 * (ParamTable->PageProgramMultipler + 1);
      break;

    case SPI_NOR_REQUEST_TYPE_ERASE:
      switch (SpiNorParams->EraseIndex) {
        case 0:
          switch (ParamTable->Erase1_TypUnit) {
            case 0:
              TimeUsec = 1000;
              break;
            case 1:
              TimeUsec = 16000;
              break;
            case 2:
              TimeUsec = 128000;
              break;
            case 3:
              TimeUsec = 1000000;
              break;
            default:
              break;
          }
          TimeUsec *= (ParamTable->Erase1_TypCount + 1);
          break;

        case 1:
          switch (ParamTable->Erase2_TypUnit) {
            case 0:
              TimeUsec = 1000;
              break;
            case 1:
              TimeUsec = 16000;
              break;
            case 2:
              TimeUsec = 128000;
              break;
            case 3:
              TimeUsec = 1000000;
              break;
            default:
              break;
          }
          TimeUsec *= (ParamTable->Erase2_TypCount + 1);
          break;

        case 2:
          switch (ParamTable->Erase3_TypUnit) {
            case 0:
              TimeUsec = 1000;
              break;
            case 1:
              TimeUsec = 16000;
              break;
            case 2:
              TimeUsec = 128000;
              break;
            case 3:
              TimeUsec = 1000000;
              break;
            default:
              break;
          }
          TimeUsec *= (ParamTable->Erase3_TypCount + 1);
          break;

        case 3:
          switch (ParamTable->Erase4_TypUnit) {
            case 0:
              TimeUsec = 1000;
              break;
            case 1:
              TimeUsec = 16000;
              break;
            case 2:
              TimeUsec = 128000;
              break;
            case 3:
              TimeUsec = 1000000;
              break;
            default:
              break;
          }
          TimeUsec *= (ParamTable->Erase4_TypCount + 1);
          break;
      }
      TimeUsec *= 2 * (ParamTable->EraseTimeMultiplier + 1);
      break;

    default:
      break;
  }

  return TimeUsec;
}

EFI_STATUS
WaitForOperation (
  IN  EFI_SPI_IO_PROTOCOL     *SpiIo,
  IN  SPI_NOR_PARAMS          *SpiNorParams,
  IN  SPI_NOR_REQUEST_TYPE    RequestType
  )
{
  EFI_SPI_REQUEST_PACKET        *RequestPacket;
  UINT64                        MaxTimeout;
  UINT64                        StartTime;
  SFDP_FLASH_PARAM              *ParamTable;
  EFI_STATUS                    Status;

  ParamTable = SpiNorParams->ParamTable;
  RequestPacket = SpiNorGetRequestPacket (SpiNorParams, SPI_NOR_REQUEST_TYPE_READ_STATUS);
  MaxTimeout = GetMaxTimeout (SpiNorParams, RequestType);
  Status = EFI_SUCCESS;

  StartTime = GetTimeInNanoSecond(GetPerformanceCounter());
  while (TRUE) {
    Status = SpiIo->Transaction (
                      SpiIo,
                      RequestPacket,
                      0
                      );
    if (EFI_ERROR (Status)) {
      break;
    }
    if ((ParamTable->StatusRegPolling & BIT1) &&
        (SpiNorParams->Register[0] & BIT7)) {
      break;
    } else if ((ParamTable->StatusRegPolling & BIT0) &&
               !(SpiNorParams->Register[0] & BIT0)) {
      break;
    }

    if ((GetTimeInNanoSecond(GetPerformanceCounter()) - StartTime) >= (MaxTimeout * 1000)) {
      Status = EFI_TIMEOUT;
      break;
    }
  }

  return Status;
}

EFI_STATUS
FillSfdpReadRequestPacket (
  IN  EFI_SPI_REQUEST_PACKET        *RequestPacket,
  IN  SPI_NOR_PARAMS                *SpiNorParams
  )
{
  UINT8         Index;

  SpiNorParams->Commands[SPI_NOR_REQUEST_TYPE_SFDP_READ] = 0x5A;

  Index = 0;
  // Send Command
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_COMMAND;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[Index].Length = 1;
  RequestPacket->Transaction[Index].FrameSize = 8;
  RequestPacket->Transaction[Index++].WriteBuffer = &SpiNorParams->Commands[SPI_NOR_REQUEST_TYPE_SFDP_READ];

  // Send Address
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_ADDRESS;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[Index].Length = 3; // 24 bit address
  RequestPacket->Transaction[Index].FrameSize = 8;
  // Put the address when starting the read transaction using SPI IO protocol
  RequestPacket->Transaction[Index++].WriteBuffer = NULL;

  // 8 dummy bytes
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_DUMMY;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[Index].Length = 8;
  RequestPacket->Transaction[Index].FrameSize = 8;
  // Put the address when starting the read transaction using SPI IO protocol
  RequestPacket->Transaction[Index++].WriteBuffer = NULL;

  // Read Data
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_DATA;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  // Put the buffer size when starting the read transaction using SPI IO protocol
  RequestPacket->Transaction[Index].Length = 0;
  RequestPacket->Transaction[Index].FrameSize = 8;
  // Put the Read buffer pointer when starting the read transaction using SPI IO protocol
  RequestPacket->Transaction[Index++].ReadBuffer = NULL;

  RequestPacket->TransactionCount = Index;

  return EFI_SUCCESS;
}

EFI_STATUS
FillReadRequestPacket (
  IN  EFI_SPI_REQUEST_PACKET        *RequestPacket,
  IN  SPI_NOR_PARAMS                *SpiNorParams
  )
{
  UINT8         Index;

  // TO DO : parse ParamTable and put the maximum supported command.
  SpiNorParams->Commands[SPI_NOR_REQUEST_TYPE_READ] = 0x03;

  Index = 0;
  // Send Command
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_COMMAND;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[Index].Length = 1;
  RequestPacket->Transaction[Index].FrameSize = 8;
  RequestPacket->Transaction[Index++].WriteBuffer = &SpiNorParams->Commands[SPI_NOR_REQUEST_TYPE_READ];

  // Send Address
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_ADDRESS;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[Index].Length = 3; // 24 bit address
  RequestPacket->Transaction[Index].FrameSize = 8;
  // Put the address when starting the read transaction using SPI IO protocol
  RequestPacket->Transaction[Index++].WriteBuffer = NULL;

  // Read Data
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_DATA;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  // Put the buffer size when starting the read transaction using SPI IO protocol
  RequestPacket->Transaction[Index].Length = 0;
  RequestPacket->Transaction[Index].FrameSize = 8;
  // Put the Read buffer pointer when starting the read transaction using SPI IO protocol
  RequestPacket->Transaction[Index++].ReadBuffer = NULL;

  RequestPacket->TransactionCount = Index;

  return EFI_SUCCESS;
}

EFI_STATUS
FillWriteRequestPacket (
  IN  EFI_SPI_REQUEST_PACKET        *RequestPacket,
  IN  SPI_NOR_PARAMS                *SpiNorParams
  )
{
  UINT8         Index;

  // TO DO : parse ParamTable and put the maximum supported command.
  SpiNorParams->Commands[SPI_NOR_REQUEST_TYPE_WRITE] = 0x02;

  Index = 0;
  // Send Command
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_COMMAND;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[Index].Length = 1;
  RequestPacket->Transaction[Index].FrameSize = 8;
  RequestPacket->Transaction[Index++].WriteBuffer = &SpiNorParams->Commands[SPI_NOR_REQUEST_TYPE_WRITE];

  // Send Address
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_ADDRESS;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[Index].Length = 3; // 24 bit address
  RequestPacket->Transaction[Index].FrameSize = 8;
  // Put the address when starting the read transaction using SPI IO protocol
  RequestPacket->Transaction[Index++].WriteBuffer = NULL;

  // Write Data
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_DATA;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  // Put the buffer size when starting the write transaction using SPI IO protocol
  RequestPacket->Transaction[Index].Length = 0;
  RequestPacket->Transaction[Index].FrameSize = 8;
  // Put the Read buffer pointer when starting the write transaction using SPI IO protocol
  RequestPacket->Transaction[Index++].WriteBuffer = NULL;

  RequestPacket->TransactionCount = Index;

  return EFI_SUCCESS;
}

EFI_STATUS
FillEraseRequestPacket (
  IN  EFI_SPI_REQUEST_PACKET        *RequestPacket,
  IN  SPI_NOR_PARAMS                *SpiNorParams
  )
{
  UINT8                 Index;
  SFDP_FLASH_PARAM      *ParamTable;

  ParamTable = SpiNorParams->ParamTable;

  // TO DO : parse ParamTable and Map table and put the maximum supported command.
  if (ParamTable->EraseSizes == 0x01) {
    for (Index = 0; Index < ARRAY_SIZE (ParamTable->Erase_Size_Command); Index++) {
      if (ParamTable->Erase_Size_Command[Index].Command == ParamTable->Erase4K_Command) {
        break;
      }
    }
    SpiNorParams->EraseIndex = Index;
  }

  SpiNorParams->EraseIndex = 2;
  SpiNorParams->Commands[SPI_NOR_REQUEST_TYPE_ERASE] = ParamTable->Erase_Size_Command[SpiNorParams->EraseIndex].Command;
  Index = 0;
  // Send Command
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_COMMAND;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[Index].Length = 1;
  RequestPacket->Transaction[Index].FrameSize = 8;
  RequestPacket->Transaction[Index++].WriteBuffer = &SpiNorParams->Commands[SPI_NOR_REQUEST_TYPE_ERASE];

  // Send Address
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_ADDRESS;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[Index].Length = 3; // 24 bit address
  RequestPacket->Transaction[Index].FrameSize = 8;
  // Put the address when starting the read transaction using SPI IO protocol
  RequestPacket->Transaction[Index++].WriteBuffer = NULL;

  RequestPacket->TransactionCount = Index;

  return EFI_SUCCESS;
}

EFI_STATUS
FillReadStatusRequestPacket (
  IN  EFI_SPI_REQUEST_PACKET        *RequestPacket,
  IN  SPI_NOR_PARAMS                *SpiNorParams
  )
{
  UINT8                     Index;
  SFDP_FLASH_PARAM          *ParamTable;

  ParamTable = SpiNorParams->ParamTable;
  if (ParamTable->StatusRegPolling & BIT0) {
    SpiNorParams->Commands[SPI_NOR_REQUEST_TYPE_READ_STATUS] = 0x05;
  } else if (ParamTable->StatusRegPolling & BIT1) {
    SpiNorParams->Commands[SPI_NOR_REQUEST_TYPE_READ_STATUS] = 0x70;
  }

  Index = 0;
  // Send Command
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_COMMAND;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[Index].Length = 1;
  RequestPacket->Transaction[Index].FrameSize = 8;
  RequestPacket->Transaction[Index++].WriteBuffer = &SpiNorParams->Commands[SPI_NOR_REQUEST_TYPE_READ_STATUS];

  // Read Register
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_DATA;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[Index].Length = 1;
  RequestPacket->Transaction[Index].FrameSize = 8;
  RequestPacket->Transaction[Index++].ReadBuffer = &SpiNorParams->Register[0];

  RequestPacket->TransactionCount = Index;

  return EFI_SUCCESS;
}

EFI_STATUS
FillWriteEnableRequestPacket (
  IN  EFI_SPI_REQUEST_PACKET        *RequestPacket,
  IN  SPI_NOR_PARAMS                *SpiNorParams
  )
{
  UINT8         Index;

  SpiNorParams->Commands[SPI_NOR_REQUEST_TYPE_WRITE_ENABLE] = 0x06;

  Index = 0;
  // Send Command
  RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_COMMAND;
  RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[Index].Length = 1;
  RequestPacket->Transaction[Index].FrameSize = 8;
  RequestPacket->Transaction[Index++].WriteBuffer = &SpiNorParams->Commands[SPI_NOR_REQUEST_TYPE_WRITE_ENABLE];

  RequestPacket->TransactionCount = Index;

  return EFI_SUCCESS;
}

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
  )
{
  EFI_STATUS                Status;
  EFI_SPI_REQUEST_PACKET    *RequestPacket;
  SPI_NOR_REQUEST_TYPE      Index;

  Status = EFI_SUCCESS;
  for (Index = SPI_NOR_REQUEST_TYPE_SFDP_READ;
       Index < SPI_NOR_REQUEST_TYPE_MAX;
       Index++) {
    RequestPacket = SpiNorGetRequestPacket (SpiNorParams, Index);

    switch (Index) {
      case SPI_NOR_REQUEST_TYPE_SFDP_READ:
        Status = FillSfdpReadRequestPacket (RequestPacket, SpiNorParams);
        break;
      case SPI_NOR_REQUEST_TYPE_READ:
        Status = FillReadRequestPacket (RequestPacket, SpiNorParams);
        break;
      case SPI_NOR_REQUEST_TYPE_WRITE:
        Status = FillWriteRequestPacket (RequestPacket, SpiNorParams);
        break;
      case SPI_NOR_REQUEST_TYPE_ERASE:
        Status = FillEraseRequestPacket (RequestPacket, SpiNorParams);
        break;
      case SPI_NOR_REQUEST_TYPE_READ_STATUS:
        Status = FillReadStatusRequestPacket (RequestPacket, SpiNorParams);
        break;
      case SPI_NOR_REQUEST_TYPE_WRITE_ENABLE:
        Status = FillWriteEnableRequestPacket (RequestPacket, SpiNorParams);
        break;
      default:
        break;
    }
    if (EFI_ERROR (Status)) {
      break;
    }
  }

  return Status;
}

EFI_STATUS
ReadSfdpParameterTable (
  IN  EFI_SPI_IO_PROTOCOL           *SpiIo,
  IN  SPI_NOR_PARAMS                *SpiNorParams,
  IN  UINT16                        ParameterId,
  IN  UINT16                        ParameterRev,
  OUT SFDP_TABLE_HEADER             **TableHeader,
  OUT VOID                          **Table,
  IN  BOOLEAN                       Runtime
  )
{
  SFDP_HEADER                   SFDPHeader;
  SFDP_TABLE_HEADER             *TableHeaders;
  EFI_SPI_REQUEST_PACKET        *RequestPacket;
  EFI_STATUS                    Status;
  INTN                          I;

  TableHeaders = NULL;
  *Table = NULL;
  *TableHeader = NULL;

  RequestPacket = SpiNorGetRequestPacket (SpiNorParams, SPI_NOR_REQUEST_TYPE_SFDP_READ);
  Status = FillSfdpReadRequestPacket (RequestPacket, SpiNorParams);
  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }
  // Fill SFDP read request packet
  FillRequestPacketData (
    SPI_NOR_REQUEST_TYPE_SFDP_READ,
    SpiNorParams,
    RequestPacket,
    0,
    &SFDPHeader,
    sizeof (SFDPHeader)
    );
  // read SFDP Header
  Status = SpiIo->Transaction (
                    SpiIo,
                    RequestPacket,
                    MHz (50)
                    );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Error Reading SFDP Header %r\n", Status));
    goto ErrorExit;
  }

  if (SFDPHeader.Signature != SIGNATURE_32 ('S', 'F', 'D', 'P')) {
    DEBUG ((DEBUG_ERROR, "Header Signature 0x%08x invalid\n", SFDPHeader.Signature));
    Status = EFI_NOT_FOUND;
    goto ErrorExit;
  }

  if (PARAMETER_REV (SFDPHeader.MajorRev, SFDPHeader.MinorRev) < PARAMETER_REV (1, 6)) {
    DEBUG ((
      DEBUG_ERROR, "Unsupported SFDP parameters Rev(Maj 0x%024x, Min 0x%02x)\n",
      SFDPHeader.MajorRev, SFDPHeader.MinorRev
      ));
    Status = EFI_NOT_FOUND;
    goto ErrorExit;
  }

  TableHeaders = (SFDP_TABLE_HEADER *)AllocateZeroPool (
                                        sizeof (SFDP_TABLE_HEADER) *
                                        (SFDPHeader.NumTableHeader + 1)
                                        );
  // Fill SFDP read request packet
  FillRequestPacketData (
    SPI_NOR_REQUEST_TYPE_SFDP_READ,
    SpiNorParams,
    RequestPacket,
    sizeof (SFDPHeader),
    TableHeaders,
    sizeof (SFDP_TABLE_HEADER) * (SFDPHeader.NumTableHeader + 1)
    );
  // read SFDP Tables' Headers
  Status = SpiIo->Transaction (
                    SpiIo,
                    RequestPacket,
                    MHz (50)
                    );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Error Reading SFDP Tables' Headers Status %r\n", Status));
    goto ErrorExit;
  }

  for (I = 0; I <= SFDPHeader.NumTableHeader; I++) {
    if ((PARAMETER_ID (TableHeaders[I].IdMsb, TableHeaders[I].IdLsb) == ParameterId) &&
        (PARAMETER_REV (TableHeaders[I].MajorRev, TableHeaders[I].MinorRev) >= ParameterRev)) {
      break;
    }
  }

  if (I > SFDPHeader.NumTableHeader) {
    DEBUG ((
      DEBUG_ERROR, "Could not find Parameter Table Id 0x%04x Rev 0x%04x\n",
      ParameterId,
      ParameterRev
      ));
    Status = EFI_NOT_FOUND;
    goto ErrorExit;
  } else if (PARAMETER_REV (TableHeaders[I].MajorRev, TableHeaders[I].MinorRev) > ParameterRev) {
    DEBUG ((
      DEBUG_WARN, "Parameter Rev requested 0x%04x found 0x%04x for Parameter Id 0x%04x\n",
      PARAMETER_REV (TableHeaders[I].MajorRev, TableHeaders[I].MinorRev),
      ParameterRev,
      ParameterId
      ));
  }

  if (Runtime) {
    *Table = AllocateRuntimeZeroPool (TableHeaders[I].Length * sizeof (UINT32));
    *TableHeader = AllocateRuntimeCopyPool (sizeof (SFDP_TABLE_HEADER), &TableHeaders[I]);
  } else {
    *Table = AllocateZeroPool (TableHeaders[I].Length * sizeof (UINT32));
    *TableHeader = AllocateCopyPool (sizeof (SFDP_TABLE_HEADER), &TableHeaders[I]);
  }

  if ((*Table == NULL) || (*TableHeader == NULL)) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }

  // Fill SFDP read request packet
  FillRequestPacketData (
    SPI_NOR_REQUEST_TYPE_SFDP_READ,
    SpiNorParams,
    RequestPacket,
    TableHeaders[I].Pointer,
    *Table,
    TableHeaders[I].Length * sizeof (UINT32)
    );
  // read SFDP Table
  Status = SpiIo->Transaction (
                    SpiIo,
                    RequestPacket,
                    MHz (50)
                    );
  if (EFI_ERROR (Status)) {
    DEBUG ((
      DEBUG_ERROR, "Error Reading SFDP Table Id 0x%04x Rev 0x%04x Status %r\n",
      ParameterId,
      ParameterRev,
      Status
      ));
    goto ErrorExit;
  }

ErrorExit:
  if (TableHeaders) {
    FreePool (TableHeaders);
  }

  if (EFI_ERROR (Status)) {
    if (*Table) {
      FreePool (*Table);
    }
    if (*TableHeader) {
      FreePool (*TableHeader);
    }
  }

  return Status;
}

EFI_STATUS
ReadFlashData (
  IN  EFI_SPI_IO_PROTOCOL           *SpiIo,
  IN  SPI_NOR_PARAMS                *SpiNorParams,
  IN  UINTN                         From,
  IN  UINTN                         Length,
  IN  UINT8                         *ReadBuf
  )
{
  EFI_SPI_REQUEST_PACKET        *RequestPacket;
  EFI_STATUS                    Status;

  RequestPacket = SpiNorGetRequestPacket (SpiNorParams, SPI_NOR_REQUEST_TYPE_READ);
  // Fill Flash read request packet
  FillRequestPacketData (
    SPI_NOR_REQUEST_TYPE_READ,
    SpiNorParams,
    RequestPacket,
    From,
    ReadBuf,
    Length
    );
  // read Flash Data
  Status = SpiIo->Transaction (
                    SpiIo,
                    RequestPacket,
                    0
                    );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Error Reading Flash Data %r\n", Status));
  }

  return Status;
}

EFI_STATUS
WriteFlashData (
  IN  EFI_SPI_IO_PROTOCOL           *SpiIo,
  IN  SPI_NOR_PARAMS                *SpiNorParams,
  IN  UINTN                         To,
  IN  UINTN                         Length,
  IN  UINT8                         *WriteBuf
  )
{
  EFI_SPI_REQUEST_PACKET        *RequestPacket;
  EFI_STATUS                    Status;

  RequestPacket = SpiNorGetRequestPacket (SpiNorParams, SPI_NOR_REQUEST_TYPE_WRITE_ENABLE);
  // Issue Write enable command
  Status = SpiIo->Transaction (
                    SpiIo,
                    RequestPacket,
                    0
                    );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Error sending write enable %r\n", Status));
    goto ErrorExit;
  }

  RequestPacket = SpiNorGetRequestPacket (SpiNorParams, SPI_NOR_REQUEST_TYPE_WRITE);
  // Fill Flash Write request packet
  FillRequestPacketData (
    SPI_NOR_REQUEST_TYPE_WRITE,
    SpiNorParams,
    RequestPacket,
    To,
    WriteBuf,
    Length
    );
  // Issue Write command
  Status = SpiIo->Transaction (
                    SpiIo,
                    RequestPacket,
                    0
                    );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Error sending write command %r\n", Status));
    goto ErrorExit;
  }

  // Poll for write done
  Status = WaitForOperation (SpiIo, SpiNorParams, SPI_NOR_REQUEST_TYPE_WRITE);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Error during write %r\n", Status));
  }

ErrorExit:
  return Status;
}

EFI_STATUS
EraseFlashBlock (
  IN  EFI_SPI_IO_PROTOCOL           *SpiIo,
  IN  SPI_NOR_PARAMS                *SpiNorParams,
  IN  UINTN                         Offset
  )
{
  EFI_SPI_REQUEST_PACKET        *RequestPacket;
  EFI_STATUS                    Status;

  RequestPacket = SpiNorGetRequestPacket (SpiNorParams, SPI_NOR_REQUEST_TYPE_WRITE_ENABLE);
  // Issue Write enable command
  Status = SpiIo->Transaction (
                    SpiIo,
                    RequestPacket,
                    0
                    );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Error sending write enable %r\n", Status));
    goto ErrorExit;
  }

  RequestPacket = SpiNorGetRequestPacket (SpiNorParams, SPI_NOR_REQUEST_TYPE_ERASE);
  // Fill Flash Write request packet
  FillRequestPacketData (
    SPI_NOR_REQUEST_TYPE_ERASE,
    SpiNorParams,
    RequestPacket,
    Offset,
    NULL,
    0
    );
  // Issue Erase command
  Status = SpiIo->Transaction (
                    SpiIo,
                    RequestPacket,
                    0
                    );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Error sending write command %r\n", Status));
    goto ErrorExit;
  }

  // Poll for Erase done
  Status = WaitForOperation (SpiIo, SpiNorParams, SPI_NOR_REQUEST_TYPE_ERASE);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Error during Erase %r\n", Status));
  }

ErrorExit:
  return Status;
}
