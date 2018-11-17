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
      case SPI_TRANSACTION_COMMAND:
        RequestPacket->Transaction[Index].WriteBuffer = (UINT8 *)&SpiNorParams->Commands[RequestType];
        break;
      case SPI_TRANSACTION_ADDRESS:
        SpiNorParams->FlashOffset = Address;
        RequestPacket->Transaction[Index].WriteBuffer = (UINT8 *)&SpiNorParams->FlashOffset;
        break;
      case SPI_TRANSACTION_DATA:
        switch (RequestType) {
          case SPI_NOR_REQUEST_TYPE_READ_STATUS:
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
  FillRequestPacketData (
    SPI_NOR_REQUEST_TYPE_READ_STATUS,
    SpiNorParams,
    RequestPacket,
    0,
    &SpiNorParams->Register[0],
    sizeof (SpiNorParams->Register[0])
  );
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
GetEraseCommandIndex (
  IN  EFI_SPI_IO_PROTOCOL     *SpiIo,
  IN  SPI_NOR_PARAMS          *SpiNorParams,
  OUT UINT8                   *EraseIndex
  )
{
  SFDP_SECTOR_MAP               *Table;
  SFDP_SECTOR_MAP               *TableBackup;
  SFDP_TABLE_HEADER             *TableHeader;
  EFI_SPI_REQUEST_PACKET        *RequestPacket;
  UINTN                         Index;
  UINT8                         ConfigRegister;
  UINT8                         SectorConfig;
  UINT32                        RegionEraseSupport;
  EFI_STATUS                    Status;

  SectorConfig = 0;
  TableBackup = NULL;
  TableHeader = NULL;
  RequestPacket = (EFI_SPI_REQUEST_PACKET *)AllocateZeroPool (sizeof (UINTN) + 4 * sizeof (EFI_SPI_BUS_TRANSACTION));
  if (!RequestPacket) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }

  // Read the JEDEC Sector Map Parameter Header and Table
  Status = ReadSfdpParameterTable (
             SpiIo,
             SpiNorParams,
             PARAMETER_ID (0xFF, 0x81),
             PARAMETER_REV (1, 0),
             &TableHeader,
             (VOID **)&TableBackup,
             FALSE
             );
  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }

  Table = TableBackup;

  // Parse the config registers and get a configuration
  while (!Table->DescriptorType) {
    Index = 0;
    // Send Command
    RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_COMMAND;
    RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
    RequestPacket->Transaction[Index].Length = 1;
    RequestPacket->Transaction[Index].FrameSize = 8;
    RequestPacket->Transaction[Index++].WriteBuffer = &Table->Config.Command;

    if (Table->Config.AddressLength) {
      // Send Address
      RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_ADDRESS;
      RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
      // At this point we have only used read SFDP (5A) command
      // so for variable length address use the same address length as used for SFDP command
      if (Table->Config.AddressLength == 1 || Table->Config.AddressLength == 3) {
        RequestPacket->Transaction[Index].Length = 3; // 24 bit address
      } else {
        // TODO : if required Enter 4-Byte Addressing
        RequestPacket->Transaction[Index].Length = 4; // 32 bit address
      }
      RequestPacket->Transaction[Index].FrameSize = 8;
      RequestPacket->Transaction[Index++].WriteBuffer = (UINT8 *)&Table->Config.Address;
    }

    if (Table->Config.ReadLatency) {
      // dummy bytes
      RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_DUMMY;
      RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
      // Range from 0 to 14 cycles of read latency, in clock cycles
      // A value of Fh indicates the read latency is variable. The software or hardware controlling
      // the memory is aware of the latency last set in the memory device and this same value is used
      // in the configuration detection command.
      // At this point we have only used read SFDP (5A) command
      // so for variable length read latency use the same dummy cycles as used for SFDP command
      if (Table->Config.ReadLatency == 0xf) {
        RequestPacket->Transaction[Index].Length = 8;
      } else {
        RequestPacket->Transaction[Index].Length = Table->Config.ReadLatency;
      }
      RequestPacket->Transaction[Index++].FrameSize = 8;
    }

    // Read one byte of Data
    RequestPacket->Transaction[Index].TransactionType = SPI_TRANSACTION_DATA;
    RequestPacket->Transaction[Index].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
    RequestPacket->Transaction[Index].Length = 1;
    RequestPacket->Transaction[Index].FrameSize = 8;
    RequestPacket->Transaction[Index++].ReadBuffer = &ConfigRegister;

    RequestPacket->TransactionCount = Index;

    Status = SpiIo->Transaction (
                      SpiIo,
                      RequestPacket,
                      MHz (50)
                      );
    if (EFI_ERROR (Status)) {
      DEBUG ((
        DEBUG_ERROR, "Error Reading Sector configuration register @ 0x%08x %r\n",
        Table->Config.Address,
        Status
        ));
      goto ErrorExit;
    }

    if (Table->Config.Address == 4)
      ConfigRegister |= (BIT3 | BIT1);
    else if (Table->Config.Address == 2)
      ConfigRegister &= ~BIT2;

    // Mask the data read
    ConfigRegister &= Table->Config.Mask;
    // Move the selected bit to LSB
    ConfigRegister >>= __builtin_ctz(Table->Config.Mask);
    // configuration bits are read in the following MSb to LSb order to form the configuration map index value:
    SectorConfig = (SectorConfig << 1 | (0x01 & ConfigRegister));

    Table++;
  }

  // The command descriptors are optional.
  // If there is a single configuration then no command descriptors are needed
  if (Table != TableBackup) {
    // Now that we have sector Config, search that config in sector map
    while (TRUE) {
      if (Table->Map.ConfigId == SectorConfig) {
        break;
      } else if (Table->SequenceEnd) {
        DEBUG ((DEBUG_ERROR, "no sector map found for SectorConfig = 0x%02x\n", SectorConfig));
        Status = EFI_NOT_FOUND;
        goto ErrorExit;
      }
      // Move table pointer after the regions of this map
      Table = (SFDP_SECTOR_MAP *)((UINT32 *)Table + Table->Map.RegionCount);
      Table++;
    }
  }

  // Look for supported Erase sizes in regions in sector map
  RegionEraseSupport = Table->Map.Erase_Support;
  if (Table->Map.RegionCount) {
    // There are multiple regions in sector map
    // look for an erase size that is supported by all the regions
    for (Index = 1; Index < Table->Map.RegionCount; Index++) {
      // Move Table Pointer by one Dword
      Table = (SFDP_SECTOR_MAP *)((UINT32 *)Table + 1);
      RegionEraseSupport &= Table->Map.Erase_Support;
    }
  }

  if (!RegionEraseSupport) {
    DEBUG ((DEBUG_ERROR, "No common erase size for all the regions for sector map config 0x%02x\n", SectorConfig));
    Status = EFI_NOT_FOUND;
    goto ErrorExit;
  }

  *EraseIndex = __builtin_ctz(RegionEraseSupport);

ErrorExit:
  if (TableBackup) {
    FreePool (TableBackup);
  }
  if (TableHeader) {
    FreePool (TableHeader);
  }

  if (RequestPacket) {
    FreePool (RequestPacket);
  }

  return Status;
}

EFI_STATUS
FillEraseRequestPacket (
  IN  EFI_SPI_IO_PROTOCOL           *SpiIo,
  IN  EFI_SPI_REQUEST_PACKET        *RequestPacket,
  IN  SPI_NOR_PARAMS                *SpiNorParams
  )
{
  UINT8                 Index;
  SFDP_FLASH_PARAM      *ParamTable;
  BOOLEAN               NeedParseSectorMapTable;
  EFI_STATUS            Status;

  ParamTable = SpiNorParams->ParamTable;
  NeedParseSectorMapTable = FALSE;

  if (ParamTable->EraseSizes == 0x01) {
    // 4 kilobyte Erase is supported throughout the device
    // NOTE If the device uses a 4k subsector size, that size and instruction must be included
    // somewhere in the 8th or 9th DWORD. This allows the user to discover the typical and maximum
    // erase times for the 4k subsector by referencing the 10th DWORD.
    for (Index = 0; Index < ARRAY_SIZE (ParamTable->Erase_Size_Command); Index++) {
      if (ParamTable->Erase_Size_Command[Index].Command == ParamTable->Erase4K_Command) {
        break;
      }
    }
    if (Index == ARRAY_SIZE (ParamTable->Erase_Size_Command)) {
      return EFI_NOT_FOUND;
    }
    SpiNorParams->EraseIndex = Index;
  } else {
    // Index 0 size would always be non zero
    for (Index = 1; Index < ARRAY_SIZE (ParamTable->Erase_Size_Command); Index++) {
      if (ParamTable->Erase_Size_Command[Index].Size != 0) {
        NeedParseSectorMapTable = TRUE;
        break;
      }
    }
    if (NeedParseSectorMapTable == FALSE) {
      SpiNorParams->EraseIndex = 0;
    } else {
      // Parse sector map table and find an erase size that can be used throughout the device
      Status = GetEraseCommandIndex (SpiIo, SpiNorParams, &Index);
      if (EFI_ERROR (Status)) {
        return Status;
      }
      SpiNorParams->EraseIndex = Index;
    }
  }

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
        Status = FillEraseRequestPacket (SpiIo, RequestPacket, SpiNorParams);
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
  UINTN                         TransferBytes;

  Status = EFI_SUCCESS;
  RequestPacket = SpiNorGetRequestPacket (SpiNorParams, SPI_NOR_REQUEST_TYPE_READ);

  while (Length) {
    TransferBytes = SpiIo->MaximumTransferBytes;
    if (SpiIo->Attributes & SPI_TRANSFER_SIZE_INCLUDES_ADDRESS) {
      TransferBytes -= sizeof (UINT32);
    }
    if (SpiIo->Attributes & SPI_TRANSFER_SIZE_INCLUDES_OPCODE) {
      TransferBytes -= sizeof (UINT8);
    }
    TransferBytes = MIN (Length, TransferBytes);
    // Fill Flash read request packet
    FillRequestPacketData (
      SPI_NOR_REQUEST_TYPE_READ,
      SpiNorParams,
      RequestPacket,
      From,
      ReadBuf,
      TransferBytes
      );
    // read Flash Data
    Status = SpiIo->Transaction (
                      SpiIo,
                      RequestPacket,
                      0
                      );
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Error Reading Flash Data %r\n", Status));
      break;
    }

    // Move Pointers
    ReadBuf += TransferBytes;
    From += TransferBytes;
    Length -= TransferBytes;
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
  FillRequestPacketData (
    SPI_NOR_REQUEST_TYPE_WRITE_ENABLE,
    SpiNorParams,
    RequestPacket,
    0,
    0,
    0
  );
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
  FillRequestPacketData (
    SPI_NOR_REQUEST_TYPE_WRITE_ENABLE,
    SpiNorParams,
    RequestPacket,
    0,
    0,
    0
  );
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
