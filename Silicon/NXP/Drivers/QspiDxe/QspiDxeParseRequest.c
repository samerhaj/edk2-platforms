/** @QspiDxeParseRequest.c

  Functions for parsing incoming SPI message request and constructting a LUT entry dynamically.

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the bsd
  license which accompanies this distribution. the full text of the license may
  be found at http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
**/

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>

#include "QspiDxe.h"

/**
  This function converts incoming SPI request to QSPI request
  Qspi Controller supports write only and write then read type of requests.
  Incoming SPI request packet is parsed and the type of request is determined.
  if the request is supported by QSPI controller, QSPI request data structure is
  filled with values needed to complete the request.

  Additionally the LUT table entries are also programmed, which will be required
  to complete the request. Which LUT entry is used, is specified by Request parameter.

  @param[in]   QMaster                QSPI_MASTER structure of a QSPI controller
  @param[in]   RequestPacket          Incoming SPI request packet
  @param[in, out]  Request            QSPI Request generated after parsing RequestPacket
                                      on Input this provides the LUT table entry to fill.

  @retval EFI_INVALID_PARAMETER       The parameters specified in RequestPacket are not
                                      Valid or the input parameters to function are null
  @retval EFI_UNSUPPORTED             The incoming request packet is not supported by QSPI
                                      controller. EFI_SPI_CONTROLLER_CAPABILITIES field of
                                      EFI_SPI_HC_PROTOCOL can be checked to debug this.
  @retval EFI_SUCCESS                 The Incoming RequestPacket is supported and specified
                                      Lut entry has been filled and QSPI Request structure is
                                      filled with values needed to complete request packet.
**/
EFI_STATUS
ParseRequest (
  IN      QSPI_MASTER             *QMaster,
  IN      EFI_SPI_REQUEST_PACKET  *RequestPacket,
  IN OUT  QSPI_REQUEST            *Request
  )
{
  UINTN                             Count;
  UINT16                            Lut[8];
  UINT8                             LutIndex;
  EFI_SPI_BUS_TRANSACTION           *SpiTransaction;
  UINT8                             Instruction;
  UINT8                             Pins;
  UINT8                             Operand;
  QSPI_REGISTERS                    *Regs;
  UINT32                            *LutBase;
  UINT32                            LutId;
  EFI_STATUS                        Status;
  EFI_SPI_HC_PROTOCOL               *SpiHc;
  INT64                             MaximumTransferBytes;
  UINT8                             ModeBits;

  if (!QMaster || !RequestPacket || !Request || !RequestPacket->TransactionCount) {
    return EFI_INVALID_PARAMETER;
  }

  if (RequestPacket->TransactionCount > ARRAY_SIZE (Lut)) {
    DEBUG ((
      DEBUG_ERROR,
      "Transaction count (%d) is greater than supported Transaction count(%d)\n",
      RequestPacket->TransactionCount,
      ARRAY_SIZE (Lut)
      ));
    return EFI_UNSUPPORTED;
  }

  Status = EFI_SUCCESS;
  Regs = QMaster->Regs;
  LutBase = Regs->Lut;
  SetMem(Lut, sizeof(Lut), 0);
  SpiHc = &QMaster->QspiHcProtocol;
  MaximumTransferBytes = SpiHc->MaximumTransferBytes;
  Request->Type = SPI_REQUEST_NONE;

  for (Count = 0, LutIndex = 0; Count < RequestPacket->TransactionCount; Count++) {
    SpiTransaction = &RequestPacket->Transaction[Count];
    if (!SpiTransaction) {
      DEBUG ((DEBUG_ERROR, "No SPI Transaction at Index %d\n",Count));
      Status = EFI_INVALID_PARAMETER;
      break;
    }

    // All SPI controllers must support single data bus width.
    Pins = 0;

    // Check if the Transaction is full duplex or not ?
    // if its full duplex, return unsupported, as QSPI controller only supports
    // half duplex operations, that too either write only or write then read operation
    if (SpiTransaction->WriteBuffer && SpiTransaction->ReadBuffer
      && (SpiHc->Attributes | SPI_HALF_DUPLEX))
    {
      DEBUG ((DEBUG_ERROR, "FULL duplex operation not supported\n"));
      Status = EFI_UNSUPPORTED;
      break;
    } else if (SpiTransaction->BusWidth != SPI_TRANSACTION_BUS_WIDTH_1) {
      switch (SpiTransaction->BusWidth) {
        case SPI_TRANSACTION_BUS_WIDTH_2:
          if (!(SpiHc->Attributes | SPI_SUPPORTS_2_BIT_DATA_BUS_WIDTH)) {
            Status = EFI_UNSUPPORTED;
          } else {
            Pins = 1;
          }
          break;

        case SPI_TRANSACTION_BUS_WIDTH_4:
          if (!(SpiHc->Attributes | SPI_SUPPORTS_4_BIT_DATA_BUS_WIDTH)) {
            Status = EFI_UNSUPPORTED;
          } else {
            Pins = 2;
          }
          break;

        default:
          Status = EFI_UNSUPPORTED;
          break;
      }
      if (EFI_ERROR (Status)) {
        DEBUG ((DEBUG_ERROR, "Transaction data width (%d) not supported\n", SpiTransaction->BusWidth));
        break;
      }
    }

    if (!(SpiHc->FrameSizeSupportMask & SPI_FRAME_MASK (SpiTransaction->FrameSize))) {
      Status = EFI_UNSUPPORTED;
      DEBUG ((DEBUG_ERROR, "Transaction FrameSize (%d) not supported\n", SpiTransaction->FrameSize));
      break;
    }

    switch (SpiTransaction->TransactionType) {
      case SPI_TRANSACTION_COMMAND:
        if (SpiTransaction->Length != sizeof (UINT8)) {
          DEBUG ((
            DEBUG_ERROR,
            "Command size (%d) greater than one byte not supported\n",
            SpiTransaction->Length
            ));
          Status = EFI_UNSUPPORTED;
        } else if (!SpiTransaction->WriteBuffer) {
          DEBUG ((DEBUG_ERROR, "No command found\n"));
          Status = EFI_INVALID_PARAMETER;
        } else if (Request->Type >= SPI_REQUEST_WRITE_THEN_READ) {
          DEBUG ((
            DEBUG_ERROR,
            "Request not supported!! "
            "Support only Write or Write then Read operations\n"
            ));
          Status = EFI_UNSUPPORTED;
        } else {
          Instruction = LUT_CMD;
          Operand = *(UINT8 *)SpiTransaction->WriteBuffer;
          Lut[LutIndex++] = Instruction << LUT_INSTR0_SHIFT | Pins << LUT_PAD0_SHIFT | Operand << LUT_OPRND0_SHIFT;
          Request->Type = SPI_REQUEST_WRITE_ONLY;
          MaximumTransferBytes -= sizeof (UINT8);
        }
        break;

      case SPI_TRANSACTION_ADDRESS:
        if (!SpiTransaction->Length || SpiTransaction->Length > sizeof (UINT32)) {
          DEBUG ((DEBUG_ERROR, "Unsupported Address bytes %d\n", SpiTransaction->Length));
          Status = EFI_UNSUPPORTED;
        } else if (!SpiTransaction->WriteBuffer) {
          DEBUG ((DEBUG_ERROR, "No Address found\n"));
          Status = EFI_INVALID_PARAMETER;
        } else if (Request->Type >= SPI_REQUEST_WRITE_THEN_READ) {
          DEBUG ((
            DEBUG_ERROR,
            "Request not supported!! "
            "Support only Write or Write then Read operations\n"
            ));
          Status = EFI_UNSUPPORTED;
        } else {
          Instruction = LUT_ADDR;
          Operand = SpiTransaction->Length << 3; // convert bytes to bits
          Lut[LutIndex++] = Instruction << LUT_INSTR0_SHIFT | Pins << LUT_PAD0_SHIFT | Operand << LUT_OPRND0_SHIFT;
          CopyMem (&Request->Address, SpiTransaction->WriteBuffer, SpiTransaction->Length);
          Request->Type = SPI_REQUEST_WRITE_ONLY;
          MaximumTransferBytes -= sizeof (UINT32);
        }
        break;

      case SPI_TRANSACTION_MODE:
        // QSPI controller supports 2 bit or 4 bit or 8 bit mode values.
        ModeBits = SpiTransaction->Length * SpiTransaction->BusWidth;
        if ( (ModeBits != 2) || (ModeBits != 4) || (ModeBits != 8)) {
          DEBUG ((DEBUG_ERROR, "Unsupported number of Mode Bits %d\n", ModeBits));
          Status = EFI_UNSUPPORTED;
        } else if (!SpiTransaction->WriteBuffer) {
          DEBUG ((DEBUG_ERROR, "No Mode bits found\n"));
          Status = EFI_INVALID_PARAMETER;
        } else if (Request->Type >= SPI_REQUEST_WRITE_THEN_READ) {
          DEBUG ((
            DEBUG_ERROR,
            "Request not supported!! "
            "Support only Write or Write then Read operations\n"
            ));
          Status = EFI_UNSUPPORTED;
        } else {
          ModeBits = SpiTransaction->Length * SpiTransaction->BusWidth;
          Instruction = (ModeBits == 2) ? LUT_MODE2 : ( (ModeBits == 4) ? LUT_MODE4 : LUT_MODE);
          // 2/4/8 bit mode value
          Operand = (*(UINT8 *)SpiTransaction->WriteBuffer) & (0xFF << ModeBits);
          Lut[LutIndex++] = Instruction << LUT_INSTR0_SHIFT | Pins << LUT_PAD0_SHIFT | Operand << LUT_OPRND0_SHIFT;
          Request->Type = SPI_REQUEST_WRITE_ONLY;
          MaximumTransferBytes -= sizeof (UINT8);
        }
        break;

      case SPI_TRANSACTION_DUMMY:
        if (!SpiTransaction->Length || SpiTransaction->Length > 64) {
          DEBUG ((DEBUG_ERROR, "Unsupported Dummy cycles %d\n", SpiTransaction->Length));
          Status = EFI_UNSUPPORTED;
        } else if (Request->Type >= SPI_REQUEST_WRITE_THEN_READ) {
          DEBUG ((
            DEBUG_ERROR,
            "Request not supported!! "
            "Support only Write or Write then Read operations\n"
            ));
          Status = EFI_UNSUPPORTED;
        } else {
          Instruction = LUT_DUMMY;
          Operand = SpiTransaction->Length;
          Lut[LutIndex++] = Instruction << LUT_INSTR0_SHIFT | Pins << LUT_PAD0_SHIFT | Operand << LUT_OPRND0_SHIFT;
          Request->Type = SPI_REQUEST_WRITE_ONLY;
        }
        break;

      case SPI_TRANSACTION_DATA:
        if (SpiTransaction->WriteBuffer) {
          if (Request->Type >= SPI_REQUEST_WRITE_THEN_READ) {
            DEBUG ((
              DEBUG_ERROR,
              "Request not supported!! "
              "Support only Write or Write then Read operations\n"
              ));
            Status = EFI_UNSUPPORTED;
          } else {
            Instruction = LUT_WRITE;
            Operand = SpiTransaction->Length;
            Lut[LutIndex++] = Instruction << LUT_INSTR0_SHIFT | Pins << LUT_PAD0_SHIFT | Operand << LUT_OPRND0_SHIFT;
            Request->Buffer = SpiTransaction->WriteBuffer;
            Request->Length = SpiTransaction->Length;
            Request->Type = SPI_REQUEST_WRITE_ONLY;
            MaximumTransferBytes -= SpiTransaction->Length;
          }
        } else if (SpiTransaction->ReadBuffer) {
          if ( (Request->Type >= SPI_REQUEST_INVALID) || (Request->Type <= SPI_REQUEST_NONE)) {
            DEBUG ((
              DEBUG_ERROR,
              "Request not supported!! "
              "Support only Write or Write then Read operations\n"
              ));
            Status = EFI_UNSUPPORTED;
          } else {
            Instruction = LUT_READ;
            Operand = SpiTransaction->Length;
            Lut[LutIndex++] = Instruction << LUT_INSTR0_SHIFT | Pins << LUT_PAD0_SHIFT | Operand << LUT_OPRND0_SHIFT;
            Request->Buffer = SpiTransaction->ReadBuffer;
            Request->Length = SpiTransaction->Length;
            Request->Type = SPI_REQUEST_WRITE_THEN_READ;
            MaximumTransferBytes -= SpiTransaction->Length;
          }
        } else {
          DEBUG ((DEBUG_ERROR, "No data found to transmit or receive\n"));
          Status = EFI_INVALID_PARAMETER;
        }
        break;

      default:
        DEBUG ((DEBUG_ERROR, "Unsupported Transaction type %d\n", SpiTransaction->TransactionType));
        Status = EFI_UNSUPPORTED;
        break;
    }

    if (EFI_ERROR (Status)) {
      break;
    }
  }

  if (Status == EFI_SUCCESS && MaximumTransferBytes < 0) {
    DEBUG ((DEBUG_ERROR, "Unsupported Transfer Bytes\n"));
    Status = EFI_UNSUPPORTED;
  }

  if (EFI_ERROR (Status)) {
    return Status;
  }

  /* Unlock The LUT */
  QMaster->Write32 ( (UINTN)&Regs->Lutkey, LUT_KEY);
  QMaster->Write32 ( (UINTN)&Regs->Lckcr, LCKCR_UNLOCK);

  LutId = Request->LutId * 4;
  QMaster->Write32 ( (UINTN)&LutBase[LutId], Lut[1] << LUT_OPRND1_SHIFT | Lut[0]);
  QMaster->Write32 ( (UINTN)&LutBase[LutId + 1], Lut[3] << LUT_OPRND1_SHIFT | Lut[2]);
  QMaster->Write32 ( (UINTN)&LutBase[LutId + 2], Lut[5] << LUT_OPRND1_SHIFT | Lut[4]);
  QMaster->Write32 ( (UINTN)&LutBase[LutId + 3], Lut[7] << LUT_OPRND1_SHIFT | Lut[6]);

  /* Lock The LUT */
  QMaster->Write32 ( (UINTN)&Regs->Lutkey, LUT_KEY);
  QMaster->Write32 ( (UINTN)&Regs->Lckcr, LCKCR_LOCK);

  return EFI_SUCCESS;
}
