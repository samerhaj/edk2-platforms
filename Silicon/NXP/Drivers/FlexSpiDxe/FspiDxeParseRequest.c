/** @FspiDxeParseRequest.c

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

#include "FspiDxe.h"

/**
  This function checks if the SPI transaction is supported by SPI host controller
  There is no NULL checking performed in this function, so caller should ensure that the input
  parameters to this function are valid parameters.

  @param[in]   Fspi           FSPI_MASTER structure of a FSPI controller
  @param[in]   SpiTransaction    Pointer of type EFI_SPI_BUS_TRANSACTION.
  @param[in]   Request           FSPI Request generated after parsing Transactions in RequestPacket
  @param[in, out] TransferBytes  Total number of bytes to be transferred between SPI host controller
                                 and SPI peripheral. it contains both bytes send from SPI host controller
         to SPI peripheral and received from SPI peripheral by SPI host controller

  @retval EFI_INVALID_PARAMETER       The parameters specified in Transaction are not Valid.
  @retval EFI_UNSUPPORTED             The Transaction is not supported by FSPI controller.
                                      EFI_SPI_CONTROLLER_CAPABILITIES field of
                                      EFI_SPI_HC_PROTOCOL can be checked to debug this.
  @retval EFI_SUCCESS                 The Transaction is supported.
**/
EFI_STATUS
CheckTransaction (
  IN      FSPI_MASTER                *Fspi,
  IN      EFI_SPI_BUS_TRANSACTION    *SpiTransaction,
  IN      FSPI_REQUEST               *Request,
  IN OUT  UINT64                     *TransferBytes
  )
{
  EFI_SPI_HC_PROTOCOL           *SpiHc;
  UINT8                         ModeBits;
  EFI_STATUS                    Status;

  SpiHc = &Fspi->FspiHcProtocol;
  Status = EFI_SUCCESS;

  // Check if the Transaction is full duplex or not ?
  // if its full duplex, return unsupported, as FSPI controller only supports
  // half duplex operations, that too either write only or write then read operation
  if (SpiTransaction->WriteBuffer && SpiTransaction->ReadBuffer
    && (SpiHc->Attributes | SPI_HALF_DUPLEX))
  {
    DEBUG ((DEBUG_ERROR, "FULL duplex operation not supported\n"));
    return EFI_UNSUPPORTED;
  }

  if (!(SpiHc->FrameSizeSupportMask & SPI_FRAME_MASK (SpiTransaction->FrameSize))) {
    DEBUG ((DEBUG_ERROR, "Transaction FrameSize (%d) not supported\n", SpiTransaction->FrameSize));
    return EFI_UNSUPPORTED;
  }

  switch (SpiTransaction->TransactionType) {
    case SPI_TRANSACTION_COMMAND:
      if (SpiTransaction->Length != sizeof (UINT8)) {
        DEBUG ((DEBUG_ERROR, "Command size (%d) greater than one byte not supported\n", SpiTransaction->Length));
        Status = EFI_UNSUPPORTED;
      } else if (!SpiTransaction->WriteBuffer) {
        DEBUG ((DEBUG_ERROR, "No command found\n"));
        Status = EFI_INVALID_PARAMETER;
      } else if (Request->Type >= SPI_REQUEST_WRITE_THEN_READ) {
        DEBUG ((DEBUG_ERROR, "Request not supported! Support only Write or Write then Read operations\n"));
        Status = EFI_UNSUPPORTED;
      }
      *TransferBytes += sizeof (UINT8);
      break;

    case SPI_TRANSACTION_ADDRESS:
      if (!SpiTransaction->Length || SpiTransaction->Length > sizeof (UINT32)) {
        DEBUG ((DEBUG_ERROR, "Unsupported Address bytes %d\n", SpiTransaction->Length));
        Status = EFI_UNSUPPORTED;
      } else if (!SpiTransaction->WriteBuffer) {
        DEBUG ((DEBUG_ERROR, "No Address found\n"));
        Status = EFI_INVALID_PARAMETER;
      } else if (Request->Type >= SPI_REQUEST_WRITE_THEN_READ) {
        DEBUG ((DEBUG_ERROR, "Request not supported! Support only Write or Write then Read operations\n"));
        Status = EFI_UNSUPPORTED;
      }
      *TransferBytes += sizeof (UINT32);
      break;

    case SPI_TRANSACTION_MODE:
      // FSPI controller supports 2 bit or 4 bit or 8 bit mode values.
      ModeBits = SpiTransaction->Length * SpiTransaction->BusWidth;
      if ( (ModeBits != 2) || (ModeBits != 4) || (ModeBits != 8)) {
        DEBUG ((DEBUG_ERROR, "Unsupported number of Mode Bits %d\n", ModeBits));
        Status = EFI_UNSUPPORTED;
      } else if (!SpiTransaction->WriteBuffer) {
        DEBUG ((DEBUG_ERROR, "No Mode bits found\n"));
        Status = EFI_INVALID_PARAMETER;
      } else if (Request->Type >= SPI_REQUEST_WRITE_THEN_READ) {
        DEBUG ((DEBUG_ERROR, "Request not supported! Support only Write or Write then Read operations\n"));
        Status = EFI_UNSUPPORTED;
      }
      *TransferBytes += sizeof (UINT8);
      break;

    case SPI_TRANSACTION_DUMMY:
      if (!SpiTransaction->Length || SpiTransaction->Length > 64) {
        DEBUG ((DEBUG_ERROR, "Unsupported Dummy cycles %d\n", SpiTransaction->Length));
        Status = EFI_UNSUPPORTED;
      } else if (Request->Type >= SPI_REQUEST_WRITE_THEN_READ) {
        DEBUG ((DEBUG_ERROR, "Request not supported! Support only Write or Write then Read operations\n"));
        Status = EFI_UNSUPPORTED;
      }
      break;

    case SPI_TRANSACTION_DATA:
      if (SpiTransaction->WriteBuffer) {
        if (Request->Type >= SPI_REQUEST_WRITE_THEN_READ) {
          DEBUG ((DEBUG_ERROR, "Request not supported! Support only Write or Write then Read operations\n"));
          Status = EFI_UNSUPPORTED;
        }
        *TransferBytes += SpiTransaction->Length;
      } else if (SpiTransaction->ReadBuffer) {
          if ( (Request->Type >= SPI_REQUEST_INVALID) || (Request->Type <= SPI_REQUEST_NONE)) {
            DEBUG ((DEBUG_ERROR, "Request not supported! Support only Write or Write then Read operations\n"));
            Status = EFI_UNSUPPORTED;
          }
        *TransferBytes += SpiTransaction->Length;
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

  if (*TransferBytes > SpiHc->MaximumTransferBytes) {
    DEBUG ((
      DEBUG_ERROR, "Transfer Bytes %d exceed MaximumTransferBytes %d\n",
      *TransferBytes, SpiHc->MaximumTransferBytes
    ));
    Status = EFI_UNSUPPORTED;
  }

  return Status;
}

/**
  This function converts incoming SPI request to FSPI request
  Fspi Controller supports write only and write then read type of requests.
  Incoming SPI request packet is parsed and the type of request is determined.
  if the request is supported by FSPI controller, FSPI request data structure is
  filled with values needed to complete the request.

  Additionally the LUT table entries are also programmed, which will be required
  to complete the request. Which LUT entry is used, is specified by Request parameter.

  @param[in]   Fspi                FSPI_MASTER structure of a FSPI controller
  @param[in]   RequestPacket          Incoming SPI request packet
  @param[in, out]  Request            FSPI Request generated after parsing RequestPacket
                                      on Input this provides the LUT table entry to fill.

  @retval EFI_INVALID_PARAMETER       The parameters specified in RequestPacket are not
                                      Valid or the input parameters to function are null
  @retval EFI_UNSUPPORTED             The incoming request packet is not supported by FSPI
                                      controller. EFI_SPI_CONTROLLER_CAPABILITIES field of
                                      EFI_SPI_HC_PROTOCOL can be checked to debug this.
  @retval EFI_SUCCESS                 The Incoming RequestPacket is supported and specified
                                      Lut entry has been filled and FSPI Request structure is
                                      filled with values needed to complete request packet.
**/
EFI_STATUS
ParseRequest (
  IN      FSPI_MASTER             *Fspi,
  IN      EFI_SPI_REQUEST_PACKET  *RequestPacket,
  IN OUT  FSPI_REQUEST            *Request
  )
{
  UINTN                             Count;
  UINT16                            Lut[8];
  UINT8                             LutIndex;
  EFI_SPI_BUS_TRANSACTION           *SpiTransaction;
  UINT8                             Instruction;
  UINT8                             Pins;
  UINT8                             Operand;
  FSPI_REGISTERS                    *Regs;
  UINT32                            *LutBase;
  UINT32                            LutId;
  EFI_SPI_HC_PROTOCOL               *SpiHc;
  EFI_STATUS                        Status;
  UINT64                            TransferBytes;
  UINT8                             ModeBits;

  if (!Fspi || !RequestPacket || !Request || !RequestPacket->TransactionCount) {
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
  SpiHc = &Fspi->FspiHcProtocol;
  Regs = Fspi->Regs;
  LutBase = Regs->Lut;
  TransferBytes = 0;
  Request->Type = SPI_REQUEST_NONE;
  Pins = 0; // All SPI controllers must support single data bus width.
  ZeroMem (Lut, sizeof (Lut));

  for (Count = 0, LutIndex = 0; Count < RequestPacket->TransactionCount; Count++) {
    SpiTransaction = &RequestPacket->Transaction[Count];
    if (!SpiTransaction) {
      DEBUG ((DEBUG_ERROR, "No SPI Transaction at Index %d\n",Count));
      Status = EFI_INVALID_PARAMETER;
      break;
    }

    Status = CheckTransaction (Fspi, SpiTransaction, Request, &TransferBytes);
    if (EFI_ERROR (Status)) {
      break;
    }

    switch (SpiTransaction->BusWidth) {
      case SPI_TRANSACTION_BUS_WIDTH_1:
        Pins = 0; // All SPI controllers must support single data bus width.
        break;

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

      case SPI_TRANSACTION_BUS_WIDTH_8:
        if (!(SpiHc->Attributes | SPI_SUPPORTS_8_BIT_DATA_BUS_WIDTH)) {
          Status = EFI_UNSUPPORTED;
        } else {
          Pins = 3;
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

    switch (SpiTransaction->TransactionType) {
      case SPI_TRANSACTION_COMMAND:
        Instruction = LUT_CMD;
        Operand = *(UINT8 *)SpiTransaction->WriteBuffer;
        Lut[LutIndex++] = Instruction << LUT_INSTR0_SHIFT | Pins << LUT_PAD0_SHIFT | Operand << LUT_OPRND0_SHIFT;
        Request->Type = SPI_REQUEST_WRITE_ONLY;
        break;

      case SPI_TRANSACTION_ADDRESS:
        Instruction = LUT_ADDR;
        Operand = SpiTransaction->Length << 3; // convert bytes to bits
        Lut[LutIndex++] = Instruction << LUT_INSTR0_SHIFT | Pins << LUT_PAD0_SHIFT | Operand << LUT_OPRND0_SHIFT;
        CopyMem (&Request->Address, SpiTransaction->WriteBuffer, SpiTransaction->Length);
        Request->Type = SPI_REQUEST_WRITE_ONLY;
        break;

      case SPI_TRANSACTION_MODE:
        // FSPI controller supports 2 bit or 4 bit or 8 bit mode values.
        ModeBits = SpiTransaction->Length * SpiTransaction->BusWidth;
        Instruction = (ModeBits == 2) ? LUT_MODE2 : ( (ModeBits == 4) ? LUT_MODE4 : LUT_MODE);
        // 2/4/8 bit mode value
        Operand = (*(UINT8 *)SpiTransaction->WriteBuffer) & (0xFF << ModeBits);
        Lut[LutIndex++] = Instruction << LUT_INSTR0_SHIFT | Pins << LUT_PAD0_SHIFT | Operand << LUT_OPRND0_SHIFT;
        Request->Type = SPI_REQUEST_WRITE_ONLY;
        break;

      case SPI_TRANSACTION_DUMMY:
        Instruction = LUT_DUMMY;
        Operand = SpiTransaction->Length;
        Lut[LutIndex++] = Instruction << LUT_INSTR0_SHIFT | Pins << LUT_PAD0_SHIFT | Operand << LUT_OPRND0_SHIFT;
        Request->Type = SPI_REQUEST_WRITE_ONLY;
        break;

      case SPI_TRANSACTION_DATA:
        if (SpiTransaction->WriteBuffer) {
          Instruction = LUT_WRITE;
          Operand = SpiTransaction->Length;
          Lut[LutIndex++] = Instruction << LUT_INSTR0_SHIFT | Pins << LUT_PAD0_SHIFT | Operand << LUT_OPRND0_SHIFT;
          Request->Buffer = SpiTransaction->WriteBuffer;
          Request->Length = SpiTransaction->Length;
          Request->Type = SPI_REQUEST_WRITE_ONLY;
        } else if (SpiTransaction->ReadBuffer) {
          Instruction = LUT_READ;
          Operand = SpiTransaction->Length;
          Lut[LutIndex++] = Instruction << LUT_INSTR0_SHIFT | Pins << LUT_PAD0_SHIFT | Operand << LUT_OPRND0_SHIFT;
          Request->Buffer = SpiTransaction->ReadBuffer;
          Request->Length = SpiTransaction->Length;
          Request->Type = SPI_REQUEST_WRITE_THEN_READ;
        }
        break;

      default:
        Status = EFI_UNSUPPORTED;
        break;
    }
  }

  if (EFI_ERROR (Status)) {
    return Status;
  }

  if (LutIndex < ARRAY_SIZE (Lut)) {
    // Stop execution; deassert CS
    Lut[LutIndex++] = 0x00;
  }

  /* Unlock The LUT */
  Fspi->Write32 ( (UINTN)&Regs->LutKey, LUT_KEY);
  Fspi->Write32 ( (UINTN)&Regs->LutCr, LUTCR_UNLOCK);

  LutId = Request->LutId * 4;
  Fspi->Write32 ( (UINTN)&LutBase[LutId], Lut[1] << LUT_OPRND1_SHIFT | Lut[0]);
  Fspi->Write32 ( (UINTN)&LutBase[LutId + 1], Lut[3] << LUT_OPRND1_SHIFT | Lut[2]);
  Fspi->Write32 ( (UINTN)&LutBase[LutId + 2], Lut[5] << LUT_OPRND1_SHIFT | Lut[4]);
  Fspi->Write32 ( (UINTN)&LutBase[LutId + 3], Lut[7] << LUT_OPRND1_SHIFT | Lut[6]);

  /* Lock The LUT */
  Fspi->Write32 ( (UINTN)&Regs->LutKey, LUT_KEY);
  Fspi->Write32 ( (UINTN)&Regs->LutCr, LUTCR_LOCK);

  return EFI_SUCCESS;
}
