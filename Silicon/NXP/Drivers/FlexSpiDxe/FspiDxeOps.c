/** @FspiFlashOps.c

  Functions for implementing fspi controller functionality.

  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the bsd
  license which accompanies this distribution. the full text of the license may
  be found at http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
**/
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>

#include "FspiDxe.h"

/**
 Dump Fspi Registers for debugging

 @param[in]  Fspi           Pointer to FSPI_MASTER strcture of a FSPI controller
**/
VOID
DumpRegs (
  IN  FSPI_MASTER  *Fspi
  )
{
  UINT32            Index;
  FSPI_REGISTERS    *Regs;

  Regs = Fspi->Regs;

  DEBUG ((DEBUG_ERROR, "Mcr0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->Mcr0)));
  DEBUG ((DEBUG_ERROR, "Mcr1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->Mcr1)));
  DEBUG ((DEBUG_ERROR, "Mcr2  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->Mcr2)));
  DEBUG ((DEBUG_ERROR, "AhbCr  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbCr)));
  DEBUG ((DEBUG_ERROR, "IntEn  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->IntEn)));
  DEBUG ((DEBUG_ERROR, "Intr  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->Intr)));
  DEBUG ((DEBUG_ERROR, "LutKey  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->LutKey)));
  DEBUG ((DEBUG_ERROR, "LutCr  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->LutCr)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf0Cr0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf0Cr0)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf1Cr0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf1Cr0)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf2Cr0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf2Cr0)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf3Cr0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf3Cr0)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf4Cr0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf4Cr0)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf5Cr0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf5Cr0)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf6Cr0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf6Cr0)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf7Cr0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf7Cr0)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf0Cr1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf0Cr1)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf1Cr1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf1Cr1)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf2Cr1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf2Cr1)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf3Cr1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf3Cr1)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf4Cr1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf4Cr1)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf5Cr1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf5Cr1)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf6Cr1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf6Cr1)));
  DEBUG ((DEBUG_ERROR, "AhbRxBuf7Cr1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbRxBuf7Cr1)));
  DEBUG ((DEBUG_ERROR, "FlshA1Cr0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshA1Cr0)));
  DEBUG ((DEBUG_ERROR, "FlshA2Cr0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshA2Cr0)));
  DEBUG ((DEBUG_ERROR, "FlshB1Cr0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshB1Cr0)));
  DEBUG ((DEBUG_ERROR, "FlshB2Cr0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshB2Cr0)));
  DEBUG ((DEBUG_ERROR, "FlshA1Cr1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshA1Cr1)));
  DEBUG ((DEBUG_ERROR, "FlshA2Cr1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshA2Cr1)));
  DEBUG ((DEBUG_ERROR, "FlshB1Cr1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshB1Cr1)));
  DEBUG ((DEBUG_ERROR, "FlshB2Cr1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshB2Cr1)));
  DEBUG ((DEBUG_ERROR, "FlshA1Cr2  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshA1Cr2)));
  DEBUG ((DEBUG_ERROR, "FlshA2Cr2  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshA2Cr2)));
  DEBUG ((DEBUG_ERROR, "FlshB1Cr2  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshB1Cr2)));
  DEBUG ((DEBUG_ERROR, "FlshB2Cr2  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshB2Cr2)));
  DEBUG ((DEBUG_ERROR, "FlshCr3  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshCr3)));
  DEBUG ((DEBUG_ERROR, "FlshCr4  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshCr4)));
  DEBUG ((DEBUG_ERROR, "FlshCr5  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshCr5)));
  DEBUG ((DEBUG_ERROR, "FlshCr6  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->FlshCr6)));
  DEBUG ((DEBUG_ERROR, "IpCr0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->IpCr0)));
  DEBUG ((DEBUG_ERROR, "IpCr1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->IpCr1)));
  DEBUG ((DEBUG_ERROR, "IpCr2  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->IpCr2)));
  DEBUG ((DEBUG_ERROR, "IpCr3  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->IpCr3)));
  DEBUG ((DEBUG_ERROR, "IpCmd  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->IpCmd)));
  DEBUG ((DEBUG_ERROR, "Dlpr  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->Dlpr)));
  DEBUG ((DEBUG_ERROR, "IpRxfCr  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->IpRxfCr)));
  DEBUG ((DEBUG_ERROR, "IpTxfCr  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->IpTxfCr)));
  DEBUG ((DEBUG_ERROR, "DllaCr  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->DllaCr)));
  DEBUG ((DEBUG_ERROR, "DllbCr  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->DllbCr)));
  DEBUG ((DEBUG_ERROR, "MiscCr2  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->MiscCr2)));
  DEBUG ((DEBUG_ERROR, "MiscCr3  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->MiscCr3)));
  DEBUG ((DEBUG_ERROR, "MiscCr4  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->MiscCr4)));
  DEBUG ((DEBUG_ERROR, "MiscCr5  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->MiscCr5)));
  DEBUG ((DEBUG_ERROR, "MiscCr6  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->MiscCr6)));
  DEBUG ((DEBUG_ERROR, "MiscCr7  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->MiscCr7)));
  DEBUG ((DEBUG_ERROR, "Sts0  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->Sts0)));
  DEBUG ((DEBUG_ERROR, "Sts1  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->Sts1)));
  DEBUG ((DEBUG_ERROR, "Sts2  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->Sts2)));
  DEBUG ((DEBUG_ERROR, "AhbSpndSts  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->AhbSpndSts)));
  DEBUG ((DEBUG_ERROR, "IpRxfSts  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->IpRxfSts)));
  DEBUG ((DEBUG_ERROR, "IpTxfSts  :0x%08x \n", Fspi->Read32 ( (UINTN)&Regs->IpTxfSts)));
  for (Index = 0; Index < ARRAY_SIZE (Regs->Rfdr); Index++) {
    DEBUG ((DEBUG_ERROR, "Rfdr[%d]  :0x%08x \n", Index, Fspi->Read32 ( (UINTN)&Regs->Rfdr[Index])));
  }
  for (Index = 0; Index < ARRAY_SIZE (Regs->Tfdr); Index++) {
    DEBUG ((DEBUG_ERROR, "Tfdr[%d]  :0x%08x \n", Index, Fspi->Read32 ( (UINTN)&Regs->Tfdr[Index])));
  }
  for (Index = 0; Index < ARRAY_SIZE (Regs->Lut); Index++) {
    DEBUG ((DEBUG_ERROR, "Lut[%d]  :0x%08x \n", Index, Fspi->Read32 ( (UINTN)&Regs->Lut[Index])));
  }

  return;
}

/**
 Check Flag register if any error bit is set or not while
 executing IP mode instruction.

 @param[in]  Fspi           Pointer to FSPI_MASTER strcture of a FSPI controller

 @retval    EFI_DEVICE_ERROR   if an error bit is set
 @retval    EFI_SUCCESS        No error bit is set
**/
STATIC
EFI_STATUS
CheckFlagRegister (
  IN  FSPI_MASTER  *Fspi
  )
{
  UINT32   IntrReg;

  IntrReg = Fspi->Read32 ((UINTN)&Fspi->Regs->Intr);
  if (IntrReg & FLEXSPI_ERR_MASK) {
    DEBUG ((DEBUG_ERROR, "IntrReg=0x%08x\n", IntrReg));
    if (IntrReg & (CMD_GRANT_ERR_MASK | AHB_TOUT_ERR_MASK)) {
      return EFI_TIMEOUT;
    }
    if (IntrReg & CMD_CHECK_ERR_MASK) {
      return EFI_INVALID_PARAMETER;
    }
    return EFI_DEVICE_ERROR;
  } else if (IntrReg & (INTR_SCKSTOPBYWR | INTR_SCKSTOPBYRD)) {
    DEBUG ((DEBUG_WARN, "SCK is stopped during command sequence IntrReg=0x%08x\n", IntrReg));
  }
  return EFI_SUCCESS;
}

/**
 Check Valid entries in Rx Buffer.
 once we start receieving data from SPI device, we first check the valid data received
 in Rx buffer and then read only that much data.

 @param[in]  Fspi    Pointer to FSPI_MASTER strcture of a FSPI controller

 @retval                Number of Valid entries in Rx Buffer.
**/
STATIC inline
UINT8
IpRxValidEntries (
  IN   FSPI_MASTER           *Fspi
  )
{
  FSPI_REGISTERS *Regs;
  Regs = Fspi->Regs;
  return (Fspi->Read32 ( (UINTN)&Regs->IpRxfSts) & IPRXFSTS_FILL_MASK) >> IPRXFSTS_FILL_SHIFT;
}

/**
  This function performs the WRITE_ONLY operation on SPI device.

  The FlexSPI module has only one method for writing to the external device:
  IP commands.
  This method has its own internal buffers.

  Before calling this function ensure that a correct write sequence in programmed in the LUT in
  accordance with the device connected on board. and the valid address, number of data bytes,
  and buffer to write data from is provided to this function. The LUT entry to use, should also
  be provided to this function.

  First the data to be written is copied from buffer provided to internal buffer.
  Then, as programmed in LUT entry, first some control information is sent to SPI device
  (like command, address etc) and then the device is able to receive data which is
  written into internal buffers.

  @param[in]   Fspi          FSPI_MASTER structure of a FSPI controller
  @param[in]   Request          FSPI Request structure which provides the LUT entry to use,
                                Address to write to, number of data bytes to write and buffer
                                from which data is to be written.

  @retval EFI_ALREADY_STARTED   The controller is busy with another transaction.
  @retval EFI_DEVICE_ERROR      There was an SPI error during the transaction.
  @retval EFI_SUCCESS           The transaction completed successfully.
**/
EFI_STATUS
WriteTransaction (
  IN  FSPI_MASTER    *Fspi,
  IN  FSPI_REQUEST   *Request
  )
{
  FSPI_REGISTERS      *Regs;
  UINT32              Index;
  UINT32              TxSize;
  UINT32              To;
  UINT32              Length;
  UINT32              *Txbuf;
  UINT32              Data;
  UINT8               TxWmarkSize; // Tx Water Mark size in 32 bit entries
  EFI_STATUS          Status;

  Regs = Fspi->Regs;
  Status = EFI_SUCCESS;
  Txbuf = (UINT32 *)Request->Buffer;
  TxWmarkSize = ((TX_WMRK + 1) << 1);
  Length = Request->Length;

  // Wait while controller is busy
  while (!(Fspi->Read32 ((UINTN)&Regs->Sts0) & STS0_ARBIDLE));

  // Clear all valid data entries in IP TX FIFO
  // Set TX Water Mark
  Fspi->Write32 (
          (UINTN)&Regs->IpTxfCr,
          ((TX_WMRK & IPTXFCR_TXWMRK_MASK) << IPTXFCR_TXWMRK_SHIFT) | IPTXFCR_CLRIPTXF
          );
  // Fill TX FIFO
  TxSize = (Length > TX_IPBUF_SIZE) ? TX_IPBUF_SIZE : Length;
  Length -= TxSize;
  while (TxSize) {
    // Processor needs to poll register INTR[IPTXWE] before filling IP TX FIFO.
    // This is to make sure there is enough space for a watermark
    // level Data filling before filling.
    while (!(Fspi->Read32 ( (UINTN)&Regs->Intr) & INTR_IPTXWE));
    // Processor need to fill a watermark level datas to IP TX FIFO each time.
    Index = 0;
    while (Index < TxWmarkSize) {
      if (TxSize >= sizeof (Data)) {
        Fspi->Write32 ( (UINTN)&Regs->Tfdr[Index++], *Txbuf++);
        TxSize -= sizeof (UINT32);
      } else if (TxSize && TxSize < sizeof (Data)) {
        Data = 0;
        CopyMem (&Data, Txbuf, TxSize);
        Fspi->Write32 ( (UINTN)&Regs->Tfdr[Index++], Data);
        Txbuf = (UINT32 *)((UINT8 *)Txbuf + TxSize);
        TxSize = 0;
      } else {
        Data = 0;
        Fspi->Write32 ( (UINTN)&Regs->Tfdr[Index++], Data);
      }
    }
    // After filling a watermark level datas to IP TX FIFO,
    // need to set register bit INTR[IPTXWE].
    // This will push a watermark level datas into IP TX FIFO
    // (write pointer is incremented).
    // NOTE: IP TX FIFO data is not pushed by each write access,
    // only pushed by set INTR[IPTXWE] bit.
    Fspi->SetBits32 ( (UINTN)&Regs->Intr, INTR_IPTXWE);
  }

  TxSize = Length;

  // Flash access start address
  To = Request->Address + Fspi->CurCSOffset;
  Fspi->Write32 ( (UINTN)&Regs->IpCr0, To);

  // Clear all flags before triggering IP command.
  Fspi->SetBits32 ((UINTN)&Regs->Intr, FLEXSPI_ERR_MASK | INTR_IPRXWA | INTR_SCKSTOPBYWR);
  // Load IP command Sequnce id and number of sequences to run
  Fspi->Write32 (
          (UINTN)&Regs->IpCr1,
          (Request->LutId << IPCR1_ISEQID_SHIFT) | Request->Length
          );
  // Trigger the command
  Fspi->Write32 ( (UINTN)&Regs->IpCmd, IPCMD_TRG);
  while (TxSize) {
    // Processor needs to poll register INTR[IPTXWE] before filling IP TX FIFO.
    // This is to make sure there is enough space for a watermark
    // level Data filling before filling.
    while (!(Fspi->Read32 ( (UINTN)&Regs->Intr) & INTR_IPTXWE));
    // Processor need to fill a watermark level datas to IP TX FIFO each time.
    Index = 0;
    while (Index < TxWmarkSize) {
      if (TxSize >= sizeof (Data)) {
        Fspi->Write32 ( (UINTN)&Regs->Tfdr[Index++], *Txbuf++);
        TxSize -= sizeof (UINT32);
      } else if (TxSize && TxSize < sizeof (Data)) {
        Data = 0;
        CopyMem (&Data, Txbuf, TxSize);
        Fspi->Write32 ( (UINTN)&Regs->Tfdr[Index++], Data);
        Txbuf = (UINT32 *)((UINT8 *)Txbuf + TxSize);
        TxSize = 0;
      } else {
        Data = 0;
        Fspi->Write32 ( (UINTN)&Regs->Tfdr[Index++], Data);
      }
    }
    // After filling a watermark level datas to IP TX FIFO,
    // need to set register bit INTR[IPTXWE].
    // This will push a watermark level datas into IP TX FIFO
    // (write pointer is incremented).
    // NOTE: IP TX FIFO data is not pushed by each write access,
    // only pushed by set INTR[IPTXWE] bit.
    Fspi->SetBits32 ( (UINTN)&Regs->Intr, INTR_IPTXWE);
  }
  // poll register bit INTR_IP_CMD_DONE
  while (!(Fspi->Read32 ( (UINTN)&Regs->Intr) & INTR_IPCMDDONE)) {
    Status = CheckFlagRegister (Fspi);
    if (EFI_ERROR (Status)) {
      break;
    }
  }

  // Invalidate the AHB buffer contents using software reset
  // SW reset by writing 1 to MCR0[SWRESET]. This bit is auto-cleared by hardware after
  // software done. SW reset lasts about 64 cycles of serial root clock plus 2 cycles of ipg_clk
  // clock. Configurtion registers will not be reset. Software should poll this bit to wait for SW
  // reset done.
  Fspi->SetBits32 ( (UINTN)&Regs->Mcr0, MCR0_SWRESET);
  while (Fspi->Read32 ( (UINTN)&Regs->Mcr0) & MCR0_SWRESET) {
    MicroSecondDelay (1);
  }

  return Status;
}

/**
  This function performs the WRITE_THEN_READ operation on SPI device.

  The FlexSPI module has two different methods for reading from the external device:
  IP commands and AHB commands.
  Each of these methods has its own internal buffers.

  Before calling this function ensure that a correct read sequence in programmed in the LUT in
  accordance with the device connected on board. and the valid address, number of data bytes,
  and buffer to read data into is provided to this function. The LUT entry to use, should also
  be provided to this function.

  As programmed in LUT entry, first some control information is sent to SPI device
  (like command, address etc) and then the device output data which is read into internal
  buffers. Hence the operation is WRITE_THEN_READ. Then the data is copied from internal
  buffers to buffer provided in FSPI Request structure.

  @param[in]   Fspi          FSPI_MASTER structure of a FSPI controller
  @param[in]   Request          FSPI Request structure which provides the LUT entry to use,
                                Address to read from, number of data bytes to read and buffer
                                in which data is to be read.

  @retval EFI_ALREADY_STARTED   The controller is busy with another transaction.
  @retval EFI_DEVICE_ERROR      There was an SPI error during the transaction.
  @retval EFI_SUCCESS           The transaction completed successfully.
**/
EFI_STATUS
ReadTransaction (
  IN   FSPI_MASTER     *Fspi,
  IN   FSPI_REQUEST    *Request
  )
{
  FSPI_REGISTERS        *Regs;
  UINT32                Data;
  UINT16                Size;
  UINT32                Index;
  UINT32                *Rxbuf;
  UINT32                From;
  EFI_STATUS            Status;
  UINT8                 RxWmarkSize; // Rx Water Mark size in 32 bit entries
  UINT8                 RxValidEntries; // Valid 64 bit entries in RX FIFO

  Data = 0;
  Status = EFI_SUCCESS;
  Regs = Fspi->Regs;
  Rxbuf = (UINT32 *)Request->Buffer;
  From = Request->Address + Fspi->CurCSOffset;
  RxWmarkSize = ((RX_WMRK + 1) << 1);

  // Wait while controller is busy
  while (!(Fspi->Read32 ((UINTN)&Regs->Sts0) & STS0_ARBIDLE));

  // Clear all valid data entries in IP RX FIFO
  // Set RX Water Mark
  Fspi->Write32 (
          (UINTN)&Regs->IpRxfCr,
          ((TX_WMRK & IPRXFCR_RXWMRK_MASK) << IPRXFCR_RXWMRK_SHIFT) | IPRXFCR_CLRIPRXF
          );

  while (Request->Length > 0) {
    // Flash access start address
    Fspi->Write32 ( (UINTN)&Regs->IpCr0, From);

    Size = (Request->Length > RX_IPBUF_SIZE) ? RX_IPBUF_SIZE : Request->Length;

    // Clear all flags before triggering IP command.
    Fspi->SetBits32 ((UINTN)&Regs->Intr, FLEXSPI_ERR_MASK | INTR_IPRXWA | INTR_SCKSTOPBYRD);
    // Load IP command Sequnce id and number of sequences to run
    Fspi->Write32 (
               (UINTN)&Regs->IpCr1,
               (Request->LutId << IPCR1_ISEQID_SHIFT) | Size
               );
    // Trigger the command
    Fspi->Write32 ( (UINTN)&Regs->IpCmd, IPCMD_TRG);
    // poll register bit INTR_IP_CMD_DONE
    while (!(Fspi->Read32 ( (UINTN)&Regs->Intr) & INTR_IPCMDDONE)) {
      Status = CheckFlagRegister (Fspi);
      if (EFI_ERROR (Status)) {
        break;
      }
    }
    if (EFI_ERROR (Status)) {
      break;
    }

    // Move offsets
    From += Size;
    Request->Length -= Size;

    // FlexSPI push read data into IP RX FIFO in terms of 64 bits every time it receives 64 bits data
    // from external device. When read data bits number is not 64 bits aligned, FlexSPI will push
    // addintion zero bits into IP RX FIFO for the last push.
    // Now Read RX Fifo using IP Bus
    while (Size) {
      if (Size >= (RxWmarkSize << 2)) {
        // Processor needs to poll register INTR[IPRXWA]
        // before reading IP RX FIFO. This is to make sure there is a watermark
        // level Data filled in IP RX FIFO before reading.
        while (!(Fspi->Read32 ( (UINTN)&Regs->Intr) & INTR_IPRXWA));
        // Processor need to read out a watermark level datas from IP RX FIFO each time
        // before set register bit INTR[IPRXWA].
        Index = 0;
        while (Index < RxWmarkSize) {
          *Rxbuf++ = Fspi->Read32 ( (UINTN)&Regs->Rfdr[Index++]);
          Size -= sizeof (UINT32);
        }
        // After reading a watermark level datas from IP RX FIFO, software need to set register bit
        // INTR[IPRXWA]. This set action will pop out a watermark level datas from IP RX FIFO.
        Fspi->SetBits32 ( (UINTN)&Regs->Intr, INTR_IPRXWA);
      } else {
        // It's supported that the total flash read/program data size is not multiple of
        // watermark level. In this case, the reading data size from IP RX FIFO will be less
        // than a watermark level for the last time, software should poll IPRXSTS[FILL] field
        // instead of polling INTR[IPRXWA].
        // Valid Data entries in IP RX FIFO is: FILL * 64 Bits.
        do {
          RxValidEntries = IpRxValidEntries (Fspi);
        } while (Size > (RxValidEntries << 3));
        Index = 0;
        while (Size && (Index < (RxValidEntries << 1))) {
          if (Size < sizeof (Data)) {
            Data = Fspi->Read32 ( (UINTN)&Regs->Rfdr[Index++]);
            CopyMem (Rxbuf, &Data, Size);
            Rxbuf = (UINT32 *)((UINT8 *)Rxbuf + Size);
            Size = 0;
          } else {
            *Rxbuf++ = Fspi->Read32 ( (UINTN)&Regs->Rfdr[Index++]);
            Size -= sizeof (UINT32);
          }
        }
      }
    }

    // Clear all valid data entries in IP RX FIFO
    Fspi->SetBits32 ( (UINTN)&Regs->IpRxfCr, IPRXFCR_CLRIPRXF);
  }

  return Status;
}

/**
 Configure The Fspi controller at startup
 FlexSPI controller iniitialization sequence is as following:
  - Enable controller clocks (AHB clock/IP Bus clock/Serial root clock) in System level.
  - Set MCR0[MDIS] to 0x1 (Make sure controller is configured in module stop mode)
  - Configure module control registers: MCR0, MCR1, MCR2. (Don't change MCR0[MDIS])
  - Configure AHB bus control register (AHBCR) and AHB RX Buffer control registers
    (AHBRXBUFxCR) optionally if AHB command will be used
  - Configure Flash control registers (FLSHxCR0,FLSHxCR1,FLSHxCR2) according to external device type
  - Configure DLL control register (DLLxCR) according to sample clock source selection
  - set MCR0[MDIS] to 0x0 (Exit module stop mode)
  - Configure LUT as needed (For AHB command or IP command)
  - Reset controller optionally (by set MCR0[SWRESET] to 0x1)

 @param[in] Fspi       Pointer to FSPI_MASTER structure of a FSPI controller
 @param[in] AmbaTotalSize Total Size for Memory mapped SPI flash devices

 @retval EFI_TIMEOUT    Timeout occured while configuring Fspi Controller
         EFI_SUCCESS    Successfully configured the controller
         EFI_DEVICE_ERROR  Error occurred while configuring Fspi controller
**/
EFI_STATUS
FspiSetup (
  IN  FSPI_MASTER           *Fspi,
  IN  UINT64                AmbaTotalSize
  )
{
  FSPI_REGISTERS                *Regs;
  UINT8                         Index;
  EFI_PHYSICAL_ADDRESS          AmbaBase;
  UINT64                        AmbaSizePerChip;
  EFI_STATUS                    Status;

  if ((Fspi == NULL) || (AmbaTotalSize == 0)) {
    return EFI_INVALID_PARAMETER;
  }

  Regs = Fspi->Regs;
  AmbaBase = Fspi->AmbaBase;
  Status = EFI_SUCCESS;

  // SW reset by writing 1 to MCR0[SWRESET]. This bit is auto-cleared by hardware after
  // software done. SW reset lasts about 64 cycles of serial root clock plus 2 cycles of ipg_clk
  // clock. Configurtion registers will not be reset. Software should poll this bit to wait for SW
  // reset done.
  Fspi->SetBits32 ( (UINTN)&Regs->Mcr0, MCR0_SWRESET);
  while (Fspi->Read32 ( (UINTN)&Regs->Mcr0) & MCR0_SWRESET) {
    MicroSecondDelay (1);
  }

  // Set MCR0[MDIS] to 0x1 (Make sure controller is configured in module stop mode)
  Fspi->SetBits32 ( (UINTN)&Regs->Mcr0, MCR0_MDIS);

  // Configure module control registers: MCR0, MCR1, MCR2. (Don't change MCR0[MDIS])
  Fspi->ClearSet32 (
          (UINTN)&Regs->Mcr0,
          MCR0_ATDFEN | MCR0_HSEN | MCR0_DOZEEN | MCR0_COMBINATIONEN |
          MCR0_SCKFREERUNEN | MCR0_LEARNEN | MCR0_ARDFEN,
          (MCR0_END_CFD_64BIT_LE & MCR0_END_CFG_MASK) |
          (MCR0_RXCLKSRC_DEFAULT & MCR0_RXCLKSRC_MASK)
          );
  // Device type may be different on A1/A2/B1/B2
  // For this case, clear the MCR2[SAMEDEVICEEN] bit and configure FLSHxCR0 and
  // FLSHxCR1 register separately for up to four external devices.
  Fspi->ClearBits32 (
          (UINTN)&Regs->Mcr2,
          MCR2_SAMEDEVICEEN
          );

  // prefetch and no start address alignment limitation
  Fspi->Write32 ( (UINTN)&Regs->AhbCr, AHBCR_PREFETCHEN | AHBCR_READADDROPT);

  for (Index = 0; Index < 7; Index++) {
    Fspi->Write32 ( (UINTN)(&Regs->AhbRxBuf0Cr0 + Index), 0);
  }

  // Set ADATSZ with the maximum AHB buffer size to improve the read performance.
  Fspi->Write32 (
          (UINTN)&Regs->AhbRxBuf7Cr0,
          (RX_AHBBUF_SIZE >> 3) | AHBRXBUFxCR_PREFETCHEN
          );

  if (Fspi->NumChipselect) {
    AmbaSizePerChip = AmbaTotalSize / Fspi->NumChipselect;
    for (Index = FSPI_CHIP_SELECT_0; Index < Fspi->NumChipselect; Index++) {
      Fspi->Write32 (
              ( (UINTN)(&Regs->FlshA1Cr0 + Index)),
              ( ( (AmbaSizePerChip + SIZE_1KB - 1) / SIZE_1KB) & FLSHCR0_FLSHSZ_MASK)
              );
      Fspi->ClearBits32 (
              ( (UINTN)(&Regs->FlshA1Cr1 + Index)),
              (FLSHCR1_CAS_MASK | FLSHCR1_WA)
              );
    }
  } else {
    Status = EFI_DEVICE_ERROR;
  }

  if (EFI_ERROR (Status)) {
    return Status;
  }

  // Reset the DLL register to default value
  Fspi->Write32 ( (UINTN)&Regs->DllaCr, 0x0100);
  Fspi->Write32 ( (UINTN)&Regs->DllbCr, 0x0100);

  // enable module
  Fspi->ClearBits32 ( (UINTN)&Regs->Mcr0, MCR0_MDIS);

  return Status;
}
