/** @QspiFlashOps.c

  Functions for implementing qspi controller functionality.

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
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>

#include "QspiDxe.h"

/**
 Dump Qspi Registers for debugging

 @param[in]  QMaster           Pointer to QSPI_MASTER strcture of a QSPI controller
**/
VOID
DumpRegs (
  IN  QSPI_MASTER  *QMaster
  )
{
  UINT32 Index;
  QSPI_REGISTERS *Regs;

  Regs = QMaster->Regs;

  DEBUG ((DEBUG_INFO, "Mcr       :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Mcr)));
  DEBUG ((DEBUG_INFO, "Ipcr      :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Ipcr)));
  DEBUG ((DEBUG_INFO, "Flshcr    :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Flshcr)));
  DEBUG ((DEBUG_INFO, "Buf0cr    :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Buf0cr)));
  DEBUG ((DEBUG_INFO, "Buf1cr    :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Buf1cr)));
  DEBUG ((DEBUG_INFO, "Buf2cr    :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Buf2cr)));
  DEBUG ((DEBUG_INFO, "Buf3cr    :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Buf3cr)));
  DEBUG ((DEBUG_INFO, "Bfgencr   :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Bfgencr)));
  DEBUG ((DEBUG_INFO, "Soccr     :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Soccr)));
  DEBUG ((DEBUG_INFO, "Buf0ind   :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Buf0ind)));
  DEBUG ((DEBUG_INFO, "Buf1ind   :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Buf1ind)));
  DEBUG ((DEBUG_INFO, "Buf2ind   :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Buf2ind)));
  DEBUG ((DEBUG_INFO, "Sfar      :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Sfar)));
  DEBUG ((DEBUG_INFO, "Sfacr     :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Sfacr)));
  DEBUG ((DEBUG_INFO, "Smpr      :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Smpr)));
  DEBUG ((DEBUG_INFO, "Rbsr      :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Rbsr)));
  DEBUG ((DEBUG_INFO, "Rbct      :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Rbct)));
  DEBUG ((DEBUG_INFO, "Tbsr      :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Tbsr)));
  DEBUG ((DEBUG_INFO, "Tbdr      :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Tbdr)));
  DEBUG ((DEBUG_INFO, "Sr        :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Sr)));
  DEBUG ((DEBUG_INFO, "Fr        :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Fr)));
  DEBUG ((DEBUG_INFO, "Rser      :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Rser)));
  DEBUG ((DEBUG_INFO, "Spndst    :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Spndst)));
  DEBUG ((DEBUG_INFO, "Sptrclr   :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Sptrclr)));
  DEBUG ((DEBUG_INFO, "Sfa1ad    :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Sfa1ad)));
  DEBUG ((DEBUG_INFO, "Sfa2ad    :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Sfa2ad)));
  DEBUG ((DEBUG_INFO, "Sfb1ad    :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Sfb1ad)));
  DEBUG ((DEBUG_INFO, "Sfb2ad    :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Sfb2ad)));
  for (Index = 0; Index < ARRAY_SIZE(Regs->Rbdr); Index++) {
    DEBUG ((DEBUG_INFO, "Rbdr[%d]   :0x%08x \n", Index, QMaster->Read32 ( (UINTN)&Regs->Rbdr[Index])));
  }
  DEBUG ((DEBUG_INFO, "Lutkey    :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Lutkey)));
  DEBUG ((DEBUG_INFO, "Lckcr     :0x%08x \n", QMaster->Read32 ( (UINTN)&Regs->Lckcr)));
  for (Index = 0; Index < ARRAY_SIZE(Regs->Lut); Index++) {
    DEBUG ((DEBUG_INFO, "Lut[%d]   :0x%08x \n", Index, QMaster->Read32 ( (UINTN)&Regs->Lut[Index])));
  }

  return;
}


/**
 Check Flag register if any error bit is set or not while
 executing IP mode instruction.

 @param[in]  QMaster           Pointer to QSPI_MASTER strcture of a QSPI controller
 @param[in]  Fr                Address of Flag Register (Fr) of a QSPI controller

 @retval    EFI_DEVICE_ERROR   if an error bit is set
 @retval    EFI_SUCCESS        No error bit is set
**/
STATIC
EFI_STATUS
CheckFlagRegister (
  IN  QSPI_MASTER  *QMaster,
  IN  UINTN        Fr
  )
{
  UINT32 FrReg;
  FrReg = QMaster->Read32 (Fr);
  if (FrReg & (FR_ILLINE | FR_IUEF | FR_IPIEF | FR_IPGEF | FR_IPAEF | FR_RBOF | FR_TBUF)) {
    DEBUG ((DEBUG_ERROR, "FrReg=0x%08x\n", FrReg));
    return EFI_DEVICE_ERROR;
  }
  return EFI_SUCCESS;
}

/**
 Check Status register for any ongoing SPI transaction.
 if a transaction is ongoing, wait for some time and keep checking the status register
 for completion. if after some retries, the status is still showing transaction ongoing,
 return error.

 @param[in]  QMaster              Pointer to QSPI_MASTER strcture of a QSPI controller

 @retval    EFI_ALREADY_STARTED   The transaction is ongoing
 @retval    EFI_SUCCESS           No transaction is ongoing
**/
STATIC
EFI_STATUS
CheckStatusRegister (
  IN  QSPI_MASTER  *QMaster
  )
{
  UINT32 SrReg;
  UINT8  Retry;

  for (Retry = QSPI_SR_RETRY_COUNT; Retry; Retry--) {
    SrReg = QMaster->Read32 ( (UINTN)&QMaster->Regs->Sr);
    // Check if the last AHB transaction has completed or not ?
    if ( (SrReg & SR_AHB_ACC) || (SrReg & SR_IP_ACC) || (SrReg & SR_BUSY)) {
      MicroSecondDelay(1);
      continue;
    } else {
      return EFI_SUCCESS;
    }
  }
  return EFI_ALREADY_STARTED;
}

/**
 Check Valid entries in Rx Buffer.
 once we start receieving data from SPI device, we first check the valid data received
 in Rx buffer and then read only that much data.

 @param[in]  QMaster    Pointer to QSPI_MASTER strcture of a QSPI controller

 @retval                Number of Valid entries in Rx Buffer.
**/
STATIC inline
UINT8
RbdrValidEntries (
  IN   QSPI_MASTER           *QMaster
  )
{
  QSPI_REGISTERS *Regs;
  Regs = QMaster->Regs;
  return (QMaster->Read32 ( (UINTN)&Regs->Rbsr) & RBSR_RDBFL_MASK) >> RBSR_RDBFL_SHIFT;
}

/**
 Update the Module configuration Register before any Read/Write operation.

 This function resets the TX and RX queue pointers, configures the endianness of
 Tx/Rx queue. It also returns the original value of Mcr register before these changes
 so that Mcr Register value can be restored after Read/Write Transaction.

 @param[in]  QMaster    Pointer to QSPI_MASTER strcture of a QSPI controller

 @retval                Module configuration register value, before any changes.
**/
STATIC
UINT32
UpdateMcr (
  IN  QSPI_MASTER *QMaster,
  IN  UINTN       Mcr
  )
{
  UINT32 McrReg;

  McrReg = QMaster->Read32 (Mcr);
  QMaster->Write32 (
             Mcr,
             MCR_CLR_RXF_MASK | MCR_CLR_TXF_MASK |
             MCR_IDLE_SIGNAL_DRIVE | MCR_END_CFD_32BIT_LE
             );

  return McrReg;
}

/**
  Enable or Disable the QSPI controller software reset

  It is advisable to reset both the serial flash domain and AHB domain at the same time.
  Resetting only one domain might lead to side effects.

  The software resets need the clock to be running to propagate to the design. The
  MCR[MDIS] should therefore be set to 0 when the software reset bits are asserted. Also,
  before they can be de-asserted again (by setting MCR[SWRSTHD] and MCR[SWRSTSD] to 0),
  it is recommended to set the MCR[MDIS] bit to 1.
  Once the software resets have been de-asserted, the normal operation can be started by
  setting the MCR[MDIS] bit to 0.

  @param[in]   QMaster          QSPI_MASTER structure of a QSPI controller
  @param[in]   ResetEn          TRUE: Enable QSPI controller software reset
                                FALSE: Disable QSPI controller software reset
**/
STATIC
VOID
QspiSwReset(
  IN  QSPI_MASTER *QMaster,
  IN  BOOLEAN     ResetEn
  )
{
  QSPI_REGISTERS *Regs;
  UINT32 Reg;

  Regs = QMaster->Regs;
  Reg = QMaster->Read32 ( (UINTN)&Regs->Mcr);

  if (ResetEn) {
    Reg |= MCR_SWRSTHD_MASK | MCR_SWRSTSD_MASK;
  } else {
    Reg &= ~(MCR_SWRSTHD_MASK | MCR_SWRSTSD_MASK);
  }

  QMaster->Write32 ( (UINTN)&Regs->Mcr, Reg);

  return;
}

/**
 Clears(puts) the QSPI controller from(into) Module Disabled state.

 The MDIS bit allows the clock to the non-memory mapped logic in the QuadSPI to be
 stopped, putting the QuadSPI in a software controlled power-saving state.

 @param[in]  QMaster     Pointer to QSPI_MASTER structure of a QSPI controller
 @param[in]  Enable      FALSE : Set Module Disable (MDS) bit of MCR Register
                          i.e. Disable QuadSPI clocks.
                         TRUE : clear MDS bit i.e. Enable QuadSPI clocks.
**/
VOID
EnableQspiModule (
  IN  QSPI_MASTER      *QMaster,
  IN  BOOLEAN          Enable
  )
{
  UINT32 Val;

  Val = QMaster->Read32 ( (UINTN)&QMaster->Regs->Mcr);

  if (Enable) {
    Val &= ~MCR_MDIS_MASK;
  } else {
    Val |= MCR_MDIS_MASK;
  }

  QMaster->Write32 ( (UINTN)&QMaster->Regs->Mcr, Val);

  return;
}

/**
 If we have changed the content of the flash by writing or erasing,
 we need to invalidate the AHB buffer. If we do not do so, we may read out
 the wrong data. The spec tells us reset the AHB domain and Serial Flash
 domain at the same time.

 @param[in]  QMaster    Pointer to QSPI_MASTER structure of a QSPI controller
**/
VOID
QspiInvalidateAHBBuffer (
  IN  QSPI_MASTER    *QMaster
  )
{
  QspiSwReset (QMaster, TRUE);
  // The minimum delay : 1 AHB + 2 SFCK clocks.
  // Delay 1 us is enough.
  MicroSecondDelay(1);

  QspiSwReset (QMaster, FALSE);
}

/**
  This function performs the WRITE_ONLY operation on SPI device.

  The QuadSPI module has only one method for writing to the external device:
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

  @param[in]   QMaster          QSPI_MASTER structure of a QSPI controller
  @param[in]   Request          QSPI Request structure which provides the LUT entry to use,
                                Address to write to, number of data bytes to write and buffer
                                from which data is to be written.

  @retval EFI_ALREADY_STARTED   The controller is busy with another transaction.
  @retval EFI_DEVICE_ERROR      There was an SPI error during the transaction.
  @retval EFI_SUCCESS           The transaction completed successfully.
**/
EFI_STATUS
WriteTransaction (
  IN  QSPI_MASTER    *QMaster,
  IN  QSPI_REQUEST   *Request
  )
{
  QSPI_REGISTERS  *Regs;
  UINT32          McrReg;
  UINT32          Data;
  UINT32          Index;
  UINT32          TxSize;
  UINT32          ToOrFrom;
  UINT32          TxBuffEmpty;
  UINT32          TxBuffFill;
  UINT32          *Txbuf;
  UINT8           TxWmarkSize; // Tx Water mark size in 32 bit entries
  EFI_STATUS      Status;

  Regs = QMaster->Regs;
  Status = EFI_SUCCESS;
  Txbuf = (UINT32 *)Request->Buffer;
  TxBuffFill = 0;
  TxWmarkSize = TX_WMRK + 1; // There must be atleast 128bit data available in TX FIFO for any pop operation

  Status = CheckStatusRegister(QMaster);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  /*Clear Tx and Rx FIFO buffers*/
  McrReg = UpdateMcr(QMaster, (UINTN)&Regs->Mcr);
  QMaster->Write32 ( (UINTN)&Regs->Rbct, RBCT_RXBRD_USEIPS);

  ToOrFrom = Request->Address + QMaster->CurAmbaBase;

  QMaster->Write32 ( (UINTN)&Regs->Sfar, ToOrFrom);
  QMaster->Write32 ( (UINTN)&Regs->Sfacr, 0);

  TxSize = (Request->Length > TX_BUFFER_SIZE) ? TX_BUFFER_SIZE : Request->Length;
  TxBuffFill = TxSize;

  while (TxSize) {
    Index = 0;
    while (Index++ < TxWmarkSize) {
      if (TxSize == 0) {
        Data = 0;
        QMaster->Write32 ( (UINTN)&Regs->Tbdr, Data);
      } else if (TxSize < sizeof (Data)) {
        Data = 0;
        CopyMem (&Data, Txbuf, TxSize);
        QMaster->Write32 ( (UINTN)&Regs->Tbdr, Data);
        Txbuf = (UINT32 *)((UINT8 *)Txbuf + TxSize);
        TxSize = 0;
      } else {
        QMaster->Write32 ( (UINTN)&Regs->Tbdr, *Txbuf++);
        TxSize -= sizeof (UINT32);
      }
    }
  }
  TxSize = (Request->Length - TxBuffFill);

  QMaster->Write32 ( (UINTN)&Regs->Ipcr, (Request->LutId << IPCR_SEQID_SHIFT) | Request->Length);

  while (TxSize) {
    if (QMaster->Read32 ( (UINTN)&Regs->Sr) & SR_TXFULL) {
      /* TX Buffer Full. No more data can be stored. */
      continue;
    }
    /*
     * TX Buffer Fill Level. The TRBFL field contains the number of *
     * entries of 4 bytes each available in the TX Buffer for the   *
     * QuadSPI module to transmit to the serial flash device.       *
     */
    TxBuffFill = ((QMaster->Read32 ( (UINTN)&Regs->Tbsr) & TBSR_TRBL_MASK) >> TBSR_TRBL_SHIFT) << 2;
    TxBuffEmpty = TX_BUFFER_SIZE - TxBuffFill;
    if (TxBuffEmpty < (TxWmarkSize * sizeof (UINT32))) {
      continue;
    }
    Index = 0;
    while (Index++ < TxWmarkSize) {
      if (TxSize == 0) {
        Data = 0;
        QMaster->Write32 ( (UINTN)&Regs->Tbdr, Data);
      } else if (TxSize < sizeof (Data)) {
        Data = 0;
        CopyMem (&Data, Txbuf, TxSize);
        QMaster->Write32 ( (UINTN)&Regs->Tbdr, Data);
        Txbuf = (UINT32 *)((UINT8 *)Txbuf + TxSize);
        TxSize = 0;
      } else {
        QMaster->Write32 ( (UINTN)&Regs->Tbdr, *Txbuf++);
        TxSize -= sizeof (UINT32);
      }
    }
  }
  while (QMaster->Read32 ( (UINTN)&Regs->Sr) & (SR_IP_ACC | SR_BUSY)) {
    Status = CheckFlagRegister(QMaster, (UINTN)&Regs->Fr);
    if (EFI_ERROR (Status)) {
      break;
    }
  }

  QMaster->Write32 ( (UINTN)&Regs->Mcr, McrReg);

  // Invalidate the AHB buffer contents
  QspiInvalidateAHBBuffer (QMaster);

  return Status;
}

/**
  This function performs the WRITE_THEN_READ operation on SPI device.

  The QuadSPI module has two different methods for reading from the external device:
  IP commands and AHB commands.
  Each of these methods has its own internal buffers.

  Before calling this function ensure that a correct read sequence in programmed in the LUT in
  accordance with the device connected on board. and the valid address, number of data bytes,
  and buffer to read data into is provided to this function. The LUT entry to use, should also
  be provided to this function.

  As programmed in LUT entry, first some control information is sent to SPI device
  (like command, address etc) and then the device output data which is read into internal
  buffers. Hence the operation is WRITE_THEN_READ. Then the data is copied from internal
  buffers to buffer provided in QSPI Request structure.

  @param[in]   QMaster          QSPI_MASTER structure of a QSPI controller
  @param[in]   Request          QSPI Request structure which provides the LUT entry to use,
                                Address to read from, number of data bytes to read and buffer
                                in which data is to be read.

  @retval EFI_ALREADY_STARTED   The controller is busy with another transaction.
  @retval EFI_DEVICE_ERROR      There was an SPI error during the transaction.
  @retval EFI_SUCCESS           The transaction completed successfully.
**/
EFI_STATUS
ReadTransaction (
  IN   QSPI_MASTER     *QMaster,
  IN   QSPI_REQUEST    *Request
  )
{
  QSPI_REGISTERS *Regs;
  UINT32         McrReg;
  UINT32         Data;
  UINT32         Size;
  UINT32         Index;
  UINT32         RxValidEntries;
  UINT32         From;
  UINT32         *Rxbuf;
  EFI_STATUS     Status;

  Data = 0;
  Status = EFI_SUCCESS;
  Regs = QMaster->Regs;
  Rxbuf = (UINT32 *)Request->Buffer;

  Status = CheckStatusRegister (QMaster);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  /*Clear Tx and Rx FIFO buffers*/
  McrReg = UpdateMcr (QMaster, (UINTN)&Regs->Mcr);
  QMaster->Write32 ( (UINTN)&Regs->Rbct, RBCT_RXBRD_USEIPS);

  From = Request->Address + QMaster->CurAmbaBase;

  while (Request->Length > 0) {
    QMaster->Write32 ( (UINTN)&Regs->Sfar, From);

    Size = (Request->Length > RX_BUFFER_SIZE) ? RX_BUFFER_SIZE : Request->Length;

    QMaster->Write32 (
               (UINTN)&Regs->Ipcr,
               (Request->LutId << IPCR_SEQID_SHIFT) | Size
               );
    while (QMaster->Read32 ( (UINTN)&Regs->Sr) & (SR_IP_ACC | SR_BUSY)) {
      Status = CheckFlagRegister (QMaster, (UINTN)&Regs->Fr);
      if (EFI_ERROR (Status)) {
        break;
      }
    }
    if (EFI_ERROR (Status)) {
      break;
    }

    From += Size;
    Request->Length -= Size;
    Index = 0;

    while (Size) {
      RxValidEntries = RbdrValidEntries(QMaster);
      while (RxValidEntries - Index) {
        Data = QMaster->Read32 ( (UINTN)&Regs->Rbdr[Index++]);
        if (Size < sizeof(Data)) {
          CopyMem(Rxbuf++, &Data, Size);
          Size = 0;
        } else {
          CopyMem(Rxbuf++, &Data, sizeof(Data));
          Size -= sizeof(Data);
        }
      }
    }
    QMaster->Write32 (
               (UINTN)&Regs->Mcr,
               QMaster->Read32 ( (UINTN)&Regs->Mcr) | MCR_CLR_RXF_MASK
               );
  }

  QMaster->Write32 ( (UINTN)&Regs->Mcr, McrReg);

  return Status;
}

/**
 Configure sampling of incoming data

 @param[in] QMaster     Pointer to QSPI_MASTER structure of a QSPI controller
 @param[in] Clear       Clear Bits of SMPR
 @param[in] Set         Set Bits of SMPR
**/
VOID
QspiConfigureSampling (
  IN  QSPI_MASTER *QMaster,
  IN  UINT32  Clear,
  IN  UINT32  Set
  )
{
  UINT32 Val;

  Val = QMaster->Read32 ((UINTN)&QMaster->Regs->Smpr);
  Val &= ~Clear;
  Val |= Set;
  QMaster->Write32 ((UINTN)&QMaster->Regs->Smpr, Val);

  return;
}

/**
 Configure The QSpi controller at startup

 @param[in] QMaster       Pointer to QSPI_MASTER structure of a QSPI controller
 @param[in] AmbaTotalSize Total Size for Memory mapped SPI flash devices

 @retval EFI_INVALID_PARAMETER  QMaster is Null or AmbaTotalSize is zero
 @retval EFI_SUCCESS       Successfully configured the controller
 @retval EFI_DEVICE_ERROR  Error occurred while configuring QSpi controller
**/
EFI_STATUS
QspiSetup (
  IN  QSPI_MASTER           *QMaster,
  IN  UINT64                AmbaTotalSize
  )
{
  QSPI_REGISTERS                *Regs;
  UINT32                        McrVal;
  UINT64                        AmbaSizePerChip;
  EFI_STATUS                    Status;

  if ((QMaster == NULL) || (AmbaTotalSize == 0)) {
    return EFI_INVALID_PARAMETER;
  }

  Status = EFI_SUCCESS;
  Regs = QMaster->Regs;

  // Reset QSPI Module
  QspiSwReset (QMaster, TRUE);

  /* Put the QSPI controller in Module Disable Mode */
  McrVal = QMaster->Read32 ( (UINTN)&Regs->Mcr);
  McrVal |= MCR_IDLE_SIGNAL_DRIVE | MCR_MDIS_MASK | (McrVal & MCR_END_CFD_MASK);

  QMaster->Write32 ( (UINTN)&Regs->Mcr, McrVal);

  QspiConfigureSampling (
    QMaster,
    (SMPR_FSDLY_MASK | SMPR_DDRSMP_MASK | SMPR_FSPHS_MASK | SMPR_HSENA_MASK),
    0
    );

  if (QMaster->NumChipselect) {
    //
    // Assign AMBA Memory Zone for Every CS
    // QSPI Has Two Channels And Every Channel Has Two CS.
    // if No Of CS Is 2, The AMBA Memory Will Be Divided Into Two Parts
    // And Assign To Every Channel. This Indicate That Every
    // Channel Only Has One Valid CS.
    // if No Of CS Is 4, The AMBA Memory Will Be Divided Into Four Parts
    // And Assign To Every Chipselect.Every Channel Will Has Two Valid CS.
    //
    AmbaSizePerChip = AmbaTotalSize >> ((QMaster->NumChipselect + 1) >> 1);

    //
    // In case Of Single Die Flash Devices, TOP_ADDR_MEMA2 And
    // TOP_ADDR_MEMB2 Should Be Initialized To TOP_ADDR_MEMA1
    // And TOP_ADDR_MEMB1 Respectively. This Would Ensure
    // That Complete Memory Map Is Assigned To One Flash Device.
    //
    switch (QMaster->NumChipselect) {
      case QSPI_CHIP_SELECT_MAX :
        QMaster->Write32 (
                   (UINTN)&Regs->Sfb2ad,
                   QMaster->AmbaBase + QSPI_CHIP_SELECT_MAX * AmbaSizePerChip
                   );
        // don't use break; use fall through mode

      case QSPI_CHIP_SELECT_3 :
        QMaster->Write32 (
                   (UINTN)&Regs->Sfb1ad,
                   QMaster->AmbaBase + QSPI_CHIP_SELECT_3 * AmbaSizePerChip
                   );
        // don't use break; use fall through mode

      case QSPI_CHIP_SELECT_2 :
        QMaster->Write32 (
                   (UINTN)&Regs->Sfa2ad,
                   QMaster->AmbaBase + QSPI_CHIP_SELECT_2 * AmbaSizePerChip
                   );
        // don't use break; use fall through mode

      default:
        QMaster->Write32 (
                   (UINTN)&Regs->Sfa1ad,
                   QMaster->AmbaBase + AmbaSizePerChip
                   );
        break;
    }
  }

  // Enable the QSPI Module
  EnableQspiModule (QMaster, TRUE);
  // Clear SW Reset Bits
  QspiSwReset (QMaster, FALSE);

  return Status;
}
