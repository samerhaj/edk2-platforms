/** @QspiDxe.c

  Driver for installing SPI Master and other spi protocols
  over QSPI controller Handle

  Copyright 2017-2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the bsd
  license which accompanies this distribution. the full text of the license may
  be found at http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
**/

#include <Library/ArmLib.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeLib.h>
#include <Library/SocClockLib.h>

#include "QspiDxe.h"

/**
  Fixup internal data so that EFI can be call in virtual mode.
  Call the passed in Child Notify event and convert any pointers in
  lib to virtual mode.

  @param[in]    Event   The Event that is being processed
  @param[in]    Context Event Context
 **/
STATIC
VOID
EFIAPI
QspiVirtualNotifyEvent (
  IN EFI_EVENT        Event,
  IN VOID             *Context
  )
{
  QSPI_MASTER     *QMaster;

  QMaster = (QSPI_MASTER *)Context;

  EfiConvertPointer (0x0, (VOID **)&QMaster->Regs);
  // Convert SpiMaster protocol
  EfiConvertPointer (0x0, (VOID **)&QMaster->QspiHcProtocol.ChipSelect);
  EfiConvertPointer (0x0, (VOID **)&QMaster->QspiHcProtocol.Clock);
  EfiConvertPointer (0x0, (VOID **)&QMaster->QspiHcProtocol.Transaction);

  return;
}

/**
  Set up the clock generator to produce the correct clock frequency, phase and
  polarity for a SPI chip.

  This routine is called at TPL_NOTIFY.
  This routine updates the clock generator to generate the correct frequency
  and polarity for the SPI clock.

  @param[in] This           Pointer to an EFI_SPI_HC_PROTOCOL structure.
  @param[in] SpiPeripheral  Pointer to a EFI_SPI_PERIPHERAL data structure from
                            which the routine can access the ClockParameter,
                            ClockPhase and ClockPolarity fields. The routine
                            also has access to the names for the SPI bus and
                            chip which can be used during debugging.
  @param[in] ClockHz        Pointer to the requested clock frequency. The SPI
                            host controller will choose a supported clock
                            frequency which is less then or equal to this
                            value. Specify zero to turn the clock generator
                            off. The actual clock frequency supported by the
                            SPI host controller will be returned.

  @retval EFI_SUCCESS      The clock was set up successfully
  @retval EFI_UNSUPPORTED  The SPI controller was not able to support the
                           frequency requested by ClockHz

**/
EFI_STATUS
QspiSetupPeripheral (
  IN CONST EFI_SPI_HC_PROTOCOL  *This,
  IN CONST EFI_SPI_PERIPHERAL   *SpiPeripheral,
  IN UINT32                     *ClockHz
  )
{
  QSPI_MASTER    *QMaster;
  UINT64         QspiInputClock;
  UINT32         ControllerNumber;

  if (!This || !SpiPeripheral || !ClockHz) {
    return EFI_INVALID_PARAMETER;
  }

  QMaster = BASE_CR (This, QSPI_MASTER, QspiHcProtocol);

  ControllerNumber = QMaster->DevicePath.Controller.ControllerNumber;
  QspiInputClock = SocGetClock (IP_QSPI, ControllerNumber);

  // without having to recalibrate the module clocks, the the serial flash device clock can be divided
  // by 2 (half speed) by setting the QSPI_SMPR[HSENA] bit.
  if (!QspiInputClock || *ClockHz < (QspiInputClock >> 1)) {
    return EFI_UNSUPPORTED;
  }

  // Disable QSPI controller before changing Sampling
  EnableQspiModule (QMaster, FALSE);

  if ( (*ClockHz >= (QspiInputClock >> 1)) && (*ClockHz < QspiInputClock)) {
    if (SpiPeripheral->ClockPhase) {
      QspiConfigureSampling (
        QMaster,
        0,
        SMPR_HSENA_MASK | SMPR_HSPHS_MASK
        );
    } else {
      QspiConfigureSampling (
        QMaster,
        SMPR_HSPHS_MASK,
        SMPR_HSENA_MASK
        );
    }
    *ClockHz = (QspiInputClock >> 1);
  } else {
    if (SpiPeripheral->ClockPhase) {
      QspiConfigureSampling (
        QMaster,
        SMPR_HSENA_MASK | SMPR_HSPHS_MASK,
        SMPR_FSPHS_MASK
        );
    } else {
      QspiConfigureSampling (
        QMaster,
        SMPR_HSENA_MASK | SMPR_HSPHS_MASK | SMPR_FSPHS_MASK,
        0
        );
    }
    *ClockHz = QspiInputClock;
  }

  // in QSPI controller its not possible to set the clock polarity.
  // Enable QSPI controller before returning
  EnableQspiModule (QMaster, TRUE);
  return EFI_SUCCESS;
}

/**
  Assert or deassert the SPI chip select.

  This routine is called at TPL_NOTIFY.
  Update the value of the chip select line for a SPI peripheral. The SPI bus
  layer calls this routine either in the board layer or in the SPI controller
  to manipulate the chip select pin at the start and end of a SPI transaction.

  @param[in] This           Pointer to an EFI_SPI_HC_PROTOCOL structure.
  @param[in] SpiPeripheral  The address of an EFI_SPI_PERIPHERAL data structure
                            describing the SPI peripheral whose chip select pin
                            is to be manipulated. The routine may access the
                            ChipSelectParameter field to gain sufficient
                            context to complete the operation.
  @param[in] PinValue       The value to be applied to the chip select line of
                            the SPI peripheral.

  @retval EFI_SUCCESS            The chip select was set as requested
  @retval EFI_NOT_READY          Support for the chip select is not properly
                                 initialized
  @retval EFI_INVALID_PARAMETER  The ChipSeLect value or its contents are
                                 invalid

**/
EFI_STATUS
QspiChipSelect (
  IN CONST EFI_SPI_HC_PROTOCOL  *This,
  IN CONST EFI_SPI_PERIPHERAL   *SpiPeripheral,
  IN BOOLEAN                    PinValue
  )
{
  QSPI_MASTER    *QMaster;
  UINT32         ChipSelect;

  if (!SpiPeripheral || !This || !(SpiPeripheral->ChipSelectParameter)) {
    return EFI_INVALID_PARAMETER;
  }

  QMaster = BASE_CR (This, QSPI_MASTER, QspiHcProtocol);

  ///
  /// Address of a data structure containing the additional values which
  /// describe the necessary control for the chip select. When SpiPeripheral->ChipSelect is
  /// NULL, the declaration for this data structure is provided by the vendor
  /// of the host's SPI controller driver. The vendor's documentation specifies
  /// the necessary values to use for the chip select pin selection and
  /// control. When SpiPeripheral->ChipSelect is not NULL, the declaration for this data
  /// structure is provided by the board layer.
  ///
  ChipSelect = *((UINT32 *)(SpiPeripheral->ChipSelectParameter));

  if (ChipSelect >= QMaster->NumChipselect) {
    return EFI_INVALID_PARAMETER;
  }

  //
  // Qspi controller asserts Chip select when starting transaction and deasserts chip select after transaction
  // has completed. we can just control which chip select to assert/deassert here.
  //
  if (PinValue == TRUE) {
    ChipSelect = QSPI_CHIP_SELECT_0;
  }

  switch (ChipSelect) {
    case QSPI_CHIP_SELECT_0:
      QMaster->CurAmbaBase = QMaster->AmbaBase;
      break;

    case QSPI_CHIP_SELECT_1:
      QMaster->CurAmbaBase = QMaster->Read32 ( (UINTN)&QMaster->Regs->Sfa1ad);
      break;

    case QSPI_CHIP_SELECT_2:
      QMaster->CurAmbaBase = QMaster->Read32 ( (UINTN)&QMaster->Regs->Sfa2ad);
      break;

    case QSPI_CHIP_SELECT_3:
      QMaster->CurAmbaBase = QMaster->Read32 ( (UINTN)&QMaster->Regs->Sfb1ad);
      break;
  }

  return EFI_SUCCESS;
}

/**
  Perform the SPI transaction on the SPI peripheral using the SPI host
  controller.

  This routine is called at TPL_NOTIFY.
  This routine synchronously returns EFI_SUCCESS indicating that the
  asynchronous SPI transaction was started. The routine then waits for
  completion of the SPI transaction prior to returning the final transaction
  status.

  @param[in] This            Pointer to an EFI_SPI_HC_PROTOCOL structure.
  @param[in] RequestPacket   Pointer to a EFI_SPI_REQUEST_PACKET containing
                             the description of the SPI transactions to perform.

  @retval EFI_SUCCESS             The transaction completed successfully.
  @retval EFI_ALREADY_STARTED     The controller is busy with another transaction.
  @retval EFI_BAD_BUFFER_SIZE     The Length value in SPI Transaction is wrong.
  @retval EFI_DEVICE_ERROR        There was an SPI error during the transaction.
  @retval EFI_INVALID_PARAMETER   The parameters specified in RequestPacket are not
                                  Valid. or the RequestPacket is NULL.
  @retval EFI_NO_RESPONSE         The SPI device is not responding to the slave
                                  address.  EFI_DEVICE_ERROR will be returned if
                                  the controller cannot distinguish when the NACK
                                  occurred.
  @retval EFI_UNSUPPORTED         The controller does not support the requested
                                  transaction.
**/
EFI_STATUS
QspiTransaction (
  IN CONST EFI_SPI_HC_PROTOCOL     *This,
  IN EFI_SPI_REQUEST_PACKET        *RequestPacket
  )
{
  QSPI_MASTER    *QMaster;
  QSPI_REQUEST   Request;
  EFI_STATUS     Status;

  if (!RequestPacket || !This || !(RequestPacket->TransactionCount)) {
    return EFI_INVALID_PARAMETER;
  }

  QMaster = BASE_CR (This, QSPI_MASTER, QspiHcProtocol);

  SetMem (&Request, sizeof(QSPI_REQUEST), 0);
  Request.LutId = SEQ_ID_IP_MODE;

  Status = ParseRequest (QMaster, RequestPacket, &Request);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Error in Parsing SPI request\n"));
    return Status;
  }
  ArmInstructionSynchronizationBarrier ();
  ArmDataSynchronizationBarrier ();

  if (Request.Type == SPI_REQUEST_WRITE_THEN_READ) {
    Status = ReadTransaction (QMaster, &Request);
  } else if (Request.Type == SPI_REQUEST_WRITE_ONLY) {
    Status = WriteTransaction (QMaster, &Request);
  } else {
    DEBUG ((DEBUG_ERROR, "SPI request not recognized\n"));
    Status = EFI_UNSUPPORTED;
  }
  return Status;
}

/**
 Installs the SpiMaster Protocol and Device Path protocol on to that handle.

 Also this function registers for an Virtual Address change event, to convert the runtime
 memory allocated from physical address space to virtual address space.

 @param[in]   QMaster      Pointer to QSPI_MASTER strcture of a QSPI controller
 @param[in]   Runtime      Weather Qspi controller is to be used at runtime or not

 @retval EFI_DEVICE_ERROR      Not able to Install SPI Host Controller Protocol or if the QMaster
                               Controller is runtime, then not able to set the memory attributes for this controller.
 @retval EFI_SUCCESS           Protocols installed successfully on QSPI controllers' handles.
**/
EFI_STATUS
EFIAPI
QspiInstallProtocol (
  IN  QSPI_MASTER    *QMaster,
  IN  BOOLEAN        Runtime
  )
{
  EFI_STATUS Status;

  // Install SPI Host controller protocol and Device Path Protocol
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &QMaster->Handle,
                  &gEfiSpiHcProtocolGuid, &QMaster->QspiHcProtocol,
                  &gEfiDevicePathProtocolGuid, &QMaster->DevicePath,
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a:%d Error = %r\n", __FUNCTION__, __LINE__, Status));
    return Status;
  }

  if (Runtime) {
    // Declare the controller registers as EFI_MEMORY_RUNTIME
    Status = gDS->AddMemorySpace (
                    EfiGcdMemoryTypeMemoryMappedIo,
                    (UINTN)QMaster->Regs,
                    ALIGN_VALUE (sizeof (QSPI_REGISTERS), SIZE_64KB),
                    EFI_MEMORY_UC | EFI_MEMORY_RUNTIME
                    );
    if (EFI_ERROR (Status)) {
      goto UninstallProtocol;
    }

    Status = gDS->SetMemorySpaceAttributes (
                    (UINTN)QMaster->Regs,
                    ALIGN_VALUE (sizeof (QSPI_REGISTERS), SIZE_64KB),
                    EFI_MEMORY_UC | EFI_MEMORY_RUNTIME
                    );
    if (EFI_ERROR (Status)) {
      goto RemoveMemorySpace;
    }

    //
    // Register for the virtual address change event
    //
    Status = gBS->CreateEventEx (
                    EVT_NOTIFY_SIGNAL,
                    TPL_NOTIFY,
                    QspiVirtualNotifyEvent,
                    (VOID *)QMaster,
                    &gEfiEventVirtualAddressChangeGuid,
                    &QMaster->Event
                    );
    if (EFI_ERROR (Status)) {
      goto RemoveMemorySpace;
    }

    // Connect the controller Recursively to SPI bus
    Status = gBS->ConnectController (QMaster->Handle, NULL, NULL, TRUE);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "%a:%d Error = %r\n", __FUNCTION__, __LINE__, Status));
      goto RemoveEvent;
    }
  }

  return Status;

RemoveEvent:
  Status = gBS->CloseEvent (QMaster->Event);
  ASSERT_EFI_ERROR (Status);

RemoveMemorySpace:
  Status = gDS->RemoveMemorySpace (
                  (UINTN)QMaster->Regs,
                  ALIGN_VALUE (sizeof (QSPI_REGISTERS), SIZE_64KB)
                  );
  ASSERT_EFI_ERROR (Status);

UninstallProtocol:
  Status = gBS->UninstallMultipleProtocolInterfaces (
                  QMaster->Handle,
                  &gEfiSpiHcProtocolGuid, &QMaster->QspiHcProtocol,
                  &gEfiDevicePathProtocolGuid, &QMaster->DevicePath,
                  NULL
                  );
  ASSERT_EFI_ERROR (Status);

  return EFI_DEVICE_ERROR;
}

/**
 This function reads the device tree file from FV and pass on to device tree parser.
 device tree parser function parses the device tree and for each QSPI controller node found,
 allocates runtime memory for the internal data structure containing a handle and installs
 the SpiMaster Protocol and Device Path protocol on to that handle.

 Also this function registers for an Virtual Address change event, to convert the runtime
 memory allocated from physical address space to virtual address space.

 @param[in]   ImageHandle      Handle of QSPI driver efi image
 @param[in]   SystemTable      System Table Pointer

 @retval EFI_OUT_OF_RESOURCES  No pool memory available
 @retval EFI_CRC_ERROR         Device tree is not correct.
 @retval EFI_NOT_FOUND         No usable QSPI controller found.
 @retval EFI_SUCCESS           Protocols installed successfully on QSPI controllers' handles.
**/
EFI_STATUS
EFIAPI
QspiInitialise (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  UINTN                 QSpiCount;
  VOID                  *Dtb;
  UINTN                 DtbSize;
  EFI_STATUS            Status;

  Status = GetSectionFromAnyFv (
             &gDtPlatformDefaultDtbFileGuid,
             EFI_SECTION_RAW,
             0,
             &Dtb,
             &DtbSize
             );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to get device tree\n"));
    return EFI_NOT_FOUND;
  }

  if (fdt_check_header (Dtb)) {
    DEBUG ((DEBUG_ERROR, "Invalid Device Tree\n"));
    return EFI_CRC_ERROR;
  }

  Status = ParseDeviceTree (Dtb, &QSpiCount);
  if (EFI_ERROR (Status) && (QSpiCount == 0)) {
    return EFI_NOT_FOUND;
  }

  return EFI_SUCCESS;
}
