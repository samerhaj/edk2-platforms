/** @FspiDxe.c

  Driver for installing SPI Master and other spi protocols
  over FSPI controller Handle

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

#include <Library/ArmLib.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeLib.h>
#include <Library/SocClockLib.h>
#include <Protocol/SpiConfigData.h>

#include "FspiDxe.h"

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
FspiVirtualNotifyEvent (
  IN EFI_EVENT        Event,
  IN VOID             *Context
  )
{
  FSPI_MASTER     *Fspi;

  Fspi = (FSPI_MASTER *)Context;

  EfiConvertPointer (0x0, (VOID **)&Fspi->Regs);
  EfiConvertPointer (0x0, (VOID **)&Fspi->Write32);
  EfiConvertPointer (0x0, (VOID **)&Fspi->Read32);
  EfiConvertPointer (0x0, (VOID **)&Fspi->ClearBits32);
  EfiConvertPointer (0x0, (VOID **)&Fspi->SetBits32);
  EfiConvertPointer (0x0, (VOID **)&Fspi->ClearSet32);
  // Convert SpiMaster protocol
  EfiConvertPointer (0x0, (VOID **)&Fspi->FspiHcProtocol.ChipSelect);
  EfiConvertPointer (0x0, (VOID **)&Fspi->FspiHcProtocol.Clock);
  EfiConvertPointer (0x0, (VOID **)&Fspi->FspiHcProtocol.Transaction);

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
FspiClock (
  IN CONST EFI_SPI_HC_PROTOCOL  *This,
  IN CONST EFI_SPI_PERIPHERAL   *SpiPeripheral,
  IN UINT32                     *ClockHz
  )
{
  FSPI_MASTER    *Fspi;
  UINT64         FspiInputClock;
  UINT32         ControllerNumber;

  if (!This || !SpiPeripheral || !ClockHz) {
    return EFI_INVALID_PARAMETER;
  }

  Fspi = BASE_CR (This, FSPI_MASTER, FspiHcProtocol);

  ControllerNumber = Fspi->DevicePath.Controller.ControllerNumber;
  // It's safe to get the clock from SocClockLib, as long as we are in UEFI context
  // We Assume that the boot loader is capable of altering the clock frequency of an IP block
  // but OS will not alter the clock of IP, which is reserved for runtime access.
  if (EfiAtRuntime ()) {
    FspiInputClock = Fspi->ClockHz;
  } else {
    FspiInputClock = SocGetClock (IP_FLEX_SPI, ControllerNumber);
    Fspi->ClockHz = FspiInputClock;
  }

  // The MCR0[HSEN] bit needs to be disabled and it must not be programmed to 1. It’s
  // default value is 0 and this default value must be used.
  if (!FspiInputClock || (*ClockHz < FspiInputClock)) {
    return EFI_UNSUPPORTED;
  }

  *ClockHz = FspiInputClock;

  // Disable FSPI controller before changing Sampling

  // FlexSPI support only SPI clock mode 0: Clock polarity (CPOL)=0 and Clock Phase (CPHA)=0.
  // SCK will stay at logic low state when SPI bus is idle.
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
FspiChipSelect (
  IN CONST EFI_SPI_HC_PROTOCOL  *This,
  IN CONST EFI_SPI_PERIPHERAL   *SpiPeripheral,
  IN BOOLEAN                    PinValue
  )
{
  FSPI_MASTER       *Fspi;
  FSPI_REGISTERS    *Regs;
  UINT32            ChipSelect;
  UINT64            FlashSize;

  if (!SpiPeripheral || !This || !(SpiPeripheral->ChipSelectParameter)) {
    return EFI_INVALID_PARAMETER;
  }

  Fspi = BASE_CR (This, FSPI_MASTER, FspiHcProtocol);
  Regs = Fspi->Regs;

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

  if (ChipSelect >= Fspi->NumChipselect) {
    return EFI_INVALID_PARAMETER;
  }

  //
  // Fspi controller asserts Chip select when starting transaction and deasserts chip select after transaction
  // has completed. we can just control which chip select to assert/deassert here.
  //
  if (PinValue == TRUE) {
    return EFI_SUCCESS;
  }

  Fspi->CurCSOffset = 0;
  while (ChipSelect-- != FSPI_CHIP_SELECT_0) {
    FlashSize = Fspi->Read32 ( (UINTN)(&Regs->FlshA1Cr0 + ChipSelect)) & FLSHCR0_FLSHSZ_MASK;
    FlashSize *= SIZE_1KB;
    Fspi->CurCSOffset += FlashSize;
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
FspiTransaction (
  IN CONST EFI_SPI_HC_PROTOCOL     *This,
  IN EFI_SPI_REQUEST_PACKET        *RequestPacket
  )
{
  FSPI_MASTER       *Fspi;
  FSPI_REQUEST      Request;
  EFI_STATUS        Status;

  if (!RequestPacket || !This || !(RequestPacket->TransactionCount)) {
    return EFI_INVALID_PARAMETER;
  }

  Fspi = BASE_CR (This, FSPI_MASTER, FspiHcProtocol);

  SetMem (&Request, sizeof(FSPI_REQUEST), 0);
  Request.LutId = SEQ_ID_IP_MODE;

  Status = ParseRequest (Fspi, RequestPacket, &Request);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Error in Parsing SPI request\n"));
    return Status;
  }
  ArmInstructionSynchronizationBarrier ();
  ArmDataSynchronizationBarrier ();

  if (Request.Type == SPI_REQUEST_WRITE_THEN_READ) {
    Status = ReadTransaction (Fspi, &Request);
  } else if (Request.Type == SPI_REQUEST_WRITE_ONLY) {
    Status = WriteTransaction (Fspi, &Request);
  } else {
    DEBUG ((DEBUG_ERROR, "SPI request not recognized\n"));
    Status = EFI_UNSUPPORTED;
  }

  return Status;
}

/**
  This routine is called by SPI bus layer to configure Host Controller from
  SpiIO.UpdateSpiPeripheral.

  Support socketed SPI parts by allowing the SPI peripheral driver to replace
  the SPI peripheral after the connection is made. An example use is socketed
  SPI NOR flash parts, where the size and parameters change depending upon
  device is in the socket.

  @param[in] This           Pointer to an EFI_SPI_HC_PROTOCOL structure.
  @param[in] SpiPeripheral  Pointer to a EFI_SPI_PERIPHERAL data structure from
                            which the routine can access the ConfigurationData.
			    The routine also has access to the names for the SPI bus and
                            chip which can be used during debugging.

  @retval EFI_SUCCESS       The SPI peripheral was updated successfully
  @retval EFI_DEVICE_ERROR  NOT able to update the SPI peripheral
  @retval EFI_INVALID_PARAMETER The ChipSeLect value or its contents are
                                invalid
**/
EFI_STATUS
FspiUpdateSpiPeripheral (
  IN CONST EFI_SPI_HC_PROTOCOL  *This,
  IN CONST EFI_SPI_PERIPHERAL   *SpiPeripheral
  )
{
  FSPI_MASTER                     *Fspi;
  FSPI_REGISTERS                  *Regs;
  UINT32                          ChipSelect;
  UINT32                          Index;
  SPI_FLASH_CONFIGURATION_DATA    *SpiConfigData;
  EFI_PHYSICAL_ADDRESS            CurrentChipSelectBaseAddress;
  UINT64                          FlashSize;

  if (!SpiPeripheral || !This || !(SpiPeripheral->ChipSelectParameter)) {
    return EFI_INVALID_PARAMETER;
  }

  Fspi = BASE_CR (This, FSPI_MASTER, FspiHcProtocol);
  Regs = Fspi->Regs;

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

  if (ChipSelect >= Fspi->NumChipselect) {
    return EFI_INVALID_PARAMETER;
  }

  SpiConfigData = (SPI_FLASH_CONFIGURATION_DATA *)(SpiPeripheral->ConfigurationData);
  if ((SpiConfigData == NULL) || (SpiConfigData->Signature != SPI_NOR_FLASH_SIGNATURE)) {
    return EFI_SUCCESS;
  }

  // Save Flash Size
  Fspi->Write32 (
          ( (UINTN)(&Regs->FlshA1Cr0 + ChipSelect)),
          ( ( (SpiConfigData->FlashSize + SIZE_1KB - 1) / SIZE_1KB) & FLSHCR0_FLSHSZ_MASK)
          );

  CurrentChipSelectBaseAddress = Fspi->AmbaBase;
  for (Index = FSPI_CHIP_SELECT_0; Index < ChipSelect; Index++) {
    FlashSize = Fspi->Read32 ( ( (UINTN)(&Regs->FlshA1Cr0 + Index))) & FLSHCR0_FLSHSZ_MASK;
    FlashSize *= SIZE_1KB;
    CurrentChipSelectBaseAddress += FlashSize;
  }

  SpiConfigData->DeviceBaseAddress = CurrentChipSelectBaseAddress;

  return EFI_SUCCESS;
}

/**
 Installs the SpiMaster Protocol and Device Path protocol on to that handle.

 Also this function registers for an Virtual Address change event, to convert the runtime
 memory allocated from physical address space to virtual address space.

 @param[in]   Fspi      Pointer to FSPI_MASTER strcture of a FSPI controller
 @param[in]   Runtime      Weather Fspi controller is to be used at runtime or not

 @retval EFI_DEVICE_ERROR      Not able to Install SPI Host Controller Protocol or if the Fspi
                               Controller is runtime, then not able to set the memory attributes for this controller.
 @retval EFI_SUCCESS           Protocols installed successfully on FSPI controllers' handles.
**/
EFI_STATUS
EFIAPI
FspiInstallProtocol (
  IN  FSPI_MASTER  *Fspi,
  IN  BOOLEAN          Runtime
  )
{
  EFI_STATUS Status;

  // Install SPI Host controller protocol and Device Path Protocol
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &Fspi->Handle,
                  &gEfiSpiHcProtocolGuid, &Fspi->FspiHcProtocol,
                  &gEfiDevicePathProtocolGuid, &Fspi->DevicePath,
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a:%d Error = %d\n", __FUNCTION__, __LINE__, Status));
    return Status;
  }

  if (Runtime) {
    // Declare the controller registers as EFI_MEMORY_RUNTIME
    Status = gDS->AddMemorySpace (
                    EfiGcdMemoryTypeMemoryMappedIo,
                    (UINTN)Fspi->Regs,
                    ALIGN_VALUE (sizeof (FSPI_REGISTERS), SIZE_64KB),
                    EFI_MEMORY_UC | EFI_MEMORY_RUNTIME
                    );
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "%a:%d Error = %r\n", __FUNCTION__, __LINE__, Status));
      goto UninstallProtocol;
    }

    Status = gDS->SetMemorySpaceAttributes (
                    (UINTN)Fspi->Regs,
                    ALIGN_VALUE (sizeof (FSPI_REGISTERS), SIZE_64KB),
                    EFI_MEMORY_UC | EFI_MEMORY_RUNTIME
                    );
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "%a:%d Error = %r\n", __FUNCTION__, __LINE__, Status));
      goto RemoveMemorySpace;
    }

    //
    // Register for the virtual address change event
    //
    Status = gBS->CreateEventEx (
                    EVT_NOTIFY_SIGNAL,
                    TPL_NOTIFY,
                    FspiVirtualNotifyEvent,
                    (VOID *)Fspi,
                    &gEfiEventVirtualAddressChangeGuid,
                    &Fspi->Event
                    );
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "%a:%d Error = %r\n", __FUNCTION__, __LINE__, Status));
      goto RemoveMemorySpace;
    }

    // Connect the controller Recursively to SPI bus
    Status = gBS->ConnectController (Fspi->Handle, NULL, NULL, TRUE);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "%a:%d Error = %d\n", __FUNCTION__, __LINE__, Status));
      goto RemoveEvent;
    }
  }

  return Status;

RemoveEvent:
  Status = gBS->CloseEvent (Fspi->Event);
  ASSERT_EFI_ERROR (Status);

RemoveMemorySpace:
  Status = gDS->RemoveMemorySpace (
                  (UINTN)Fspi->Regs,
                  ALIGN_VALUE (sizeof (FSPI_REGISTERS), SIZE_64KB)
                  );
  ASSERT_EFI_ERROR (Status);

UninstallProtocol:
  Status = gBS->UninstallMultipleProtocolInterfaces (
                  Fspi->Handle,
                  &gEfiSpiHcProtocolGuid, &Fspi->FspiHcProtocol,
                  &gEfiDevicePathProtocolGuid, &Fspi->DevicePath,
                  NULL
                  );
  ASSERT_EFI_ERROR (Status);

  return EFI_DEVICE_ERROR;
}

/**
 This function reads the device tree file from FV and pass on to device tree parser.
 device tree parser function parses the device tree and for each FSPI controller node found,
 allocates runtime memory for the internal data structure containing a handle and installs
 the SpiMaster Protocol and Device Path protocol on to that handle.

 Also this function registers for an Virtual Address change event, to convert the runtime
 memory allocated from physical address space to virtual address space.

 @param[in]   ImageHandle      Handle of FSPI driver efi image
 @param[in]   SystemTable      System Table Pointer

 @retval EFI_OUT_OF_RESOURCES  No pool memory available
 @retval EFI_CRC_ERROR         Device tree is not correct.
 @retval EFI_NOT_FOUND         No usable FSPI controller found.
 @retval EFI_SUCCESS           Protocols installed successfully on FSPI controllers' handles.
**/
EFI_STATUS
EFIAPI
FspiInitialise (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  UINTN                 FspiCount;
  VOID                  *Dtb;
  EFI_STATUS            Status;

  Status = EfiGetSystemConfigurationTable (&gFdtTableGuid, &Dtb);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Did not find the Dtb Blob.\n"));
    return Status;
  }

  Status = ParseDeviceTree (Dtb, &FspiCount);
  if (EFI_ERROR (Status) && (FspiCount == 0)) {
    return EFI_NOT_FOUND;
  }

  return EFI_SUCCESS;
}
