/** I2cDxe.c
  I2c driver APIs for read, write, initialize, set speed and reset

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/DebugLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeLib.h>

#include <Protocol/I2cMaster.h>

#include "I2cDxe.h"

STATIC EFI_EVENT VirtualAddressChangeEvent;

STATIC EFI_PHYSICAL_ADDRESS mI2cRegs;
STATIC CONST UINT16 ClkDiv[60][2] = {
  { 20,  0x00 }, { 22, 0x01 },  { 24, 0x02 },  { 26, 0x03 },
  { 28,  0x04 }, { 30,  0x05 }, { 32,  0x09 }, { 34, 0x06 },
  { 36,  0x0A }, { 40, 0x07 },  { 44, 0x0C },  { 48, 0x0D },
  { 52,  0x43 }, { 56,  0x0E }, { 60, 0x45 },  { 64, 0x12 },
  { 68,  0x0F }, { 72,  0x13 }, { 80,  0x14 }, { 88,  0x15 },
  { 96,  0x19 }, { 104, 0x16 }, { 112, 0x1A }, { 128, 0x17 },
  { 136, 0x4F }, { 144, 0x1C }, { 160, 0x1D }, { 176, 0x55 },
  { 192, 0x1E }, { 208, 0x56 }, { 224, 0x22 }, { 228, 0x24 },
  { 240, 0x1F }, { 256, 0x23 }, { 288, 0x5C }, { 320, 0x25 },
  { 384, 0x26 }, { 448, 0x2A }, { 480, 0x27 }, { 512, 0x2B },
  { 576, 0x2C }, { 640, 0x2D }, { 768, 0x31 }, { 896, 0x32 },
  { 960, 0x2F }, { 1024, 0x33 }, { 1152, 0x34 },{ 1280, 0x35 },
  { 1536, 0x36 }, { 1792, 0x3A }, { 1920, 0x37 }, { 2048, 0x3B },
  { 2304, 0x3C }, { 2560, 0x3D }, { 3072, 0x3E }, { 3584, 0x7A },
  { 3840, 0x3F }, { 4096, 0x7B }, { 5120, 0x7D }, { 6144, 0x7E },
};

/**
  Calculate and return proper clock divider

  @param  Rate       clock rate

  @retval ClkDiv     Value used to get frequency divider value

**/
STATIC
UINT8
GetClkDiv (
  IN  UINT32         Rate
  )
{
  UINTN              ClkRate;
  UINT32             Div;
  UINT8              ClkDivx;

  ClkRate = GetBusFrequency ();

  Div = (ClkRate + Rate - 1) / Rate;

  if (Div < ClkDiv[0][0]) {
    ClkDivx = 0;
  } else if (Div > ClkDiv[ARRAY_SIZE (ClkDiv) - 1][0]){
    ClkDivx = ARRAY_SIZE (ClkDiv) - 1;
  } else {
    for (ClkDivx = 0; ClkDiv[ClkDivx][0] < Div; ClkDivx++);
  }

  return ClkDivx;
}

/**
  Function used to check if i2c is in mentioned state or not

  @param   I2cRegs        Pointer to I2C registers
  @param   State          i2c state need to be checked

  @retval  EFI_NOT_READY  Arbitration was lost
  @retval  EFI_TIMEOUT    Timeout occured
  @retval  CurrState      Value of state register

**/
STATIC
EFI_STATUS
WaitForI2cState (
  IN  I2C_REGS            *I2cRegs,
  IN  UINT32              State
  )
{
  UINT8                   CurrState;
  UINT64                  Cnt;

  for (Cnt = 0; Cnt < 50000; Cnt++) {
    MemoryFence ();
    CurrState = MmioRead8 ((UINTN)&I2cRegs->I2cSr);
    if (CurrState & I2C_SR_IAL) {
       MmioWrite8 ((UINTN)&I2cRegs->I2cSr, CurrState | I2C_SR_IAL);
        return EFI_NOT_READY;
    }

    if ((CurrState & (State >> 8)) == (UINT8)State) {
      return CurrState;
    }
  }

  return EFI_TIMEOUT;
}

/**
  Function to transfer byte on i2c

  @param   I2cRegs        Pointer to i2c registers
  @param   Byte           Byte to be transferred on i2c bus

  @retval  EFI_NOT_READY  Arbitration was lost
  @retval  EFI_TIMEOUT    Timeout occured
  @retval  EFI_NOT_FOUND  ACK was not recieved
  @retval  EFI_SUCCESS    Data transfer was succesful

**/
STATIC
EFI_STATUS
TransferByte (
  IN  I2C_REGS            *I2cRegs,
  IN  UINT8               Byte
  )
{
  EFI_STATUS              Ret;

  MmioWrite8 ((UINTN)&I2cRegs->I2cSr, I2C_SR_IIF_CLEAR);
  MmioWrite8 ((UINTN)&I2cRegs->I2cDr, Byte);

  Ret = WaitForI2cState (I2cRegs, IIF);
  if ((Ret == EFI_TIMEOUT) || (Ret == EFI_NOT_READY)) {
    return Ret;
  }

  if (Ret & I2C_SR_RX_NO_AK) {
    return EFI_NOT_FOUND;
  }

  return EFI_SUCCESS;
}

/**
  Function to stop transaction on i2c bus

  @param   I2cRegs          Pointer to i2c registers

  @retval  EFI_NOT_READY    Arbitration was lost
  @retval  EFI_TIMEOUT      Timeout occured
  @retval  EFI_SUCCESS      Stop operation was successful

**/
STATIC
EFI_STATUS
I2cStop (
  IN  I2C_REGS             *I2cRegs
  )
{
  INT32                    Ret;
  UINT32                   Temp;

  Temp = MmioRead8 ((UINTN)&I2cRegs->I2cCr);

  Temp &= ~(I2C_CR_MSTA | I2C_CR_MTX);
  MmioWrite8 ((UINTN)&I2cRegs->I2cCr, Temp);

  Ret = WaitForI2cState (I2cRegs, BUS_IDLE);

  if (Ret < 0) {
    return Ret;
  } else {
    return EFI_SUCCESS;
  }
}

/**
  Function to send start signal, Chip Address and
  memory offset

  @param   I2cRegs         Pointer to i2c base registers
  @param   Chip            Chip Address
  @param   Offset          Slave memory's offset
  @param   Alen            length of chip address

  @retval  EFI_NOT_READY   Arbitration lost
  @retval  EFI_TIMEOUT     Failed to initialize data transfer in predefined time
  @retval  EFI_NOT_FOUND   ACK was not recieved
  @retval  EFI_SUCCESS     Read was successful

**/
STATIC
EFI_STATUS
InitTransfer (
  IN  I2C_REGS             *I2cRegs,
  IN  UINT8                Chip,
  IN  UINT32               Offset,
  IN  INT32                Alen
  )
{
  UINT32                   Temp;
  EFI_STATUS               Ret;

  // Enable I2C controller
  if (MmioRead8 ((UINTN)&I2cRegs->I2cCr) & I2C_CR_IDIS) {
    MmioWrite8 ((UINTN)&I2cRegs->I2cCr, I2C_CR_IEN);
  }

  if (MmioRead8 ((UINTN)&I2cRegs->I2cAdr) == (Chip << 1)) {
    MmioWrite8 ((UINTN)&I2cRegs->I2cAdr, (Chip << 1) ^ 2);
  }

  MmioWrite8 ((UINTN)&I2cRegs->I2cSr, I2C_SR_IIF_CLEAR);
  Ret = WaitForI2cState (I2cRegs, BUS_IDLE);
  if ((Ret == EFI_TIMEOUT) || (Ret == EFI_NOT_READY)) {
    return Ret;
  }

  // Start I2C transaction
  Temp = MmioRead8 ((UINTN)&I2cRegs->I2cCr);
  // set to master mode
  Temp |= I2C_CR_MSTA;
  MmioWrite8 ((UINTN)&I2cRegs->I2cCr, Temp);

  Ret = WaitForI2cState (I2cRegs, BUS_BUSY);
  if ((Ret == EFI_TIMEOUT) || (Ret == EFI_NOT_READY)) {
    return Ret;
  }

  Temp |= I2C_CR_MTX | I2C_CR_TX_NO_AK;
  MmioWrite8 ((UINTN)&I2cRegs->I2cCr, Temp);

  // write slave Address
  Ret = TransferByte (I2cRegs, Chip << 1);
  if (Ret != EFI_SUCCESS) {
    return Ret;
  }

  if (Alen >= 0) {
    while (Alen--) {
      Ret = TransferByte (I2cRegs, (Offset >> (Alen * 8)) & 0xff);
      if (Ret != EFI_SUCCESS)
        return Ret;
    }
  }
  return EFI_SUCCESS;
}

/**
  Function to check if i2c bus is idle

  @param   Base          Pointer to base address of I2c controller

  @retval  EFI_SUCCESS

**/
STATIC
INT32
I2cBusIdle (
  IN  VOID               *Base
  )
{
  return EFI_SUCCESS;
}

/**
  Function to initiate data transfer on i2c bus

  @param   I2cRegs         Pointer to i2c base registers
  @param   Chip            Chip Address
  @param   Offset          Slave memory's offset
  @param   Alen            length of chip address

  @retval  EFI_NOT_READY   Arbitration lost
  @retval  EFI_TIMEOUT     Failed to initialize data transfer in predefined time
  @retval  EFI_NOT_FOUND   ACK was not recieved
  @retval  EFI_SUCCESS     Read was successful

**/
STATIC
EFI_STATUS
InitDataTransfer (
  IN  I2C_REGS             *I2cRegs,
  IN  UINT8                Chip,
  IN  UINT32               Offset,
  IN  INT32                Alen
  )
{
  EFI_STATUS               Status;
  INT32                    Retry;

  for (Retry = 0; Retry < 3; Retry++) {
    Status = InitTransfer (I2cRegs, Chip, Offset, Alen);
    if (Status == EFI_SUCCESS) {
      return EFI_SUCCESS;
    }

    I2cStop (I2cRegs);

    if (EFI_NOT_FOUND == Status) {
      return Status;
    }

    // Disable controller
    if (Status != EFI_NOT_READY) {
      MmioWrite8 ((UINTN)&I2cRegs->I2cCr, I2C_CR_IDIS);
    }

    if (I2cBusIdle (I2cRegs) < 0) {
      break;
    }
  }
  return Status;
}

/**
  Function to read data using i2c bus

  @param   I2cBus          I2c Controller number
  @param   Chip            Address of slave device from where data to be read
  @param   Offset          Offset of slave memory
  @param   Alen            Address length of slave
  @param   Buffer          A pointer to the destination buffer for the data
  @param   Len             Length of data to be read

  @retval  EFI_NOT_READY   Arbitration lost
  @retval  EFI_TIMEOUT     Failed to initialize data transfer in predefined time
  @retval  EFI_NOT_FOUND   ACK was not recieved
  @retval  EFI_SUCCESS     Read was successful

**/
STATIC
EFI_STATUS
I2cDataRead (
  IN  UINT32               I2cBus,
  IN  UINT8                Chip,
  IN  UINT32               Offset,
  IN  UINT32               Alen,
  IN  UINT8                *Buffer,
  IN  UINT32               Len
  )
{
  EFI_STATUS               Status;
  UINT32                   Temp;
  INT32                    I;
  I2C_REGS                 *I2cRegs;

  I2cRegs = (I2C_REGS *)mI2cRegs;

  Status = InitDataTransfer (I2cRegs, Chip, Offset, Alen);
  if (Status != EFI_SUCCESS) {
    return Status;
  }

  Temp = MmioRead8 ((UINTN)&I2cRegs->I2cCr);
  Temp |= I2C_CR_RSTA;
  MmioWrite8 ((UINTN)&I2cRegs->I2cCr, Temp);

  Status = TransferByte (I2cRegs, (Chip << 1) | 1);
  if (Status != EFI_SUCCESS) {
    I2cStop (I2cRegs);
    return Status;
  }

  // setup bus to read data
  Temp = MmioRead8 ((UINTN)&I2cRegs->I2cCr);
  Temp &= ~(I2C_CR_MTX | I2C_CR_TX_NO_AK);
  if (Len == 1) {
    Temp |= I2C_CR_TX_NO_AK;
  }

  MmioWrite8 ((UINTN)&I2cRegs->I2cCr, Temp);
  MmioWrite8 ((UINTN)&I2cRegs->I2cSr, I2C_SR_IIF_CLEAR);

  // Dummy Read to initiate recieve operation
  MmioRead8 ((UINTN)&I2cRegs->I2cDr);

  for (I = 0; I < Len; I++) {
    Status = WaitForI2cState (I2cRegs, IIF);
    if ((Status == EFI_TIMEOUT) || (Status == EFI_NOT_READY)) {
       I2cStop (I2cRegs);
       return Status;
    }
    //
    // It must generate STOP before read I2DR to prevent
    // controller from generating another clock cycle
    //
    if (I == (Len - 1)) {
      I2cStop (I2cRegs);
    } else if (I == (Len - 2)) {
      Temp = MmioRead8 ((UINTN)&I2cRegs->I2cCr);
      Temp |= I2C_CR_TX_NO_AK;
      MmioWrite8 ((UINTN)&I2cRegs->I2cCr, Temp);
    }
    MmioWrite8 ((UINTN)&I2cRegs->I2cSr, I2C_SR_IIF_CLEAR);
    Buffer[I] = MmioRead8 ((UINTN)&I2cRegs->I2cDr);
  }

  I2cStop (I2cRegs);

  return EFI_SUCCESS;
}

/**
  Function to write data using i2c bus

  @param   I2cBus          I2c Controller number
  @param   Chip            Address of slave device where data to be written
  @param   Offset          Offset of slave memory
  @param   Alen            Address length of slave
  @param   Buffer          A pointer to the source buffer for the data
  @param   Len             Length of data to be write

  @retval  EFI_NOT_READY   Arbitration lost
  @retval  EFI_TIMEOUT     Failed to initialize data transfer in predefined time
  @retval  EFI_NOT_FOUND   ACK was not recieved
  @retval  EFI_SUCCESS     Read was successful

**/
STATIC
EFI_STATUS
I2cDataWrite (
  IN  UINT32               I2cBus,
  IN  UINT8                Chip,
  IN  UINT32               Offset,
  IN  INT32                Alen,
  OUT UINT8                *Buffer,
  IN  INT32                Len
  )
{
  EFI_STATUS               Status;
  I2C_REGS                 *I2cRegs;
  INT32                    I;

  I2cRegs = (I2C_REGS *)mI2cRegs;

  Status = InitDataTransfer (I2cRegs, Chip, Offset, Alen);
  if (Status != EFI_SUCCESS) {
    return Status;
  }

  // Write operation
  for (I = 0; I < Len; I++) {
    Status = TransferByte (I2cRegs, Buffer[I]);
    if (Status != EFI_SUCCESS) {
      break;
    }
  }

  I2cStop (I2cRegs);
  return Status;
}

/**
  Function to set i2c bus frequency

  @param   This            Pointer to I2c master protocol
  @param   BusClockHertz   value to be set

  @retval EFI_SUCCESS      Operation successfull
**/

EFI_STATUS
EFIAPI
SetBusFrequency (
  IN CONST EFI_I2C_MASTER_PROTOCOL   *This,
  IN OUT UINTN                       *BusClockHertz
 )
{
  I2C_REGS                 *I2cRegs;
  UINT8                    ClkId;
  UINT8                    SpeedId;

  I2cRegs = (I2C_REGS *)mI2cRegs;

  ClkId = GetClkDiv (*BusClockHertz);
  SpeedId = ClkDiv[ClkId][1];

  // Store divider value
  MmioWrite8 ((UINTN)&I2cRegs->I2cFdr, SpeedId);

  MemoryFence ();

  return EFI_SUCCESS;
}

/**
  Function to reset I2c Controller

  @param  This             Pointer to I2c master protocol

  @return EFI_SUCCESS      Operation successfull
**/
EFI_STATUS
EFIAPI
Reset (
  IN CONST EFI_I2C_MASTER_PROTOCOL *This
  )
{
  I2C_REGS                         *I2cRegs;

  I2cRegs = (I2C_REGS *)mI2cRegs;

  // Reset module
  MmioWrite8 ((UINTN)&I2cRegs->I2cCr, I2C_CR_IDIS);
  MmioWrite8 ((UINTN)&I2cRegs->I2cSr, 0);

  MemoryFence ();

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
StartRequest (
  IN CONST EFI_I2C_MASTER_PROTOCOL *This,
  IN UINTN                         SlaveAddress,
  IN EFI_I2C_REQUEST_PACKET        *RequestPacket,
  IN EFI_EVENT                     Event            OPTIONAL,
  OUT EFI_STATUS                   *I2cStatus       OPTIONAL
  )
{
  UINT32                           Count;
  INT32                            Ret;
  UINT32                           Length;
  UINT8                            *Buffer;
  UINT32                           Flag;
  UINT32                           RegAddress;
  UINT32                           OffsetLength;

  RegAddress = 0;

  if (RequestPacket->OperationCount <= 0) {
    BOOTTIME_DEBUG ((DEBUG_ERROR,"%a: Operation count is not valid %d\n",
           __FUNCTION__, RequestPacket->OperationCount));
    return EFI_INVALID_PARAMETER;
  }

  OffsetLength = RequestPacket->Operation[0].LengthInBytes;
  RegAddress = *RequestPacket->Operation[0].Buffer;

  for (Count = 1; Count < RequestPacket->OperationCount; Count++) {
    Flag = RequestPacket->Operation[Count].Flags;
    Length = RequestPacket->Operation[Count].LengthInBytes;
    Buffer = RequestPacket->Operation[Count].Buffer;

    if (Length <= 0) {
      BOOTTIME_DEBUG ((DEBUG_ERROR,"%a: Invalid length of buffer %d\n",
             __FUNCTION__, Length));
      return EFI_INVALID_PARAMETER;
    }

    if (Flag == I2C_FLAG_READ) {
      Ret = I2cDataRead (PcdGet32 (PcdI2cBus), SlaveAddress,
              RegAddress, OffsetLength, Buffer, Length);
      if (Ret != EFI_SUCCESS) {
        BOOTTIME_DEBUG ((DEBUG_ERROR,"%a: I2c read operation failed (error %d)\n",
               __FUNCTION__, Ret));
        return Ret;
      }
    } else if (Flag == I2C_FLAG_WRITE) {
      Ret = I2cDataWrite (PcdGet32 (PcdI2cBus), SlaveAddress,
              RegAddress, OffsetLength, Buffer, Length);
      if (Ret != EFI_SUCCESS) {
        BOOTTIME_DEBUG ((DEBUG_ERROR,"%a: I2c write operation failed (error %d)\n",
               __FUNCTION__, Ret));
        return Ret;
      }
    } else {
      BOOTTIME_DEBUG ((DEBUG_ERROR,"%a: Invalid Flag %d\n",
             __FUNCTION__, Flag));
      return EFI_INVALID_PARAMETER;
    }
  }

  return EFI_SUCCESS;
}

CONST EFI_I2C_CONTROLLER_CAPABILITIES I2cControllerCapabilities = {
  0,
  0,
  0,
  0
};

STATIC EFI_I2C_MASTER_PROTOCOL gI2c = {
  ///
  /// Set the clock frequency for the I2C bus.
  ///
  SetBusFrequency,
  ///
  /// Reset the I2C host controller.
  ///
  Reset,
  ///
  /// Start an I2C transaction in master mode on the host controller.
  ///
  StartRequest,
  ///
  /// Pointer to an EFI_I2C_CONTROLLER_CAPABILITIES data structure containing
  /// the capabilities of the I2C host controller.
  ///
  &I2cControllerCapabilities
};

STATIC I2C_DEVICE_PATH gDevicePath = {
  {
    {
      HARDWARE_DEVICE_PATH, HW_VENDOR_DP,
      {
        sizeof (VENDOR_DEVICE_PATH), 0
      }
    },
    EFI_CALLER_ID_GUID
  },
  {
    END_DEVICE_PATH_TYPE,
    END_ENTIRE_DEVICE_PATH_SUBTYPE,
    {
      sizeof (EFI_DEVICE_PATH_PROTOCOL), 0
    }
  }
};

/**
  Fixup controller regs data so that EFI can be call in virtual mode

  @param[in]    Event   The Event that is being processed
  @param[in]    Context Event Context
**/
STATIC
VOID
EFIAPI
I2cVirtualNotifyEvent (
  IN EFI_EVENT        Event,
  IN VOID             *Context
  )
{

  EfiConvertPointer (0x0, (VOID **)&mI2cRegs);
}

/**
  The Entry Point for I2C driver.

  @param[in] ImageHandle    The firmware allocated handle for the EFI image.
  @param[in] SystemTable    A pointer to the EFI System Table.

  @retval EFI_SUCCESS       The entry point is executed successfully.
  @retval other             Some error occurs when executing this entry point.

**/
EFI_STATUS
EFIAPI
I2cDxeEntryPoint (
  IN EFI_HANDLE             ImageHandle,
  IN EFI_SYSTEM_TABLE       *SystemTable
  )
{
  EFI_STATUS                Status;


  mI2cRegs = ( EFI_PHYSICAL_ADDRESS)(FixedPcdGet64 (PcdI2c0BaseAddr +
                         (PcdGet32 (PcdI2cBus) * FixedPcdGet32 (PcdI2cSize))));
  //
  // Install I2c Master protocol on this controller
  //
  Status = gBS->InstallMultipleProtocolInterfaces (
                &ImageHandle,
                &gEfiI2cMasterProtocolGuid,
                (VOID**)&gI2c,
                &gEfiDevicePathProtocolGuid,
                &gDevicePath,
                NULL
                );

  // Declare the controller as EFI_MEMORY_RUNTIME
  Status = gDS->AddMemorySpace (
                  EfiGcdMemoryTypeMemoryMappedIo,
                  (EFI_PHYSICAL_ADDRESS)mI2cRegs,
                  (SIZE_64KB),
                  EFI_MEMORY_UC | EFI_MEMORY_RUNTIME
                );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = gDS->SetMemorySpaceAttributes (
                  (EFI_PHYSICAL_ADDRESS)mI2cRegs,
                   (SIZE_64KB),
                  EFI_MEMORY_UC | EFI_MEMORY_RUNTIME
                );

  ASSERT_EFI_ERROR (Status);

    //
    // Register for the virtual address change event
    //
    Status = gBS->CreateEventEx (EVT_NOTIFY_SIGNAL, TPL_NOTIFY,
                    I2cVirtualNotifyEvent, NULL,
                    &gEfiEventVirtualAddressChangeGuid,
                    &VirtualAddressChangeEvent);

  return Status;
}

/**
  Unload function for the I2c Driver.

  @param  ImageHandle[in]        The allocated handle for the EFI image

  @retval EFI_SUCCESS            The driver was unloaded successfully
  @retval EFI_INVALID_PARAMETER  ImageHandle is not a valid image handle.

**/
EFI_STATUS
EFIAPI
I2cDxeUnload (
  IN EFI_HANDLE                  ImageHandle
  )
{
  EFI_STATUS                     Status;
  EFI_HANDLE                     *HandleBuffer;
  UINTN                          HandleCount;
  UINTN                          Index;

  //
  // Retrieve all I2c handles in the handle database
  //
  Status = gBS->LocateHandleBuffer (ByProtocol,
                                    &gEfiI2cMasterProtocolGuid,
                                    NULL,
                                    &HandleCount,
                                    &HandleBuffer);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  //
  // Disconnect the driver from the handles in the handle database
  //
  for (Index = 0; Index < HandleCount; Index++) {
    Status = gBS->DisconnectController (HandleBuffer[Index],
                                        gImageHandle,
                                        NULL);
  }

  //
  // Free the handle array
  //
  gBS->FreePool (HandleBuffer);

  //
  // Uninstall protocols installed by the driver in its entrypoint
  //
  Status = gBS->UninstallMultipleProtocolInterfaces (ImageHandle,
                  &gEfiI2cMasterProtocolGuid, &gI2c,
                  &gEfiDevicePathProtocolGuid, &gDevicePath,
                  NULL);

  return Status;
}
