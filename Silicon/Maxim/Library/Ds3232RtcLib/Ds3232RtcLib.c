/** Ds3232RtcLib.c
*  Implement EFI RealTimeClock via RTC Lib for DS3232 RTC.
*
*  Based on RTC implementation available in
*  EmbeddedPkg/Library/TemplateRealTimeClockLib/RealTimeClockLib.c
*
*  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.<BR>
*  Copyright 2017 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#include <PiDxe.h>
#include <Base.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/RealTimeClockLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeLib.h>
#include <Protocol/I2cMaster.h>

#include "Ds3232Rtc.h"

STATIC VOID                       *mDriverEventRegistration;
STATIC EFI_I2C_MASTER_PROTOCOL    *mI2cMaster;
STATIC EFI_EVENT                  mRtcVirtualAddrChangeEvent;

/**
  Read RTC register.

  @param  SlaveDeviceAddress   Slave device address offset of RTC to be read.
  @param  RtcRegAddr           Register offset of RTC to be read.

  @retval                      Register Value read

**/
STATIC
UINT8
RtcRead (
  IN  UINT8                SlaveDeviceAddress,
  IN  UINT8                RtcRegAddr
  )
{
  RTC_I2C_REQUEST          Req;
  EFI_STATUS               Status;
  UINT8                    Val;

  Val = 0;

  Req.OperationCount = 2;

  Req.SetAddressOp.Flags = 0;
  Req.SetAddressOp.LengthInBytes = sizeof (RtcRegAddr);
  Req.SetAddressOp.Buffer = &RtcRegAddr;

  Req.GetSetDateTimeOp.Flags = I2C_FLAG_READ;
  Req.GetSetDateTimeOp.LengthInBytes = sizeof (Val);
  Req.GetSetDateTimeOp.Buffer = &Val;

  Status = mI2cMaster->StartRequest (mI2cMaster, SlaveDeviceAddress,
                                     (VOID *)&Req,
                                     NULL,  NULL);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "RTC read error at Addr:0x%x\n", RtcRegAddr));
  }

  return Val;
}

/**
  Write RTC register.

  @param  SlaveDeviceAddress   Slave device address offset of RTC to be read.
  @param  RtcRegAddr           Register offset of RTC to write.
  @param  Val                  Value to be written

**/
STATIC
VOID
RtcWrite (
  IN  UINT8                SlaveDeviceAddress,
  IN  UINT8                RtcRegAddr,
  IN  UINT8                Val
  )
{
  RTC_I2C_REQUEST          Req;
  EFI_STATUS               Status;

  Req.OperationCount = 2;

  Req.SetAddressOp.Flags = 0;
  Req.SetAddressOp.LengthInBytes = sizeof (RtcRegAddr);
  Req.SetAddressOp.Buffer = &RtcRegAddr;

  Req.GetSetDateTimeOp.Flags = 0;
  Req.GetSetDateTimeOp.LengthInBytes = sizeof (Val);
  Req.GetSetDateTimeOp.Buffer = &Val;

  Status = mI2cMaster->StartRequest (mI2cMaster, SlaveDeviceAddress,
                                     (VOID *)&Req,
                                     NULL,  NULL);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "RTC write error at Addr:0x%x\n", RtcRegAddr));
  }
}

/**
  Configure the MUX device connected to I2C.

  @param  RegValue               Value to write on mux device register address

**/
VOID
ConfigureMuxDevice (
  IN  UINT8                RegValue
  )
{
  RtcWrite (FixedPcdGet8 (PcdMuxDeviceAddress), FixedPcdGet8 (PcdMuxControlRegOffset), RegValue);
}

/**
  Returns the current time and date information, and the time-keeping capabilities
  of the hardware platform.

  @param  Time                  A pointer to storage to receive a snapshot of the current time.
  @param  Capabilities          An optional pointer to a buffer to receive the real time clock
                                device's capabilities.

  @retval EFI_SUCCESS           The operation completed successfully.
  @retval EFI_INVALID_PARAMETER Time is NULL.
  @retval EFI_DEVICE_ERROR      The time could not be retrieved due to hardware error.

**/
EFI_STATUS
EFIAPI
LibGetTime (
  OUT  EFI_TIME                 *Time,
  OUT  EFI_TIME_CAPABILITIES    *Capabilities
  )
{
  EFI_STATUS                    Status;
  UINT8                         Second;
  UINT8                         Minute;
  UINT8                         Hour;
  UINT8                         Day;
  UINT8                         Month;
  UINT8                         Year;

  if (mI2cMaster == NULL) {
    return EFI_DEVICE_ERROR;
  }

  Status = EFI_SUCCESS;

  //
  // Check if the I2C device is connected though a MUX device.
  //
  if (FixedPcdGetBool (PcdIsRtcDeviceMuxed)) {
    // Switch to the channel connected to Ds3232 RTC
    ConfigureMuxDevice (FixedPcdGet8 (PcdMuxRtcChannelValue));
  }

  Second = RtcRead (FixedPcdGet8 (PcdI2cSlaveAddress), DS3232_SEC_REG_ADDR);
  Minute = RtcRead (FixedPcdGet8 (PcdI2cSlaveAddress), DS3232_MIN_REG_ADDR);
  Hour = RtcRead (FixedPcdGet8 (PcdI2cSlaveAddress), DS3232_HR_REG_ADDR);
  Day = RtcRead (FixedPcdGet8 (PcdI2cSlaveAddress), DS3232_DATE_REG_ADDR);
  Month = RtcRead (FixedPcdGet8 (PcdI2cSlaveAddress), DS3232_MON_REG_ADDR);
  Year = RtcRead (FixedPcdGet8 (PcdI2cSlaveAddress), DS3232_YR_REG_ADDR);

  if (Second & DS3232_SEC_BIT_CH) {
    DEBUG ((DEBUG_ERROR, "### Warning: RTC oscillator has stopped\n"));
    /* clear the CH flag */
    RtcWrite (FixedPcdGet8 (PcdI2cSlaveAddress), DS3232_SEC_REG_ADDR,
              RtcRead (FixedPcdGet8 (PcdI2cSlaveAddress), DS3232_SEC_REG_ADDR) & ~DS3232_SEC_BIT_CH);
    Status = EFI_DEVICE_ERROR;
    goto EXIT;
  }

  Time->Second  = BcdToDecimal8 (Second & MASK_SEC);
  Time->Minute  = BcdToDecimal8 (Minute & MASK_MIN);
  Time->Hour = BcdToDecimal8 (Hour & MASK_HOUR);
  Time->Day = BcdToDecimal8 (Day & MASK_DAY);
  Time->Month  = BcdToDecimal8 (Month & MASK_MONTH);

  //
  // RTC can save year 1970 to 2069
  // On writing Year, save year % 100
  // On Reading reversing the operation e.g. 2012
  // write = 12 (2012 % 100)
  // read = 2012 (12 + 2000)
  //
  Time->Year = BcdToDecimal8 (Year) +
               (BcdToDecimal8 (Year) >= 70 ? START_YEAR - 70 : END_YEAR -70);

EXIT:
  if (FixedPcdGetBool (PcdIsRtcDeviceMuxed)) {
    // Switch to the default channel
    ConfigureMuxDevice (FixedPcdGet8 (PcdMuxDefaultChannelValue));
  }

  return Status;
}

/**
  Sets the current local time and date information.

  @param  Time                  A pointer to the current time.

  @retval EFI_SUCCESS           The operation completed successfully.
  @retval EFI_INVALID_PARAMETER A time field is out of range.

**/
EFI_STATUS
EFIAPI
LibSetTime (
  IN  EFI_TIME                *Time
  )
{
  if (mI2cMaster == NULL) {
    return EFI_DEVICE_ERROR;
  }

  if (Time->Year < START_YEAR || Time->Year >= END_YEAR){
    DEBUG ((DEBUG_ERROR, "WARNING: Year should be between 1970 and 2069!\n"));
    return EFI_INVALID_PARAMETER;
  }

  //
  // Check if the I2C device is connected though a MUX device.
  //
  if (FixedPcdGetBool (PcdIsRtcDeviceMuxed)) {
    // Switch to the channel connected to Ds3232 RTC
    ConfigureMuxDevice (FixedPcdGet8 (PcdMuxRtcChannelValue));
  }

  RtcWrite (FixedPcdGet8 (PcdI2cSlaveAddress), DS3232_YR_REG_ADDR, DecimalToBcd8 (Time->Year % 100));
  RtcWrite (FixedPcdGet8 (PcdI2cSlaveAddress), DS3232_MON_REG_ADDR, DecimalToBcd8 (Time->Month));
  RtcWrite (FixedPcdGet8 (PcdI2cSlaveAddress), DS3232_DATE_REG_ADDR, DecimalToBcd8 (Time->Day));
  RtcWrite (FixedPcdGet8 (PcdI2cSlaveAddress), DS3232_HR_REG_ADDR, DecimalToBcd8 (Time->Hour));
  RtcWrite (FixedPcdGet8 (PcdI2cSlaveAddress), DS3232_MIN_REG_ADDR, DecimalToBcd8 (Time->Minute));
  RtcWrite (FixedPcdGet8 (PcdI2cSlaveAddress), DS3232_SEC_REG_ADDR, DecimalToBcd8 (Time->Second));

  if (FixedPcdGetBool (PcdIsRtcDeviceMuxed)) {
    // Switch to the default channel
    ConfigureMuxDevice (FixedPcdGet8 (PcdMuxDefaultChannelValue));
  }

  return EFI_SUCCESS;
}

/**
  Returns the current wakeup alarm clock setting.

  @param  Enabled               Indicates if the alarm is currently enabled or disabled.
  @param  Pending               Indicates if the alarm signal is pending and requires acknowledgement.
  @param  Time                  The current alarm setting.

  @retval EFI_SUCCESS           The alarm settings were returned.
  @retval EFI_INVALID_PARAMETER Any parameter is NULL.
  @retval EFI_DEVICE_ERROR      The wakeup time could not be retrieved due to a hardware error.

**/
EFI_STATUS
EFIAPI
LibGetWakeupTime (
  OUT  BOOLEAN                  *Enabled,
  OUT  BOOLEAN                  *Pending,
  OUT  EFI_TIME                 *Time
  )
{
  // Currently not supporting this feature.
  return EFI_UNSUPPORTED;
}

/**
  Sets the system wakeup alarm clock time.

  @param  Enabled               Enable or disable the wakeup alarm.
  @param  Time                  If Enable is TRUE, the time to set the wakeup alarm for.

  @retval EFI_SUCCESS           If Enable is TRUE, then the wakeup alarm was enabled. If
                                Enable is FALSE, then the wakeup alarm was disabled.
  @retval EFI_INVALID_PARAMETER A time field is out of range.
  @retval EFI_DEVICE_ERROR      The wakeup time could not be set due to a hardware error.
  @retval EFI_UNSUPPORTED       A wakeup timer is not supported on this platform.

**/
EFI_STATUS
EFIAPI
LibSetWakeupTime (
  IN BOOLEAN                    Enabled,
  OUT EFI_TIME                  *Time
  )
{
  // Currently not supporting this feature.
  return EFI_UNSUPPORTED;
}

/**
  Fixup internal data so that EFI can be call in virtual mode.
  Call the passed in Child Notify event and convert any pointers in
  lib to virtual mode.

  @param[in]    Event   The Event that is being processed
  @param[in]    Context Event Context
**/
VOID
EFIAPI
LibRtcVirtualNotifyEvent (
  IN EFI_EVENT        Event,
  IN VOID             *Context
  )
{
  EfiConvertPointer (0x0, (VOID **)&mI2cMaster);
}


STATIC
VOID
I2cDriverRegistrationEvent (
  IN  EFI_EVENT                 Event,
  IN  VOID                      *Context
  )
{
  EFI_STATUS                    Status;
  EFI_I2C_MASTER_PROTOCOL       *I2cMaster;
  UINTN                         BusFrequency;

  Status = gBS->LocateProtocol (&gEfiI2cMasterProtocolGuid, NULL, (VOID **)&I2cMaster);

  gBS->CloseEvent (Event);

  ASSERT_EFI_ERROR (Status);

  Status = I2cMaster->Reset (I2cMaster);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: I2CMaster->Reset () failed - %r\n",
      __FUNCTION__, Status));
    return;
  }

  BusFrequency = FixedPcdGet16 (PcdI2cBusFrequency);
  Status = I2cMaster->SetBusFrequency (I2cMaster, &BusFrequency);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: I2CMaster->SetBusFrequency () failed - %r\n",
      __FUNCTION__, Status));
    return;
  }

  mI2cMaster = I2cMaster;

  //
  // Register for the virtual address change event
  //
  Status = gBS->CreateEventEx (EVT_NOTIFY_SIGNAL, TPL_NOTIFY,
                  LibRtcVirtualNotifyEvent, NULL,
                  &gEfiEventVirtualAddressChangeGuid,
                  &mRtcVirtualAddrChangeEvent);
  ASSERT_EFI_ERROR (Status);
}

/**
  This is the declaration of an EFI image entry point. This can be the entry point to an application
  written to this specification, an EFI boot service driver.

  @param  ImageHandle           Handle that identifies the loaded image.
  @param  SystemTable           System Table for this image.

  @retval EFI_SUCCESS           The operation completed successfully.

**/
EFI_STATUS
EFIAPI
LibRtcInitialize (
  IN EFI_HANDLE                 ImageHandle,
  IN EFI_SYSTEM_TABLE           *SystemTable
  )
{
  //
  // Register a protocol registration notification callback on the driver
  // binding protocol so we can attempt to connect our I2C master to it
  // as soon as it appears.
  //
  EfiCreateProtocolNotifyEvent (
    &gEfiI2cMasterProtocolGuid,
    TPL_CALLBACK,
    I2cDriverRegistrationEvent,
    NULL,
    &mDriverEventRegistration);

  return EFI_SUCCESS;
}
