/** @PCF2129RtcLib.c
  Implement EFI RealTimeClock with runtime services via RTC Lib for PCF2129 RTC.

  Based on RTC implementation available in
  EmbeddedPkg/Library/TemplateRealTimeClockLib/RealTimeClockLib.c

  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.<BR>
  Copyright 2017, 2019 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

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

#include "Pcf2129Rtc.h"

STATIC EFI_I2C_MASTER_PROTOCOL    *mI2cMaster;
STATIC EFI_EVENT                  mRtcVirtualAddrChangeEvent;

/**
  returns Day of the week [0-6] 0=Sunday
  Don't try to provide a Year that's before 1998, please !
 **/
UINTN
EfiTimeToWday (
  IN  EFI_TIME  *Time
  )
{
  UINTN MonthDiff;
  UINTN Year;
  UINTN Month;
  UINTN JulianDate;  // Absolute Julian Date representation of the supplied Time
  UINTN EpochDays;   // Number of days elapsed since EPOCH_JULIAN_DAY

  MonthDiff = (14 - Time->Month) / 12 ;
  Year = Time->Year + 4800 - MonthDiff;
  Month = Time->Month + (12*MonthDiff) - 3;

  JulianDate = Time->Day + ((153*Month + 2)/5) + (365*Year) + (Year/4) - (Year/100) + (Year/400) - 32045;

  ASSERT (JulianDate >= EPOCH_JULIAN_DATE);
  EpochDays = JulianDate - EPOCH_JULIAN_DATE;

   // 4=1/1/1998 was a Thursday

  return (EpochDays + 4) % 7;
}

/**
  Write RTC register.

  @param  RtcRegAddr       Register offset of RTC to write.
  @param  Val              Value to be written

**/

STATIC
VOID
RtcWrite (
  IN  UINT8                RtcRegAddr,
  IN  UINT8                Val
  )
{
  RTC_I2C_REQUEST          Req;
  EFI_STATUS               Status;

  Req.OperationCount = 2;

  Req.SetAddressOp.Flags = 0;
  Req.SetAddressOp.LengthInBytes = 0;
  Req.SetAddressOp.Buffer = &RtcRegAddr;

  Req.GetSetDateTimeOp.Flags = 0;
  Req.GetSetDateTimeOp.LengthInBytes = sizeof (Val);
  Req.GetSetDateTimeOp.Buffer = &Val;

  Status = mI2cMaster->StartRequest (mI2cMaster, FixedPcdGet8 (PcdI2cSlaveAddress),
                                     (VOID *)&Req,
                                     NULL,  NULL);
  if (EFI_ERROR (Status)) {
    BOOTTIME_DEBUG ((DEBUG_ERROR, "RTC write error at Addr:0x%x\n", RtcRegAddr));
  }

}
/**
  Write RTC register.

  @param  SlaveDeviceAddress   Slave device address offset of RTC to be read.
  @param  RtcRegAddr           Register offset of RTC to write.
  @param  Val                  Value to be written

**/
STATIC
VOID
I2CWriteMux (
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
    BOOTTIME_DEBUG ((DEBUG_ERROR, "RTC write error at Addr:0x%x\n", RtcRegAddr));
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
  I2CWriteMux (FixedPcdGet8 (PcdMuxDeviceAddress), FixedPcdGet8 (PcdMuxControlRegOffset), RegValue);
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
  OUT EFI_TIME                *Time,
  OUT  EFI_TIME_CAPABILITIES  *Capabilities
  )
{
  EFI_STATUS      Status;
  UINT8           Buffer[10];
  RTC_I2C_REQUEST Req;
  UINT8           RtcRegAddr;

  Status = EFI_SUCCESS;
  RtcRegAddr = PCF2129_CTRL1_REG_ADDR;
  Buffer[0] = 0;

  if (mI2cMaster == NULL) {
    return EFI_DEVICE_ERROR;
  }

  //
  // Check if the I2C device is connected though a MUX device.
  //
  if (FixedPcdGetBool (PcdIsRtcDeviceMuxed)) {
    // Switch to the channel connected to Ds3232 RTC
    ConfigureMuxDevice (FixedPcdGet8 (PcdMuxRtcChannelValue));
  }

  RtcWrite (PCF2129_CTRL1_REG_ADDR, Buffer[0]);

  if (Time == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  Req.OperationCount = 2;

  Req.SetAddressOp.Flags = 0;
  Req.SetAddressOp.LengthInBytes = 0;
  Req.SetAddressOp.Buffer = &RtcRegAddr;

  Req.GetSetDateTimeOp.Flags = I2C_FLAG_READ;
  Req.GetSetDateTimeOp.LengthInBytes = sizeof (Buffer);
  Req.GetSetDateTimeOp.Buffer = Buffer;

  Status = mI2cMaster->StartRequest (mI2cMaster, FixedPcdGet8 (PcdI2cSlaveAddress),
                                     (VOID *)&Req,
                                     NULL,  NULL);
  if (EFI_ERROR (Status)) {
    BOOTTIME_DEBUG ((DEBUG_ERROR, "RTC read error at Addr:0x%x\n", RtcRegAddr));
  }

  if (Buffer[PCF2129_CTRL3_REG_ADDR] & PCF2129_CTRL3_BIT_BLF) {
    BOOTTIME_DEBUG ((DEBUG_INFO,
      "### Warning: RTC battery status low, check/replace RTC battery.\n"));
  }

  Time->Nanosecond = 0;
  Time->Second  = BcdToDecimal8 (Buffer[PCF2129_SEC_REG_ADDR] & 0x7F);
  Time->Minute  = BcdToDecimal8 (Buffer[PCF2129_MIN_REG_ADDR] & 0x7F);
  Time->Hour = BcdToDecimal8 (Buffer[PCF2129_HR_REG_ADDR] & 0x3F);
  Time->Day = BcdToDecimal8 (Buffer[PCF2129_DAY_REG_ADDR] & 0x3F);
  Time->Month  = BcdToDecimal8 (Buffer[PCF2129_MON_REG_ADDR] & 0x1F);
  Time->Year = BcdToDecimal8 (Buffer[PCF2129_YR_REG_ADDR]) + ( BcdToDecimal8 (Buffer[PCF2129_YR_REG_ADDR]) >= 98 ? 1900 : 2000);

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
  @retval EFI_DEVICE_ERROR      The time could not be set due due to hardware error.

**/

EFI_STATUS
EFIAPI
LibSetTime (
  IN EFI_TIME                *Time
  )
{
  UINT8           Buffer[8];
  UINT8           Index;
  EFI_STATUS      Status;
  RTC_I2C_REQUEST Req;
  UINT8           RtcRegAddr;

  Index = 0;
  Status = EFI_SUCCESS;
  RtcRegAddr = PCF2129_CTRL1_REG_ADDR;

  if (mI2cMaster == NULL) {
    return EFI_DEVICE_ERROR;
  }
  //
  // Check if the I2C device is connected though a MUX device.
  //
  if (FixedPcdGetBool (PcdIsRtcDeviceMuxed)) {
    // Switch to the channel connected to Ds3232 RTC
    ConfigureMuxDevice (FixedPcdGet8 (PcdMuxRtcChannelValue));
  }

  // start register address
  Buffer[Index++] = PCF2129_SEC_REG_ADDR;

  // hours, minutes and seconds
  Buffer[Index++] = DecimalToBcd8 (Time->Second);
  Buffer[Index++] = DecimalToBcd8 (Time->Minute);
  Buffer[Index++] = DecimalToBcd8 (Time->Hour);
  Buffer[Index++] = DecimalToBcd8 (Time->Day);
  Buffer[Index++] = EfiTimeToWday (Time) & 0x07;
  Buffer[Index++] = DecimalToBcd8 (Time->Month);
  Buffer[Index++] = DecimalToBcd8 (Time->Year % 100);

  Req.OperationCount = 2;
  Req.SetAddressOp.Flags = 0;
  Req.SetAddressOp.LengthInBytes = 0;
  Req.SetAddressOp.Buffer = &RtcRegAddr;

  Req.GetSetDateTimeOp.Flags = 0;
  Req.GetSetDateTimeOp.LengthInBytes = sizeof (Buffer);
  Req.GetSetDateTimeOp.Buffer = Buffer;

  Status = mI2cMaster->StartRequest (mI2cMaster, FixedPcdGet8 (PcdI2cSlaveAddress),
                                     (VOID *)&Req,
                                     NULL,  NULL);
  if (EFI_ERROR (Status)) {
    BOOTTIME_DEBUG ((DEBUG_ERROR, "RTC write error at Addr:0x%x\n", RtcRegAddr));
    return Status;
  }
    if (FixedPcdGetBool (PcdIsRtcDeviceMuxed)) {
    // Switch to the default channel
    ConfigureMuxDevice (FixedPcdGet8 (PcdMuxDefaultChannelValue));
  }

  return Status;
}


/**
  Returns the current wakeup alarm clock setting.

  @param  Enabled               Indicates if the alarm is currently enabled or disabled.
  @param  Pending               Indicates if the alarm signal is pending and requires acknowledgement.
  @param  Time                  The current alarm setting.

  @retval EFI_SUCCESS           The alarm settings were returned.
  @retval EFI_INVALID_PARAMETER Any parameter is NULL.
  @retval EFI_DEVICE_ERROR      The wakeup time could not be retrieved due to a hardware error.
  @retval EFI_UNSUPPORTED       A wakeup timer is not supported on this platform.

**/
EFI_STATUS
EFIAPI
LibGetWakeupTime (
  OUT BOOLEAN     *Enabled,
  OUT BOOLEAN     *Pending,
  OUT EFI_TIME    *Time
  )
{
  // Not a required feature
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
  IN BOOLEAN      Enabled,
  OUT EFI_TIME    *Time
  )
{
  // Not a required feature
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

/**
  This is the declaration of an EFI image entry point. This can be the entry point to an application
  written to this specification, an EFI boot service driver, or an EFI runtime driver.

  @param  ImageHandle           Handle that identifies the loaded image.
  @param  SystemTable           System Table for this image.

  @retval EFI_SUCCESS           The operation completed successfully.
  @retval EFI_DEVICE_ERROR      The operation could not be started.

**/
EFI_STATUS
EFIAPI
LibRtcInitialize (
  IN EFI_HANDLE                            ImageHandle,
  IN EFI_SYSTEM_TABLE                      *SystemTable
  )
{

  EFI_STATUS                    Status;
  EFI_I2C_MASTER_PROTOCOL       *I2cMaster;
  UINTN                         BusFrequency;

  Status = gBS->LocateProtocol (&gEfiI2cMasterProtocolGuid, NULL, (VOID **)&I2cMaster);

  ASSERT_EFI_ERROR (Status);

  Status = I2cMaster->Reset (I2cMaster);
  if (EFI_ERROR (Status)) {
    BOOTTIME_DEBUG ((DEBUG_ERROR, "%a: I2CMaster->Reset () failed - %r\n",
      __FUNCTION__, Status));
    return Status;
  }

  BusFrequency = FixedPcdGet32 (PcdI2cSpeed);
  Status = I2cMaster->SetBusFrequency (I2cMaster, &BusFrequency);
  if (EFI_ERROR (Status)) {
    BOOTTIME_DEBUG ((DEBUG_ERROR, "%a: I2CMaster->SetBusFrequency () failed - %r\n",
      __FUNCTION__, Status));
    return Status;
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

  return EFI_SUCCESS;
}
