/** @file

  Implement EFI RealTimeClock with runtime services for PCF85263 RTC.

  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <PiDxe.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/RealTimeClockLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/UefiRuntimeLib.h>
#include <Library/UefiLib.h>
#include <Protocol/I2cMaster.h>

#include "Pcf85263Rtc.h"

STATIC CONST CHAR16               mTimeZoneVariableName[] = L"Pcf85263RtcTimeZone";
STATIC CONST CHAR16               mDaylightVariableName[] = L"Pcf85263RtcDaylight";
STATIC EFI_EVENT                  mRtcVirtualAddrChangeEvent;
STATIC EFI_I2C_MASTER_PROTOCOL    *mI2cMaster;
STATIC VOID                       *mDriverEventRegistration;

/**
  Read RTC register.

  @param  RtcRegAddr       Register offset of RTC to be read.

  @retval                  Register Value read

**/

STATIC
UINT8
RtcRead (
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

  Status = mI2cMaster->StartRequest (mI2cMaster, FixedPcdGet8 (PcdI2cSlaveAddress),
                                     (VOID *)&Req,
                                     NULL,  NULL);
  if (EFI_ERROR (Status)) {
    BOOTTIME_DEBUG ((DEBUG_ERROR, "RTC read error at Addr:0x%x\n", RtcRegAddr));
  }

  return Val;
}

/**
  Write RTC register.

  @param  RtcRegAddr       Register offset of RTC to write.
  @param  Val              Value to be written

**/

STATIC
EFI_STATUS
RtcWrite (
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

  Status = mI2cMaster->StartRequest (mI2cMaster, FixedPcdGet8 (PcdI2cSlaveAddress),
                                     (VOID *)&Req,
                                     NULL,  NULL);
  if (EFI_ERROR (Status)) {
    BOOTTIME_DEBUG ((DEBUG_ERROR, "RTC write error at Addr:0x%x\n", RtcRegAddr));
  }

  return Status;
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
  //
  // Only needed if you are going to support the OS calling RTC functions in virtual mode.
  // You will need to call EfiConvertPointer (). To convert any stored physical addresses
  // to virtual address. After the OS transistions to calling in virtual mode, all future
  // runtime calls will be made in virtual mode.
  //
  EfiConvertPointer (0x0, (VOID **)&mI2cMaster);
}

STATIC
BOOLEAN
IsLeapYear (
  IN EFI_TIME         *Time
  )
{
  if (Time->Year % 4 == 0) {
    if (Time->Year % 100 == 0) {
      if (Time->Year % 400 == 0) {
        return TRUE;
      } else {
        return FALSE;
      }
    } else {
      return TRUE;
    }
  } else {
    return FALSE;
  }
}

STATIC
BOOLEAN
IsDayValid (
  IN  EFI_TIME        *Time
  )
{
  STATIC CONST INTN DayOfMonth[12] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

  if (Time->Day < 1 ||
      Time->Day > DayOfMonth[Time->Month - 1] ||
      (Time->Month == 2 && (!IsLeapYear (Time) && Time->Day > 28))
     ) {
    return FALSE;
  }

  return TRUE;
}

/**
  Returns the current time and date information, and the time-keeping capabilities
  of the hardware platform.

  @param  Time                  A pointer to storage to receive a snapshot of the current time.

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
  INT16                         TimeZone;
  UINT8                         Daylight;
  UINT8                         Buffer[8];
  UINTN                         Size;
  UINT8                         Val;
  RTC_I2C_REQUEST               Req;
  UINT8                         RtcRegAddr;

  if (mI2cMaster == NULL) {
    BOOTTIME_DEBUG ((DEBUG_ERROR, "%a : I2c Master protocol is not yet installed\n", __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }

  if (Time == NULL) {
    BOOTTIME_DEBUG ((DEBUG_ERROR, "Received NULL parameter\n"));
    return EFI_INVALID_PARAMETER;
  }

  RtcRegAddr = PCF85263_100SEC_REG_ADDR;

  Req.OperationCount = 2;

  Req.SetAddressOp.Flags = 0;
  Req.SetAddressOp.LengthInBytes = sizeof (RtcRegAddr);
  Req.SetAddressOp.Buffer = &RtcRegAddr;

  Req.GetSetDateTimeOp.Flags = I2C_FLAG_READ;
  Req.GetSetDateTimeOp.LengthInBytes = sizeof (Buffer);
  Req.GetSetDateTimeOp.Buffer = Buffer;

  Status = mI2cMaster->StartRequest (mI2cMaster, FixedPcdGet8 (PcdI2cSlaveAddress),
                                     (VOID *)&Req,
                                     NULL,  NULL);
  if (EFI_ERROR (Status)) {
    BOOTTIME_DEBUG ((DEBUG_ERROR, "RTC read error at Addr:0x%x\n", RtcRegAddr));
    return Status;
  }

  if (Buffer[PCF85263_SEC_REG_ADDR] & PCF85263_SEC_BIT_OSC) {
    BOOTTIME_DEBUG ((DEBUG_ERROR, "### Warning: RTC oscillator has stopped\n"));
    /* clear the OS flag */
    Val = Buffer[PCF85263_SEC_REG_ADDR] & ~PCF85263_SEC_BIT_OSC;
    RtcWrite (PCF85263_SEC_REG_ADDR, Val);

    return EFI_DEVICE_ERROR;
  }

  Time->Nanosecond = BcdToDecimal8 (Buffer[PCF85263_100SEC_REG_ADDR]);
  Time->Second  = BcdToDecimal8 (Buffer[PCF85263_SEC_REG_ADDR] & MASK_SEC);
  Time->Minute  = BcdToDecimal8 (Buffer[PCF85263_MIN_REG_ADDR] & MASK_MIN);
  Time->Hour = BcdToDecimal8 (Buffer[PCF85263_HR_REG_ADDR] & MASK_HOUR);
  Time->Day = BcdToDecimal8 (Buffer[PCF85263_DATE_REG_ADDR] & MASK_DAY);
  Time->Month  = BcdToDecimal8 (Buffer[PCF85263_MON_REG_ADDR] & MASK_MONTH);
  Time->Year = BcdToDecimal8 (Buffer[PCF85263_YR_REG_ADDR]) +
               (BcdToDecimal8 (Buffer[PCF85263_YR_REG_ADDR]) >= 98 ? 1900 : 2000);

  // Get the current time zone information from non-volatile storage
  Size = sizeof (TimeZone);
  Status = EfiGetVariable (
             (CHAR16 *)mTimeZoneVariableName,
             &gEfiCallerIdGuid,
             NULL,
             &Size,
             (VOID *)&TimeZone
             );

  if (EFI_ERROR (Status)) {
    ASSERT (Status != EFI_INVALID_PARAMETER);
    ASSERT (Status != EFI_BUFFER_TOO_SMALL);

    if (Status != EFI_NOT_FOUND) {
      return Status;
    }

    // The time zone variable does not exist in non-volatile storage, so create it.
    Time->TimeZone = EFI_UNSPECIFIED_TIMEZONE;
    // Store it
    Status = EfiSetVariable (
               (CHAR16 *)mTimeZoneVariableName,
               &gEfiCallerIdGuid,
               EFI_VARIABLE_NON_VOLATILE | EFI_VARIABLE_BOOTSERVICE_ACCESS | EFI_VARIABLE_RUNTIME_ACCESS,
               Size,
               (VOID *)&(Time->TimeZone)
               );

    if (EFI_ERROR (Status)) {
      BOOTTIME_DEBUG ((DEBUG_ERROR,
        "LibGetTime: Failed to save %s variable to non-volatile storage, Status = %d\n",
        mTimeZoneVariableName,
        Status
        ));
      return Status;
    }
  } else {
    // Got the time zone
    Time->TimeZone = TimeZone;

    // Check TimeZone bounds:   -1440 to 1440 or 2047
    if (((Time->TimeZone < -1440) || (Time->TimeZone > 1440))
        && (Time->TimeZone != EFI_UNSPECIFIED_TIMEZONE)) {
      Time->TimeZone = EFI_UNSPECIFIED_TIMEZONE;
    }
  }

  // Get the current daylight information from non-volatile storage
  Size = sizeof (Daylight);
  Status = EfiGetVariable (
             (CHAR16 *)mDaylightVariableName,
             &gEfiCallerIdGuid,
             NULL,
             &Size,
             (VOID *)&Daylight
             );

  if (EFI_ERROR (Status)) {
    ASSERT (Status != EFI_INVALID_PARAMETER);
    ASSERT (Status != EFI_BUFFER_TOO_SMALL);

    if (Status != EFI_NOT_FOUND) {
      return Status;
    }

    // The daylight variable does not exist in non-volatile storage, so create it.
    Time->Daylight = 0;
    // Store it
    Status = EfiSetVariable (
               (CHAR16 *)mDaylightVariableName,
               &gEfiCallerIdGuid,
               EFI_VARIABLE_NON_VOLATILE | EFI_VARIABLE_BOOTSERVICE_ACCESS | EFI_VARIABLE_RUNTIME_ACCESS,
               Size,
               (VOID *)&(Time->Daylight)
               );
    if (EFI_ERROR (Status)) {
      BOOTTIME_DEBUG ((DEBUG_ERROR,
        "LibGetTime: Failed to save %s variable to non-volatile storage, Status = %d\n",
        mDaylightVariableName,
        Status
        ));
      return Status;
    }
  } else {
    // Got the daylight information
    Time->Daylight = Daylight;
  }

  return Status;
}
/*
 * Returns Day of the week [0-6] 0=Sunday
 *
 * Don't try to provide a Year that's before 1998, please !
 */
STATIC
UINTN
EfiTimeToWday (
  IN  EFI_TIME  *Time
  )
{
  UINTN         a;
  UINTN         y;
  UINTN         m;
  UINTN         JulianDate;  // Absolute Julian Date representation of the supplied Time
  UINTN         EpochDays;   // Number of days elapsed since EPOCH_JULIAN_DAY

  a = (14 - Time->Month) / 12 ;
  y = Time->Year + 4800 - a;
  m = Time->Month + (12*a) - 3;

  JulianDate = Time->Day + ((153*m + 2)/5) + (365*y) + (y/4) - (y/100) + (y/400) - 32045;

  ASSERT (JulianDate >= EPOCH_JULIAN_DATE);
  EpochDays = JulianDate - EPOCH_JULIAN_DATE;

  /*
   * 4=1/1/1998 was a Thursday
   */
  return (EpochDays + 4) % 7;
}

STATIC
EFI_STATUS
StartRtc  (
  VOID
  )
{
  EFI_STATUS  Status;
  UINT8       Val;

  /* Clear STOP bit or start RTC */
  Val = START_RTC;
  Status = RtcWrite (PCF85263_STOPBIT_REG_ADDR, Val);

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
  EFI_STATUS                 Status;
  UINT8                      Buffer[8];
  UINT8                      Val;
  RTC_I2C_REQUEST            Req;
  UINT8                      RtcRegAddr;

  if (mI2cMaster == NULL) {
    BOOTTIME_DEBUG ((DEBUG_ERROR, "%a : I2c Master protocol is not yet installed\n", __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }

  // Check the input parameters are within the range specified by UEFI
  if ((Time->Year   < 1900) ||
       (Time->Year   > 9999) ||
       (Time->Month  < 1   ) ||
       (Time->Month  > 12  ) ||
       (!IsDayValid (Time)    ) ||
       (Time->Hour   > 23  ) ||
       (Time->Minute > 59  ) ||
       (Time->Second > 59  ) ||
       (Time->Nanosecond > 999999999) ||
       (!((Time->TimeZone == EFI_UNSPECIFIED_TIMEZONE) || ((Time->TimeZone >= -1440) && (Time->TimeZone <= 1440)))) ||
       (Time->Daylight & (~(EFI_TIME_ADJUST_DAYLIGHT | EFI_TIME_IN_DAYLIGHT)))
    ) {
    Status = EFI_INVALID_PARAMETER;
    return Status;
  }

  if (Time->Year < 1998 || Time->Year > 2097) {
    DEBUG((DEBUG_ERROR, "WARNING: Year should be between 1998 and 2097!\n"));
    Status = EFI_INVALID_PARAMETER;
    return Status;
  }

  /* hours, minutes and seconds */
  Buffer[PCF85263_100SEC_REG_ADDR] = DecimalToBcd8 (Time->Nanosecond);
  Buffer[PCF85263_SEC_REG_ADDR]    = DecimalToBcd8 (Time->Second);
  Buffer[PCF85263_MIN_REG_ADDR]    = DecimalToBcd8 (Time->Minute);
  Buffer[PCF85263_HR_REG_ADDR]     = DecimalToBcd8 (Time->Hour);
  Buffer[PCF85263_DATE_REG_ADDR]   = DecimalToBcd8 (Time->Day);
  Buffer[PCF85263_DAY_REG_ADDR]    = EfiTimeToWday (Time) & 0x07;
  Buffer[PCF85263_MON_REG_ADDR]    = DecimalToBcd8 (Time->Month);
  Buffer[PCF85263_YR_REG_ADDR]     = DecimalToBcd8 (Time->Year % 100);

  /* Send STOP to RTC to freeze time */
  Val = STOP_RTC;
  Status = RtcWrite (PCF85263_STOPBIT_REG_ADDR, Val);

  if (EFI_ERROR (Status)) {
     DEBUG((DEBUG_ERROR, "RTC failed to set STOP bit\n"));
     return Status;
  }

  /* Clear prescaler */
  Val = CLEAR_PRESCALER;
  Status = RtcWrite (PCF85263_RESET_REG_ADDR, Val);

  if (EFI_ERROR (Status)) {
     DEBUG((DEBUG_ERROR, "RTC error in clearing prescaler\n"));
     StartRtc();
     return Status;
  }

  /* Write Time */
  RtcRegAddr = PCF85263_100SEC_REG_ADDR;

  Req.OperationCount = 2;
  Req.SetAddressOp.Flags = 0;
  Req.SetAddressOp.LengthInBytes = sizeof (RtcRegAddr);
  Req.SetAddressOp.Buffer = &RtcRegAddr;

  Req.GetSetDateTimeOp.Flags = 0;
  Req.GetSetDateTimeOp.LengthInBytes = sizeof (Buffer);
  Req.GetSetDateTimeOp.Buffer = Buffer;

  Status = mI2cMaster->StartRequest (mI2cMaster, FixedPcdGet8 (PcdI2cSlaveAddress),
                                     (VOID *)&Req,
                                     NULL,  NULL);

  if (EFI_ERROR (Status)) {
     DEBUG((DEBUG_ERROR, "RTC write error at Addr:0x%x \n", RtcRegAddr));
     StartRtc();
     return Status;
  }

  Status = StartRtc();
  if (EFI_ERROR (Status)) {
     DEBUG((DEBUG_ERROR, "RTC failed to clear STOP bit\n"));
     return Status;
  }
  //
  // The accesses to Variable Services can be very slow, because we may be writing to Flash.
  // Do this after having set the RTC.
  //

  // Save the current time zone information into non-volatile storage
  Status = EfiSetVariable (
              (CHAR16 *)mTimeZoneVariableName,
              &gEfiCallerIdGuid,
              EFI_VARIABLE_NON_VOLATILE | EFI_VARIABLE_BOOTSERVICE_ACCESS | EFI_VARIABLE_RUNTIME_ACCESS,
              sizeof (Time->TimeZone),
              (VOID *)&(Time->TimeZone)
              );
  if (EFI_ERROR (Status)) {
      BOOTTIME_DEBUG ((DEBUG_ERROR,
        "LibSetTime: Failed to save %s variable to non-volatile storage, Status = %d\n",
        mTimeZoneVariableName,
        Status
        ));
    return Status;
  }

  // Save the current daylight information into non-volatile storage
  Status = EfiSetVariable (
             (CHAR16 *)mDaylightVariableName,
             &gEfiCallerIdGuid,
             EFI_VARIABLE_NON_VOLATILE | EFI_VARIABLE_BOOTSERVICE_ACCESS | EFI_VARIABLE_RUNTIME_ACCESS,
             sizeof (Time->Daylight),
             (VOID *)&(Time->Daylight)
             );
  if (EFI_ERROR (Status)) {
    BOOTTIME_DEBUG ((DEBUG_ERROR,
      "LibSetTime: Failed to save %s variable to non-volatile storage, Status = %d\n",
      mDaylightVariableName,
      Status
      ));
    return Status;
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
  Function to enable RTC counts at a resolution of 0.01 seconds.

  @retval EFI_SUCCESS           The operation completed successfully.

**/

STATIC
VOID
Enable100Sec  (
  IN  VOID
  )
{
  UINT8 Value;

  Value = RtcRead (PCF85263_FUNC_REG_ADDR);

  Value |= ENABLE_100SEC;

  RtcWrite (PCF85263_FUNC_REG_ADDR, Value);

  return;
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
    BOOTTIME_DEBUG ((DEBUG_ERROR, "%a: I2CMaster->Reset () failed - %r\n",
      __FUNCTION__, Status));
    return;
  }

  BusFrequency = FixedPcdGet16 (PcdI2cBusFrequency);
  Status = I2cMaster->SetBusFrequency (I2cMaster, &BusFrequency);
  if (EFI_ERROR (Status)) {
    BOOTTIME_DEBUG ((DEBUG_ERROR, "%a: I2CMaster->SetBusFrequency () failed - %r\n",
      __FUNCTION__, Status));
    return;
  }

  mI2cMaster = I2cMaster;


  /* Enable milisecond calculation */
  Enable100Sec ();

  //
  // Register for the virtual address change event
  //
  Status = gBS->CreateEventEx (
                  EVT_NOTIFY_SIGNAL,
                  TPL_NOTIFY,
                  LibRtcVirtualNotifyEvent,
                  NULL,
                  &gEfiEventVirtualAddressChangeGuid,
                  &mRtcVirtualAddrChangeEvent
                  );
  ASSERT_EFI_ERROR (Status);
}

/**
  This is the declaration of an EFI image entry point. This can be the entry point to an application
  written to this specification, an EFI boot service driver, or an EFI runtime driver.

  @retval EFI_SUCCESS           The operation completed successfully.
  @retval others                The operation failed.

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
