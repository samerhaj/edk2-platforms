/** WatchDog.c
*
*  Based on Watchdog driver implemenation available in
*  ArmPlatformPkg/Drivers/SP805WatchdogDxe/SP805Watchdog.c
*
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
#include <Library/BaseLib.h>
#include <Library/BeIoLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Protocol/WatchdogTimer.h>

#include "WatchDog.h"

STATIC EFI_EVENT  EfiExitBootServicesEvent;
STATIC EFI_EVENT  WdogFeedEvent;

STATIC
UINT16
EFIAPI
WdogRead (
  IN  UINTN     Address
  )
{
  if (FixedPcdGetBool (PcdWdogBigEndian)) {
    return BeMmioRead16 (Address);
  } else {
    return MmioRead16(Address);
  }
}

STATIC
UINT16
EFIAPI
WdogWrite (
  IN  UINTN     Address,
  IN  UINT16    Value
  )
{
  if (FixedPcdGetBool (PcdWdogBigEndian)) {
    return BeMmioWrite16 (Address, Value);
  } else {
    return MmioWrite16 (Address, Value);
  }
}

STATIC
UINT16
EFIAPI
WdogAndThenOr (
  IN  UINTN     Address,
  IN  UINT16    And,
  IN  UINT16    Or
  )
{
  if (FixedPcdGetBool (PcdWdogBigEndian)) {
    return BeMmioAndThenOr16 (Address, And, Or);
  } else {
    return MmioAndThenOr16 (Address, And, Or);
  }
}

STATIC
UINT16
EFIAPI
WdogOr (
  IN  UINTN     Address,
  IN  UINT16    Or
  )
{
  if (FixedPcdGetBool (PcdWdogBigEndian)) {
    return BeMmioOr16 (Address, Or);
  } else {
    return MmioOr16 (Address, Or);
  }
}

STATIC
VOID
WdogPing (
  VOID
  )
{
  //
  // To reload a timeout value to the counter the proper service sequence begins by
  // writing 0x_5555 followed by 0x_AAAA to the Watchdog Service Register (WDOG_WSR).
  // This service sequence will reload the counter with the timeout value WT[7:0] of
  // Watchdog Control Register (WDOG_WCR).
  //

  WdogWrite (PcdGet64 (PcdWdog1BaseAddr) + WDOG_WSR_OFFSET,
                     WDOG_SERVICE_SEQ1);
  WdogWrite (PcdGet64 (PcdWdog1BaseAddr) + WDOG_WSR_OFFSET,
                     WDOG_SERVICE_SEQ2);
}

/**
  Stop the Wdog watchdog timer from counting down.
**/
STATIC
VOID
WdogStop (
  VOID
  )
{
  // Watchdog cannot be disabled by software once started.
  // At best, we can keep reload counter with maximum value

  WdogAndThenOr (PcdGet64 (PcdWdog1BaseAddr) + WDOG_WCR_OFFSET,
                 (UINT16)(~WDOG_WCR_WT),
                 (WD_COUNT (WT_MAX_TIME) & WD_COUNT_MASK));
  WdogPing ();
}

/**
  Starts the Wdog counting down by feeding Service register with
  desired pattern.
  The count down will start from the value stored in the Load register,
  not from the value where it was previously stopped.
**/
STATIC
VOID
WdogStart (
  VOID
  )
{
  //Reload the timeout value
  WdogPing ();
}

/**
    On exiting boot services we must make sure the Wdog Watchdog Timer
    is stopped.
**/
STATIC
VOID
EFIAPI
ExitBootServicesEvent (
  IN EFI_EVENT  Event,
  IN VOID       *Context
  )
{
  WdogStop ();
}

/**
  This function registers the handler NotifyFunction so it is called every time
  the watchdog timer expires.  It also passes the amount of time since the last
  handler call to the NotifyFunction.
  If NotifyFunction is not NULL and a handler is not already registered,
  then the new handler is registered and EFI_SUCCESS is returned.
  If NotifyFunction is NULL, and a handler is already registered,
  then that handler is unregistered.
  If an attempt is made to register a handler when a handler is already registered,
  then EFI_ALREADY_STARTED is returned.
  If an attempt is made to unregister a handler when a handler is not registered,
  then EFI_INVALID_PARAMETER is returned.

  @param  This             The EFI_TIMER_ARCH_PROTOCOL instance.
  @param  NotifyFunction   The function to call when a timer interrupt fires. This
                           function executes at TPL_HIGH_LEVEL. The DXE Core will
                           register a handler for the timer interrupt, so it can know
                           how much time has passed. This information is used to
                           signal timer based events. NULL will unregister the handler.

  @retval EFI_SUCCESS           The watchdog timer handler was registered.
  @retval EFI_ALREADY_STARTED   NotifyFunction is not NULL, and a handler is already
                                registered.
  @retval EFI_INVALID_PARAMETER NotifyFunction is NULL, and a handler was not
                                previously registered.

**/
STATIC
EFI_STATUS
EFIAPI
WdogRegisterHandler (
  IN EFI_WATCHDOG_TIMER_ARCH_PROTOCOL   *This,
  IN EFI_WATCHDOG_TIMER_NOTIFY          NotifyFunction
  )
{
  // ERROR: This function is not supported.
  // The hardware watchdog will reset the board
  return EFI_INVALID_PARAMETER;
}

/**

  This function adjusts the period of timer interrupts to the value specified
  by TimerPeriod.  If the timer period is updated, then the selected timer
  period is stored in EFI_TIMER.TimerPeriod, and EFI_SUCCESS is returned.  If
  the timer hardware is not programmable, then EFI_UNSUPPORTED is returned.
  If an error occurs while attempting to update the timer period, then the
  timer hardware will be put back in its state prior to this call, and
  EFI_DEVICE_ERROR is returned.  If TimerPeriod is 0, then the timer interrupt
  is disabled.  This is not the same as disabling the CPU's interrupts.
  Instead, it must either turn off the timer hardware, or it must adjust the
  interrupt controller so that a CPU interrupt is not generated when the timer
  interrupt fires.

  @param  This             The EFI_TIMER_ARCH_PROTOCOL instance.
  @param  TimerPeriod      The rate to program the timer interrupt in 100 nS units. If
                           the timer hardware is not programmable, then EFI_UNSUPPORTED is
                           returned. If the timer is programmable, then the timer period
                           will be rounded up to the nearest timer period that is supported
                           by the timer hardware. If TimerPeriod is set to 0, then the
                           timer interrupts will be disabled.


  @retval EFI_SUCCESS           The timer period was changed.
  @retval EFI_UNSUPPORTED       The platform cannot change the period of the timer interrupt.
  @retval EFI_DEVICE_ERROR      The timer period could not be changed due to a device error.

**/
STATIC
EFI_STATUS
EFIAPI
WdogSetTimerPeriod (
  IN EFI_WATCHDOG_TIMER_ARCH_PROTOCOL   *This,
  IN UINT64                             TimerPeriod   // In 100ns units
  )
{
  EFI_STATUS  Status;
  UINT64      TimerPeriodInSec;
  UINT16      Val;

  Status = EFI_SUCCESS;

  if (TimerPeriod == 0) {
    // This is a watchdog stop request
    WdogStop ();
    return Status;
  } else {
    // Convert the TimerPeriod (in 100 ns unit) to an equivalent second value

    TimerPeriodInSec = DivU64x32 (TimerPeriod, NANO_SECOND_BASE);

    // The registers in the Wdog are only 32 bits
    if (TimerPeriodInSec > WT_MAX_TIME) {
      // We could load the watchdog with the maximum supported value but
      // if a smaller value was requested, this could have the watchdog
      // triggering before it was intended.
      // Better generate an error to let the caller know.
      Status = EFI_DEVICE_ERROR;
      return Status;
    }

    // set the new timeout value in the WCR
    // Convert the timeout value from Seconds to timer count
    Val = ((WD_COUNT(TimerPeriodInSec) & WD_COUNT_MASK) << 8);

    WdogAndThenOr (PcdGet64 (PcdWdog1BaseAddr) + WDOG_WCR_OFFSET,
                   (UINT16)(~WDOG_WCR_WT),
                   Val);
    // Start the watchdog
    WdogStart ();
  }

  return Status;
}

/**
  This function retrieves the period of timer interrupts in 100 ns units,
  returns that value in TimerPeriod, and returns EFI_SUCCESS.  If TimerPeriod
  is NULL, then EFI_INVALID_PARAMETER is returned.  If a TimerPeriod of 0 is
  returned, then the timer is currently disabled.

  @param  This             The EFI_TIMER_ARCH_PROTOCOL instance.
  @param  TimerPeriod      A pointer to the timer period to retrieve in 100 ns units. If
                           0 is returned, then the timer is currently disabled.


  @retval EFI_SUCCESS           The timer period was returned in TimerPeriod.
  @retval EFI_INVALID_PARAMETER TimerPeriod is NULL.

**/
STATIC
EFI_STATUS
EFIAPI
WdogGetTimerPeriod (
  IN  EFI_WATCHDOG_TIMER_ARCH_PROTOCOL   *This,
  OUT UINT64                             *TimerPeriod
  )
{
  EFI_STATUS  Status;
  UINT64      ReturnValue;
  UINT16      Val;

  Status = EFI_SUCCESS;

  if (TimerPeriod == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  // Check if the watchdog is stopped
  if ((WdogRead (PcdGet64 (PcdWdog1BaseAddr) + WDOG_WCR_OFFSET)
              & WDOG_WCR_WDE) == 0 ) {
    // It is stopped, so return zero.
    ReturnValue = 0;
  } else {
    // Convert the Watchdog ticks into equivalent TimerPeriod second value.
    Val = (WdogRead (PcdGet64 (PcdWdog1BaseAddr) + WDOG_WCR_OFFSET)
            & WDOG_WCR_WT ) >> 8;
    ReturnValue = WD_SEC(Val);
  }

  *TimerPeriod = ReturnValue;
  return Status;
}

/**
  Interface structure for the Watchdog Architectural Protocol.

  @par Protocol Description:
  This protocol provides a service to set the amount of time to wait
  before firing the watchdog timer, and it also provides a service to
  register a handler that is invoked when the watchdog timer fires.

  @par When the watchdog timer fires, control will be passed to a handler
  if one has been registered.  If no handler has been registered,
  or the registered handler returns, then the system will be
  reset by calling the Runtime Service ResetSystem().

  @param RegisterHandler
  Registers a handler that will be called each time the
  watchdogtimer interrupt fires.  TimerPeriod defines the minimum
  time between timer interrupts, so TimerPeriod will also
  be the minimum time between calls to the registered
  handler.
  NOTE: If the watchdog resets the system in hardware, then
        this function will not have any chance of executing.

  @param SetTimerPeriod
  Sets the period of the timer interrupt in 100 nS units.
  This function is optional, and may return EFI_UNSUPPORTED.
  If this function is supported, then the timer period will
  be rounded up to the nearest supported timer period.

  @param GetTimerPeriod
  Retrieves the period of the timer interrupt in 100 nS units.

**/
STATIC
EFI_WATCHDOG_TIMER_ARCH_PROTOCOL  gWatchdogTimer = {
  WdogRegisterHandler,
  WdogSetTimerPeriod,
  WdogGetTimerPeriod
};

/**
  Call back function when the timer event is signaled.
  This function will feed the watchdog with maximum value
  so that system wont reset in idle case e.g. stopped on UEFI shell.

  @param[in]  Event     The Event this notify function registered to.
  @param[in]  Context   Pointer to the context data registered to the
                        Event.

**/
VOID
EFIAPI
WdogFeed (
  IN EFI_EVENT          Event,
  IN VOID*              Context
  )
{
  WdogPing();
}
/**
  Initialize state information for the Watchdog Timer Architectural Protocol.

  @param  ImageHandle   of the loaded driver
  @param  SystemTable   Pointer to the System Table

  @retval EFI_SUCCESS           Protocol registered
  @retval EFI_OUT_OF_RESOURCES  Cannot allocate protocol data structure
  @retval EFI_DEVICE_ERROR      Hardware problems

**/
EFI_STATUS
EFIAPI
WdogInitialize (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS  Status;
  EFI_HANDLE  Handle;

  WdogAndThenOr (PcdGet64 (PcdWdog1BaseAddr) + WDOG_WCR_OFFSET,
                 (UINT16)(~WDOG_WCR_WT),
                 (WD_COUNT (WT_MAX_TIME) & WD_COUNT_MASK));

  WdogOr (PcdGet64 (PcdWdog1BaseAddr) + WDOG_WCR_OFFSET, WDOG_WCR_WDE);

  //
  // Make sure the Watchdog Timer Architectural Protocol
  // has not been installed in the system yet.
  // This will avoid conflicts with the universal watchdog
  //
  ASSERT_PROTOCOL_ALREADY_INSTALLED (NULL, &gEfiWatchdogTimerArchProtocolGuid);

  // Register for an ExitBootServicesEvent
  Status = gBS->CreateEvent (EVT_SIGNAL_EXIT_BOOT_SERVICES, TPL_NOTIFY,
              ExitBootServicesEvent, NULL, &EfiExitBootServicesEvent);
  if (EFI_ERROR (Status)) {
    Status = EFI_OUT_OF_RESOURCES;
    return Status;
  }

  //
  // Start the timer to feed Watchdog with maximum timeout value.
  //
  Status = gBS->CreateEvent (
                  EVT_TIMER | EVT_NOTIFY_SIGNAL,
                  TPL_NOTIFY,
                  WdogFeed,
                  NULL,
                  &WdogFeedEvent
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = gBS->SetTimer (WdogFeedEvent, TimerPeriodic, WT_FEED_INTERVAL);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // Install the Timer Architectural Protocol onto a new handle
  Handle = NULL;
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &Handle,
                  &gEfiWatchdogTimerArchProtocolGuid, &gWatchdogTimer,
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    gBS->CloseEvent (EfiExitBootServicesEvent);
    Status = EFI_OUT_OF_RESOURCES;
    return Status;
  }

  WdogPing ();

  return Status;
}
