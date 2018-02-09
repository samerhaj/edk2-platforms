/** @file
  Provides enhancements to TimerLib. Uses standard TimerLib APIs.

  Copyright (C) 2017, Red Hat, Inc.
  Copyright 2017 NXP.

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS, WITHOUT
  WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
**/

#include <Library/BaseLib.h>
#include <Library/TimerLib.h>

/**
  From two values retrieved with TimerLib's GetPerformanceCounter(), calculate
  the number of ticks elapsed between them, such that the difference can be
  passed to TimerLib's GetTimeInNanoSecond().

  The function cannot handle multiple wrap-arounds of the performance counter.

  @param[in] PerfCounterStart  The value returned by GetPerformanceCounter(),
                               called earlier in wall clock time.

  @param[in] PerfCounterStop   The value returned by GetPerformanceCounter(),
                               called later in wall clock time.

  @retval  The number of ticks corresponding to the conceptual difference
           (PerfCounterStop-PerfCounterStart).
**/
UINT64
EFIAPI
GetTickDifference (
  IN UINT64 PerfCounterStart,
  IN UINT64 PerfCounterStop
  )
{
  UINT64 StartValue;
  UINT64 EndValue;
  UINT64 TickDifference;

  GetPerformanceCounterProperties (&StartValue, &EndValue);

  if (StartValue < EndValue) {
    //
    // The performance counter counts up.
    //
    if (PerfCounterStart < PerfCounterStop) {
      //
      // The counter didn't wrap around.
      //
      TickDifference = PerfCounterStop - PerfCounterStart;
    } else {
      //
      // The counter wrapped around.
      //
      TickDifference = (EndValue - PerfCounterStart) +
                       (PerfCounterStop - StartValue);
    }
  } else {
    //
    // The performance counter counts down.
    //
    if (PerfCounterStart < PerfCounterStop) {
      //
      // The counter wrapped around.
      //
      TickDifference = (PerfCounterStart - EndValue) +
                       (StartValue - PerfCounterStop);
    } else {
      //
      // The counter didn't wrap around.
      //
      TickDifference = PerfCounterStart - PerfCounterStop;
    }
  }

  return TickDifference;
}

/**
  From two values retrieved with TimerLib's GetPerformanceCounter(), calculate
  the number of nanoseconds elapsed between them.

  The function cannot handle multiple wrap-arounds of the performance counter.

  @param[in] PerfCounterStart  The value returned by GetPerformanceCounter(),
                               called earlier in wall clock time.

  @param[in] PerfCounterStop   The value returned by GetPerformanceCounter(),
                               called later in wall clock time.

  @retval  The number of nanoseconds corresponding to the conceptual difference
           (PerfCounterStop-PerfCounterStart).
**/
UINT64
EFIAPI
GetNanoSecondDifference (
  IN UINT64 PerfCounterStart,
  IN UINT64 PerfCounterStop
  )
{
  UINT64 TickDifference;
  UINT64 NanoSecondDifference;

  TickDifference = GetTickDifference (PerfCounterStart, PerfCounterStop);
  NanoSecondDifference = GetTimeInNanoSecond (TickDifference);
  return NanoSecondDifference;
}

/**
  From two values retrieved with TimerLib's GetPerformanceCounter(), calculate
  the number of microseconds elapsed between them.

  The function cannot handle multiple wrap-arounds of the performance counter.

  @param[in] PerfCounterStart  The value returned by GetPerformanceCounter(),
                               called earlier in wall clock time.

  @param[in] PerfCounterStop   The value returned by GetPerformanceCounter(),
                               called later in wall clock time.

  @retval  The number of microseconds corresponding to the conceptual
           difference (PerfCounterStop-PerfCounterStart).
**/
UINT64
EFIAPI
GetMicroSecondDifference (
  IN UINT64 PerfCounterStart,
  IN UINT64 PerfCounterStop
  )
{
  UINT64 NanoSecondDifference;
  UINT64 MicroSecondDifference;

  NanoSecondDifference = GetNanoSecondDifference (
                           PerfCounterStart,
                           PerfCounterStop
                           );
  MicroSecondDifference = DivU64x32 (NanoSecondDifference, 1000);
  return MicroSecondDifference;
}

/**
  From two values retrieved with TimerLib's GetPerformanceCounter(), calculate
  the number of milliseconds elapsed between them.

  The function cannot handle multiple wrap-arounds of the performance counter.

  @param[in] PerfCounterStart  The value returned by GetPerformanceCounter(),
                               called earlier in wall clock time.

  @param[in] PerfCounterStop   The value returned by GetPerformanceCounter(),
                               called later in wall clock time.

  @retval  The number of milliseconds corresponding to the conceptual
           difference (PerfCounterStop-PerfCounterStart).
**/
UINT64
EFIAPI
GetMilliSecondDifference (
  IN UINT64 PerfCounterStart,
  IN UINT64 PerfCounterStop
  )
{
  UINT64 NanoSecondDifference;
  UINT64 MilliSecondDifference;

  NanoSecondDifference = GetNanoSecondDifference (
                           PerfCounterStart,
                           PerfCounterStop
                           );
  MilliSecondDifference = DivU64x32 (NanoSecondDifference, 1000 * 1000);
  return MilliSecondDifference;
}

/**
  Get Elapsed time in NanoSeconds w.r.t BaseTime

  This function calculates the time elapsed in NanoSeconds between call
  to this function and BaseTime, which is passed as argument.

  @param  BaseTime     BaseTime in NanoSeconds.

  @return              The elapsed time in NanoSeconds.

**/
UINT64
EFIAPI
GetElapsedTime (
  IN      UINT64                     BaseTime
  )
{
  UINT64   StartValue;
  UINT64   EndValue;
  UINT64   NanoSeconds;
  UINT64   NanoSecondsDifference;

  NanoSeconds = GetTimeInNanoSecond (GetPerformanceCounter ());

  GetPerformanceCounterProperties (&StartValue, &EndValue);

  if (StartValue < EndValue) {
    //
    // The performance counter counts up.
    //
    if (BaseTime < NanoSeconds) {
      //
      // The counter didn't wrap around.
      //
      NanoSecondsDifference = NanoSeconds - BaseTime;
    } else {
      //
      // The counter wrapped around.
      //
      NanoSecondsDifference = (GetTimeInNanoSecond (EndValue) - BaseTime) +
                              (NanoSeconds - GetTimeInNanoSecond (StartValue));
    }
  } else {
    //
    // The performance counter counts down.
    //
    if (BaseTime < NanoSeconds) {
      //
      // The counter wrapped around.
      //
      NanoSecondsDifference = (BaseTime - GetTimeInNanoSecond (EndValue)) +
                              (GetTimeInNanoSecond (StartValue) - NanoSeconds);
    } else {
      //
      // The counter didn't wrap around.
      //
      NanoSecondsDifference = BaseTime - NanoSeconds;
    }
  }

  return NanoSecondsDifference;
}
