/** @file
  Provides enhancements to TimerLib. Uses standard TimerLib APIs.

  Copyright (C) 2017, Red Hat, Inc.
  Copyright 2017 NXP.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __TIMER_EXTENSION_LIB__
#define __TIMER_EXTENSION_LIB__

#include <Base.h>
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
  );

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
  );

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
  );

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
  );

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
  );

#endif // __TIMER_EXTENSION_LIB__
