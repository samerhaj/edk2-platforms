/** Dpaa1DebugLib.h
  DPAA1 debug macros

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __DPAA1_DEBUG_LIB_H__
#define __DPAA1_DEBUG_LIB_H__

#include <Library/DebugLib.h>
#include <Uefi.h>

/*
 * DPAA1 debugging flags
 */
#define DPAA1_DEBUG_MESSAGES_ON             0x1     /* Enable DPAA1 debugging messages */
#define DPAA1_DEBUG_DUMP_VALUES             0x2     /* Dump values of RAM words or registers */
#define DPAA1_DEBUG_EXTRA_CHECKS            0x4     /* Perform extra checks */
#define DPAA1_DEBUG_TRACE_NET_PACKETS       0x8     /* Trace sent/received network packets */

/**
 * Print a debug message with prefix, if DPAA1_DEBUG_MESSAGES_ON set
 */
#define DPAA1_DEBUG_MSG(_Fmt, ...) \
  do {                                                                  \
    if (gDpaa1DebugFlags & DPAA1_DEBUG_MESSAGES_ON) {                   \
        DEBUG((EFI_D_INFO, "DPAA1 DBG: " _Fmt, ##__VA_ARGS__));      \
    }                                                                   \
  } while (0)

/**
 * Print a debug message with no prefix, if DPAA1_DEBUG_MESSAGES_ON set
 */
#define DPAA1_DEBUG_MSG_NO_PREFIX(_Fmt, ...) \
  do {                                                              \
    if (gDpaa1DebugFlags & DPAA1_DEBUG_MESSAGES_ON) {               \
        DEBUG((EFI_D_INFO, _Fmt, ##__VA_ARGS__));                \
    }                                                               \
  } while (0)

/**
 * Print an error message with prefix
 */
#define DPAA1_ERROR_MSG(_Fmt, ...) \
  do {                                                              \
    DEBUG((EFI_D_ERROR, "DPAA1 ERROR: " _Fmt, ##__VA_ARGS__));      \
  } while (0)

/**
 * Print a warning message with prefix
 */
#define DPAA1_WARN_MSG(_Fmt, ...) \
  do {                                                              \
    DEBUG((EFI_D_INFO, "DPAA1 WARNING: " _Fmt, ##__VA_ARGS__));     \
  } while (0)

/**
 * Print a production informational message with prefix
 */
#define DPAA1_INFO_MSG(_Fmt, ...) \
  do {                                                              \
    DEBUG((EFI_D_INFO, "DPAA1: " _Fmt, ##__VA_ARGS__));          \
  } while (0)

/**
 * Print a production informational message with no prefix
 */
#define DPAA1_INFO_MSG_NO_PREFIX(_Fmt, ...) \
  do {                                                              \
    DEBUG((EFI_D_INFO, _Fmt, ##__VA_ARGS__));                    \
  } while (0)

extern UINT32 gDpaa1DebugFlags;

#endif /* __DPAA1_DEBUG_LIB_H__ */
