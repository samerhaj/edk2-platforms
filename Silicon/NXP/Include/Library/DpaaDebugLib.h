/** DpaaDebugLib.h
  DPAA debug macros

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __DPAA_DEBUG_LIB_H__
#define __DPAA_DEBUG_LIB_H__

#include <Library/DebugLib.h>
#include <Uefi.h>

/*
 * DPAA debugging flags
 */
#define DPAA_DEBUG_MESSAGES_ON             0x1     /* Enable DPAA debugging messages */
#define DPAA_DEBUG_DUMP_VALUES             0x2     /* Dump values of RAM words or registers */
#define DPAA_DEBUG_TRACE_MC_COMMANDS       0x4     /* Trace commands sent to the MC */
#define DPAA_DEBUG_DUMP_MC_LOG_FRAGMENT    0x8     /* Dump MC log fragment */
#define DPAA_DEBUG_DUMP_ROOT_DPRC          0x10    /* Dump contents of the root DPRC */
#define DPAA_DEBUG_EXTRA_CHECKS            0x20    /* Perform extra checks */
#define DPAA_DEBUG_TRACE_NET_PACKETS       0x40    /* Trace sent/received network packets */

/**
 * Print a debug message with prefix, if DPAA_DEBUG_MESSAGES_ON set
 */
#define DPAA_DEBUG_MSG(_Fmt, ...) \
  do {                                                                  \
    if (gDpaaDebugFlags & DPAA_DEBUG_MESSAGES_ON) {                   \
        DEBUG ((DEBUG_INFO, "%a DBG: " _Fmt, gDpaaDebugString, ##__VA_ARGS__));      \
    }                                                                   \
  } while (0)

/**
 * Print a debug message with no prefix, if DPAA_DEBUG_MESSAGES_ON set
 */
#define DPAA_DEBUG_MSG_NO_PREFIX(_Fmt, ...) \
  do {                                                              \
    if (gDpaaDebugFlags & DPAA_DEBUG_MESSAGES_ON) {               \
        DEBUG ((DEBUG_INFO, _Fmt, ##__VA_ARGS__));                \
    }                                                               \
  } while (0)

/**
 * Print an error message with prefix
 */
#define DPAA_ERROR_MSG(_Fmt, ...) \
  do {                                                              \
    DEBUG ((DEBUG_ERROR, "%a ERROR: " _Fmt, gDpaaDebugString, ##__VA_ARGS__));      \
  } while (0)

/**
 * Print a warning message with prefix
 */
#define DPAA_WARN_MSG(_Fmt, ...) \
  do {                                                              \
    DEBUG ((DEBUG_INFO, "%a WARNING: " _Fmt, gDpaaDebugString, ##__VA_ARGS__));     \
  } while (0)

/**
 * Print a production informational message with prefix
 */
#define DPAA_INFO_MSG(_Fmt, ...) \
  do {                                                              \
    DEBUG ((DEBUG_INFO, "%a: " _Fmt, gDpaaDebugString, ##__VA_ARGS__));          \
  } while (0)

/**
 * Print a production informational message with no prefix
 */
#define DPAA_INFO_MSG_NO_PREFIX(_Fmt, ...) \
  do {                                                              \
    DEBUG ((DEBUG_INFO, _Fmt, ##__VA_ARGS__));                    \
  } while (0)

extern UINT32 gDpaaDebugFlags;
extern CHAR8  * CONST gDpaaDebugString;

#endif /* __DPAA_DEBUG_LIB_H__ */
