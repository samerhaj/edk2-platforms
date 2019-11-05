/** @file
  SMBIOS struct information for NXP LX2160ARDB platforms

  Copyright (c) 2015, ARM Limited. All rights reserved.
  Copyright 2019 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

  Based on the files under ARM/JunoPkg/SmbiosPlatformDxe

**/

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Protocol/Smbios.h>

#define SERIAL_LEN 10  /* this must be less than the buffer len allocated in the type1 structure */

#define TYPE0_STRINGS                                    \
  "EFI Development Kit II / NXP\0"   /* Vendor */      \
  "EDK II\0"                         /* BiosVersion */ \
  __DATE__"\0"                       /* BiosReleaseDate */

#define TYPE1_STRINGS                                   \
  "NXP\0"                            /* Manufacturer */ \
  "NXP LX2160ARDB Platform\0"        /* Product Name */ \
  "None\0"                           /* Version */      \
  "                    \0"           /* 20 character buffer */

#define TYPE3_STRINGS                                   \
  "NXP\0"                            /* Manufacturer */ \
  "None\0"                           /* Version */      \
  "Serial Not Set\0"                 /* Serial  */

#define TYPE4_STRINGS                                               \
  "BGA-1156\0"                       /* socket type */              \
  "NXP\0"                            /* manufactuer */              \
  "Cortex-A57\0"                     /* processor 1 description */  \
  "Cortex-A53\0"                     /* processor 2 description */  \
  "Cortex-A72\0"                     /* processor 2 description */  \
  "0xd03\0"                          /* A53 part number */          \
  "0xd07\0"                          /* A57 part number */          \
  "0xd08\0"                          /* A72 part number */

#define TYPE7_STRINGS                              \
  "L1 Instruction\0"                 /* L1I  */    \
  "L1 Data\0"                        /* L1D  */    \
  "L2\0"                             /* L2   */

#define TYPE16_STRINGS                             \
  "\0"                               /* nothing */

#define TYPE17_STRINGS                                       \
  "RIGHT SIDE\0"                     /* location */          \
  "BANK 0\0"                         /* bank description */

#define TYPE19_STRINGS                             \
  "\0"                               /* nothing */

#define TYPE32_STRINGS                             \
  "\0"                               /* nothing */


/**
 * Type definition and contents of the default SMBIOS table.
 * This table covers only the minimum structures required by
 * the SMBIOS specification (section 6.2, version 3.0)
 */
#pragma pack(1)
typedef struct {
  SMBIOS_TABLE_TYPE0 Base;
  INT8              Strings [sizeof (TYPE0_STRINGS)];
} ARM_TYPE0;

typedef struct {
  SMBIOS_TABLE_TYPE1 Base;
  UINT8              Strings [sizeof (TYPE1_STRINGS)];
} ARM_TYPE1;

typedef struct {
  SMBIOS_TABLE_TYPE3 Base;
  UINT8              Strings [sizeof (TYPE3_STRINGS)];
} ARM_TYPE3;

typedef struct {
  SMBIOS_TABLE_TYPE4 Base;
  UINT8              Strings [sizeof (TYPE4_STRINGS)];
} ARM_TYPE4;

typedef struct {
  SMBIOS_TABLE_TYPE7 Base;
  UINT8              Strings [sizeof (TYPE7_STRINGS)];
} ARM_TYPE7;


typedef struct {
  SMBIOS_TABLE_TYPE16 Base;
  UINT8              Strings [sizeof (TYPE16_STRINGS)];
} ARM_TYPE16;

typedef struct {
  SMBIOS_TABLE_TYPE17 Base;
  UINT8              Strings [sizeof (TYPE17_STRINGS)];
} ARM_TYPE17;

typedef struct {
  SMBIOS_TABLE_TYPE19 Base;
  UINT8              Strings [sizeof (TYPE19_STRINGS)];
} ARM_TYPE19;

typedef struct {
  SMBIOS_TABLE_TYPE32 Base;
  UINT8              Strings [sizeof (TYPE32_STRINGS)];
} ARM_TYPE32;

/**
 * SMBIOS tables often reference each other using
 * fixed constants, define a list of these constants
 * for our hardcoded tables
 */
typedef enum  {
  SMBIOS_HANDLE_A57_L1I = 0x1000,
  SMBIOS_HANDLE_A57_L1D,
  SMBIOS_HANDLE_A57_L2,
  SMBIOS_HANDLE_A53_L1I,
  SMBIOS_HANDLE_A53_L1D,
  SMBIOS_HANDLE_A53_L2,
  SMBIOS_HANDLE_MOTHERBOARD,
  SMBIOS_HANDLE_CHASSIS,
  SMBIOS_HANDLE_A72_CLUSTER,
  SMBIOS_HANDLE_A57_CLUSTER,
  SMBIOS_HANDLE_A53_CLUSTER,
  SMBIOS_HANDLE_MEMORY,
  SMBIOS_HANDLE_DIMM
} SMBIOS_REFRENCE_HANDLES;
