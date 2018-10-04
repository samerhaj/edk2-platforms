/*++

Copyright (c) 2004-2010, Intel Corporation
All rights reserved. This program and the accompanying materials
are licensed and made available under the terms and conditions of the BSD License
which accompanies this distribution.  The full text of the license may be found at
http://opensource.org/licenses/bsd-license.php

THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

Module Name:

  Hii.h

Abstract:


Revision History

--*/

#ifndef _HII_H_
#define _HII_H_

#include "NVDataStruc.h"

//
// This is the generated header file which includes whatever needs to be exported (strings + IFR)
//
#include "LanIntelE1000DxeStrDefs.h"

//
// This is the generated IFR binary data for each formset defined in VFR.
// This data array is ready to be used as input of PreparePackageList() to
// create a packagelist (which contains Form packages, String packages, etc).
//
extern UINT8  vfrBin[];
extern UINT8  inventoryBin[];

//
// This is the generated String package data for all .UNI files.
// This data array is ready to be used as input of PreparePackageList() to
// create a packagelist (which contains Form packages, String packages, etc).
//
extern UINT8  GigUndiStrings[];

#define SAMPLE_STRING               L"This is an error!"

#define DRIVER_SAMPLE_PRIVATE_SIGNATURE SIGNATURE_32 ('D', 'S', 'p', 's')

//
// EEPROM power management bit definitions
//
#define E1000_INIT_CONTROL_WORD1          0x0A
#define E1000_PME_ENABLE_BIT              0x0008

#define E1000_INIT_CONTROL_WORD2          0x0F
#define E1000_APM_PME_ENABLE_BIT          0x8000

#define E1000_LEGACY_APM_ENABLE_BIT       0x0004
#define E1000_LEGACY_FLASH_DISABLE_BIT    0x0100

#define E1000_INIT_CONTROL_WORD3          0x24
#define E1000_INIT_CONTROL_WORD3_LANB     0x14
#define E1000_FLASH_DISABLE_BIT           0x0800
#define E1000_FLASH_DISABLE_BIT_ZOAR      0x0080
#define E1000_APM_ENABLE_BIT              0x0400

#define E1000_FLASH_SIZE_WORD_HARTW		  0xf
#define E1000_NVM_TYPE_BIT_HARTW          0x1000

#define E1000_HARTW_FLASH_LAN_ADDRESS     0x21
#define E1000_HARTW_EXP_ROM_DISABLE       0x80    /* bit 7 */

#define LAN1_BASE_ADDRESS_82580			  0x80
#define LAN2_BASE_ADDRESS_82580           0xC0
#define LAN3_BASE_ADDRESS_82580           0x100


#define HII_STRING_LEN                    200
#define MAX_PBA_STR_LENGTH                15    // normally it is 10 chars string

#define DRIVER_SAMPLE_PRIVATE_FROM_THIS(a)  CR (a, GIG_UNDI_PRIVATE_DATA, ConfigAccess, GIG_UNDI_DEV_SIGNATURE)

#endif

