/*++

Copyright (c) 2007-2010, Intel Corporation
All rights reserved. This program and the accompanying materials
are licensed and made available under the terms and conditions of the BSD License
which accompanies this distribution.  The full text of the license may be found at
http://opensource.org/licenses/bsd-license.php

THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

Module Name:

  NVDataStruc.h

Abstract:

  NVData structure used by the sample driver

Revision History:

--*/

#ifndef _NVDATASTRUC_H
#define _NVDATASTRUC_H

#define E1000_HII_FORM_GUID \
  { \
    0x77f2ea2f, 0x4312, 0x4569, { 0x85, 0xc4, 0x58, 0x3a, 0xcd, 0x8d, 0xb7, 0xe2 } \
  }

#define E1000_HII_DATA_GUID \
  { \
    0xa31abb16, 0xc627, 0x475b, { 0x98, 0x8e, 0x7e, 0xe0, 0x77, 0x67, 0x40, 0xf3 } \
  }

#define VAR_EQ_TEST_NAME  0x100

#define FORM_2							0x1235

#define LABEL_START						0x1236
#define LABEL_END                   	0x2223

#define LINK_SPEED_AUTO_NEG				0x00
#define LINK_SPEED_10HALF				0x01
#define LINK_SPEED_10FULL				0x02
#define LINK_SPEED_100HALF				0x03
#define LINK_SPEED_100FULL				0x04
#define LINK_SPEED_NO_CONFIGURE_AUTO	0x10

#define WOL_DISABLE 					0x00
#define WOL_ENABLE  					0x01
#define WOL_NA      					0x02

#define OROM_DISABLE					0x00
#define OROM_ENABLE						0x01


#pragma pack(1)
typedef struct {
  UINT8   OptionRomEnable;
  UINT8   LinkSpeed;
  UINT8   WolEnable;
} GIG_DRIVER_CONFIGURATION;
#pragma pack()

#endif

