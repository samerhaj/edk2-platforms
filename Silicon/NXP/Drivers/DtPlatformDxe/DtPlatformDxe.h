/** @file
*
*  Copyright 2018 NXP
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

#ifndef __DT_PLATFORM_DXE_H__
#define __DT_PLATFORM_DXE_H__

#define JR3_OFFSET                0x40000

#define VIDMS_OFFSET              0xFF8
#define CCBVID_OFFSET             0xFE4

#define SECVID_MS_IPID_MASK       0xffff0000
#define SECVID_MS_IPID_SHIFT      16
#define SECVID_MS_MAJ_REV_MASK    0x0000ff00
#define SECVID_MS_MAJ_REV_SHIFT   8
#define CCBVID_ERA_MASK           0xff000000
#define CCBVID_ERA_SHIFT          24

// Structure defining the mapping between SEC block Era and Id-Rev
typedef struct _ERA_MAP{
  UINT16 Id;
  UINT8  Rev;
  UINT8  Era;
} ERA_MAP;

STATIC CONST ERA_MAP Eras[] = {
  {0x0A10, 1, 1},
  {0x0A10, 2, 2},
  {0x0A12, 1, 3},
  {0x0A14, 1, 3},
  {0x0A14, 2, 4},
  {0x0A16, 1, 4},
  {0x0A10, 3, 4},
  {0x0A11, 1, 4},
  {0x0A18, 1, 4},
  {0x0A11, 2, 5},
  {0x0A12, 2, 5},
  {0x0A13, 1, 5},
  {0x0A1C, 1, 5}
};

#endif // __DT_PLATFORM_DXE_H__
