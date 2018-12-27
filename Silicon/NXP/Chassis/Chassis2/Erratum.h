/** @file
*  Header defining the Base addresses, sizes, flags etc for Erratas
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

#ifndef __ERRATUM_H__
#define __ERRATUM_H__

#define USB_TXVREFTUNE        0x9
#define USB_SQRXTUNE          0xFC7FFFFF
#define USB_PCSTXSWINGFULL    0x47
#define USB_PHY_RX_EQ_VAL_1   0x0000
#define USB_PHY_RX_EQ_VAL_2   0x8000
#define USB_PHY_RX_EQ_VAL_3   0x8003
#define USB_PHY_RX_EQ_VAL_4   0x800b

#define USB_PHY_RX_OVRD_IN_HI 0x200c

VOID
ErratumA009008 (
  VOID
  );

VOID
ErratumA009798 (
  VOID
  );

VOID
ErratumA008997 (
  VOID
  );

VOID
ErratumA009007 (
  VOID
  );

#endif /* __ERRATUM_H__*/
