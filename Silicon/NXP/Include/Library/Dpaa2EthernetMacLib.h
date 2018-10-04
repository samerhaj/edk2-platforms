/** Dpaa2EthernetMacLib.h
  DPAA2 Ethernet MAC library interface

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __DPAA2_ETHERTNET_MAC_LIB_H__
#define __DPAA2_ETHERTNET_MAC_LIB_H__

#include <Library/BaseLib.h>
#include <Library/Dpaa2EthernetPhyLib.h>
#include <Uefi.h>

#define MAC_MULTICAST_ADDRESS_BIT   0x1

/**
 * DPAA2 WRIOP DPMAC IDs
 *
 * NOTE: Valid DPMAC IDs must start at 1, since that is what the dpmac_create()
 * MC command expects.
 */
typedef enum _WRIOP_DPMAC_ID {
  INVALID_WRIOP_DPMAC_ID = 0x0,
  WRIOP_DPMAC1 = 0x1,
  WRIOP_DPMAC2,
  WRIOP_DPMAC3,
  WRIOP_DPMAC4,
  WRIOP_DPMAC5,
  WRIOP_DPMAC6,
  WRIOP_DPMAC7,
  WRIOP_DPMAC8,
  WRIOP_DPMAC9,
  WRIOP_DPMAC10,
  WRIOP_DPMAC11,
  WRIOP_DPMAC12,
  WRIOP_DPMAC13,
  WRIOP_DPMAC14,
  WRIOP_DPMAC15,
  WRIOP_DPMAC16,
  WRIOP_DPMAC17,
  WRIOP_DPMAC18,
  WRIOP_DPMAC19,
  WRIOP_DPMAC20,
  WRIOP_DPMAC21,
  WRIOP_DPMAC22,
  WRIOP_DPMAC23,
  WRIOP_DPMAC24,

  /*
   * New values must added above this entry
   */
  NUM_WRIOP_DPMACS,
} WRIOP_DPMAC_ID;

/**
 * State information kept for a WRIOP DPMAC
 */
typedef struct _WRIOP_DPMAC {
  /**
   * Signature for run-time type checking
   */
  UINT32 Signature;
# define WRIOP_DPMAC_SIGNATURE SIGNATURE_32('D', 'P', 'M', 'A')

  /**
   * Dpmac ID
   */
  WRIOP_DPMAC_ID Id;

  /**
   * Flag indicating if this Dpmac is enabled (corresponding bit in the
   * CCSR DEVDISR2 register is off)
   */
  BOOLEAN Enabled;

  /**
   * Node in a linked list of enabled DPMACs
   */
  LIST_ENTRY ListNode;

  /**
   * Ethernet PHY associated with this DPMAC
   */
  DPAA2_PHY Phy;

} WRIOP_DPMAC;

VOID
WriopDpmacInit (
  WRIOP_DPMAC_ID     DpmacId,
  PHY_INTERFACE_TYPE PhyInterfaceType,
  DPAA2_PHY_MDIO_BUS *MdioBus,
  UINT8              PhyAddress,
  PHY_MEDIA_TYPE     PhyMediaType,
  UINT8              PhyId,
  LIST_ENTRY         *EnabledDmpacsList
  );

CONST
CHAR8 *
PhyInterfaceTypeToString (
  PHY_INTERFACE_TYPE PhyInterfaceType
  );

extern const CHAR8 *const gWriopDpmacStrings[];

#endif /* __DPAA2_ETHERTNET_MAC_LIB_H__ */
