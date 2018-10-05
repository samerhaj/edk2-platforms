/** Dpaa1BoardLib.h
  DPAA1 Board Specific library interface

  Copyright (c) 2016, Freescale Semiconductor, Inc. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __DPAA1_BOARD_LIB_H__
#define __DPAA1_BOARD_LIB_H__

#include <Library/Dpaa1EthernetMacLib.h>
#include <Library/Dpaa1EthernetPhyLib.h>
#include <SerDes.h>
#include <Uefi.h>

/**
 * Number of Ethernet PHY MDIO buses
 */
#define DPAA1_MDIO_BUSES_COUNT		2

#define RGMII_PHY1_ADDR			0x1
#define RGMII_PHY2_ADDR			0x2

#define QSGMII_PORT1_PHY_ADDR		0x4
#define QSGMII_PORT2_PHY_ADDR		0x5
#define QSGMII_PORT3_PHY_ADDR		0x6
#define QSGMII_PORT4_PHY_ADDR		0x7

#define FM1_10GEC1_PHY_ADDR		0x1

#define FMAN_FW_LENGTH			0x10000

#define MEMAC_ID_MAX_COUNT		4
typedef struct FdtPort {
    FMAN_MEMAC_ID  Id;
    PHY_INTERFACE_TYPE  PhyInterfaceType;
    UINTN  CompatAddress;
} FDT_PORT;

VOID
Dpaa1ParseSerDes(SERDES_PROTOCOL LaneProtocol,
                 VOID *Arg);
VOID
Dpaa1DiscoverFmanMemac(FMAN_MEMAC_ID MemacId,
                       PHY_INTERFACE_TYPE PhyInterfaceType,
                       VOID *Arg);

BOOLEAN
IsMemacEnabled (
  IN  FMAN_MEMAC_ID MemacId
  );

extern DPAA1_PHY_MDIO_BUS gDpaa1MdioBuses[];

/**
 * Mapping of FMAN MEMACs to Ethernet PHYs
 */
typedef struct _MEMAC_PHY_MAPPING {
  /**
   * Pointer to the MDIO bus that connects a MEMAC to a PHY
   */
  DPAA1_PHY_MDIO_BUS *MdioBus;

  /**
   * PHY address of the associated PHY
   */
  UINT8 PhyAddress;
} MEMAC_PHY_MAPPING;

/**
   Returns MAC ID Array and PHY type for given SerDes lane protocol

   @param[in] LaneProtocol      SerDes lane protocol representing a device

   @retval MemacId, NULL if lane protocol not found
   @param[in],@retval Size of Mac ID Array.
   Input is the maximum size allocated for MAC ID Array.
   Output is the actual number of MAC ids associated with a serdes protocol
   @retval PhyInterfaceType, NULL if lane protocol not found

 **/
VOID
GetMemacIdAndPhyType(IN     SERDES_PROTOCOL            LaneProtocol,
                     OUT    FMAN_MEMAC_ID        *MemacId,
                     IN OUT UINT8                *MemacIdCount,
                     OUT    PHY_INTERFACE_TYPE   *PhyInterfaceType);

/**
 * Table of mappings of FMAN MEMACs to PHYs
 */
STATIC CONST MEMAC_PHY_MAPPING gMemacToPhyMap[] = {
  [FM1_DTSEC_1] = {
    .MdioBus = &gDpaa1MdioBuses[0],
    .PhyAddress = QSGMII_PORT1_PHY_ADDR,
  },

  [FM1_DTSEC_2] = {
    .MdioBus = &gDpaa1MdioBuses[0],
    .PhyAddress = QSGMII_PORT2_PHY_ADDR,
  },

  [FM1_DTSEC_3] = {
    .MdioBus = &gDpaa1MdioBuses[0],
    .PhyAddress = RGMII_PHY1_ADDR,
  },

  [FM1_DTSEC_4] = {
    .MdioBus = &gDpaa1MdioBuses[0],
    .PhyAddress = RGMII_PHY2_ADDR,
  },

  [FM1_DTSEC_5] = {
    .MdioBus = &gDpaa1MdioBuses[0],
    .PhyAddress = QSGMII_PORT3_PHY_ADDR,
  },

  [FM1_DTSEC_6] = {
    .MdioBus = &gDpaa1MdioBuses[0],
    .PhyAddress = QSGMII_PORT4_PHY_ADDR,
  },

  [FM1_DTSEC_9] = {
    .MdioBus = &gDpaa1MdioBuses[1],
    .PhyAddress = FM1_10GEC1_PHY_ADDR,
  },
};

#endif /* __DPAA1_BOARD_LIB_H__ */
