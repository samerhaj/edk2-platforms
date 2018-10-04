/** Dpaa2LS2088aRdb.c
  DPAA2 definitions specific for the LS2088aRDB board

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/Dpaa2BoardSpecificLib.h>
#include <Library/Dpaa2EthernetMacLib.h>
#include <Library/Dpaa2EthernetPhyLib.h>
#include <Library/DpaaDebugLib.h>


/**
 * Mapping of WRIOP DPMACs to Ethernet PHYs
 */
typedef struct _DPMAC_PHY_MAPPING {
  /**
   * Pointer to the MDIO bus that connects a DPMAC to a PHY
   */
  DPAA2_PHY_MDIO_BUS *MdioBus;

  /**
   * PHY address of the associated PHY
   */
  UINT8 PhyAddress;

  /**
   * PHY media type:
   */
  PHY_MEDIA_TYPE PhyMediaType;

  /**
   * PHY Id of the associated PHY
   */
  UINT8 PhyId;
} DPMAC_PHY_MAPPING;

/**
 * PHY MDIO buses
 */
DPAA2_PHY_MDIO_BUS gDpaa2MdioBuses[] = {
  [0] = {
    .Signature = DPAA2_PHY_MDIO_BUS_SIGNATURE,
    .IoRegs = (MEMAC_MDIO_BUS_REGS *)DPAA2_WRIOP1_MDIO1_ADDR,
  },

  [1] = {
    .Signature = DPAA2_PHY_MDIO_BUS_SIGNATURE,
    .IoRegs = (MEMAC_MDIO_BUS_REGS *)DPAA2_WRIOP1_MDIO2_ADDR,
  },
};

/**
 * Table of mappings of WRIOP DPMACs to PHYs
 */
static const DPMAC_PHY_MAPPING gDpmacToPhyMap[] = {
  [WRIOP_DPMAC1] = {
    .MdioBus = &gDpaa2MdioBuses[0],
    .PhyAddress = CORTINA_PHY_ADDR1,
    .PhyMediaType = OPTICAL_PHY,
  },

  [WRIOP_DPMAC2] = {
    .MdioBus = &gDpaa2MdioBuses[0],
    .PhyAddress = CORTINA_PHY_ADDR2,
    .PhyMediaType = OPTICAL_PHY,
  },

  [WRIOP_DPMAC3] = {
    .MdioBus = &gDpaa2MdioBuses[0],
    .PhyAddress = CORTINA_PHY_ADDR3,
    .PhyMediaType = OPTICAL_PHY,
  },

  [WRIOP_DPMAC4] = {
    .MdioBus = &gDpaa2MdioBuses[0],
    .PhyAddress = CORTINA_PHY_ADDR4,
    .PhyMediaType = OPTICAL_PHY,
  },

  [WRIOP_DPMAC5] = {
    .MdioBus = &gDpaa2MdioBuses[1],
    .PhyAddress = AQUANTIA_PHY_ADDR1,
    .PhyMediaType = COPPER_PHY,
  },

  [WRIOP_DPMAC6] = {
    .MdioBus = &gDpaa2MdioBuses[1],
    .PhyAddress = AQUANTIA_PHY_ADDR2,
    .PhyMediaType = COPPER_PHY,
  },

  [WRIOP_DPMAC7] = {
    .MdioBus = &gDpaa2MdioBuses[1],
    .PhyAddress = AQUANTIA_PHY_ADDR3,
    .PhyMediaType = COPPER_PHY,
  },

  [WRIOP_DPMAC8] = {
    .MdioBus = &gDpaa2MdioBuses[1],
    .PhyAddress = AQUANTIA_PHY_ADDR4,
    .PhyMediaType = COPPER_PHY,
  },
};

/**
   SerDes lane probe callback

   @param[in] LaneProtocol  SerDes lane protocol representing a device

   @retval Lane index, if found
   @retval -1, if not found

 **/
VOID
Dpaa2DiscoverWriopDpmac (
  SERDES_PROTOCOL LaneProtocol,
  VOID *Arg
  )
{
  WRIOP_DPMAC_ID DpmacId;

  ASSERT (LaneProtocol != NONE);
  if (LaneProtocol >= XFI1 && LaneProtocol <= XFI8) {
    DpmacId = (WRIOP_DPMAC_ID)(LaneProtocol - XFI1 + 1);
    ASSERT (DpmacId < ARRAY_SIZE (gDpmacToPhyMap));
    WriopDpmacInit (DpmacId,
                   PHY_INTERFACE_XGMII,
                   gDpmacToPhyMap[DpmacId].MdioBus,
                   gDpmacToPhyMap[DpmacId].PhyAddress,
                   gDpmacToPhyMap[DpmacId].PhyMediaType,
                   gDpmacToPhyMap[DpmacId].PhyId,
                   Arg);
  }
}

VOID ProbeDpaaLanes (
  VOID *Arg
  )
{
  // Probe SerDes Lanes
  SerDesProbeLanes (Dpaa2DiscoverWriopDpmac, Arg);
}
