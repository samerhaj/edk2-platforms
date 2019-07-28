/** Dpaa2LX2160aCex7.c
  DPAA2 definitions specific for the LX2160aCEX7 board

  Copyright 2017-2018 NXP

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
// Checked dpmac 2, 3, 4
// to be done 5 and 6 for inphy
// 17 and 18 for rgmii

static const DPMAC_PHY_MAPPING gDpmacToPhyMap[] = {

  [WRIOP_DPMAC17] = {
    .MdioBus = &gDpaa2MdioBuses[0],
    .PhyAddress = QC_PHY_ADDR1,
    .PhyMediaType = COPPER_PHY,
    .PhyId        = QC_PHY,
  },
};

STATIC VOID GetDpMacId (
  SERDES_PROTOCOL LaneProtocol,
  OUT WRIOP_DPMAC_ID *DpmacId
  )
{
  switch(LaneProtocol) {
    case XFI1...XFI14:
      *DpmacId = (WRIOP_DPMAC_ID)(LaneProtocol - XFI1 + 1);
       break;
    case SGMII1...SGMII18:
      *DpmacId = (WRIOP_DPMAC_ID)(LaneProtocol - SGMII1 + 1);
       break;
    case GE100_1...GE100_2:
      *DpmacId = (WRIOP_DPMAC_ID)(LaneProtocol - GE100_1 + 1);
       break;
    case GE50_1...GE50_2:
      *DpmacId = (WRIOP_DPMAC_ID)(LaneProtocol - GE50_1 + 1);
       break;
    case GE40_1...GE40_2:
      *DpmacId = (WRIOP_DPMAC_ID)(LaneProtocol - GE40_1 + 1);
       break;
    case GE25_1...GE25_10:
      *DpmacId = (WRIOP_DPMAC_ID)(LaneProtocol - GE25_1 + 1);
       break;
    default:
      *DpmacId = 0;
      break;
  }
}
/**
   SerDes lane probe callback

   @param[in] LaneProtocol  SerDes lane protocol representing a device

   @retval Lane index, if found
   @retval -1, if not found

 **/
STATIC UINT32 AssignedMac[NUM_WRIOP_DPMACS] = {0};

VOID
Dpaa2DiscoverWriopDpmac (
  SERDES_PROTOCOL LaneProtocol,
  VOID *Arg
  )
{
  WRIOP_DPMAC_ID DpmacId;
  ASSERT (LaneProtocol != NONE);

  return;
  if (LaneProtocol >= XFI1 && LaneProtocol <= XFI14) {
    GetDpMacId(LaneProtocol, &DpmacId);
    if (!DpmacId || AssignedMac[DpmacId]) {
      DEBUG((DEBUG_INFO, "Lane protocol %d, has mac %d, New Lane %d\n",
                           AssignedMac[DpmacId], DpmacId, LaneProtocol));
      return;
    }
    AssignedMac[DpmacId] = LaneProtocol;
    ASSERT (DpmacId < ARRAY_SIZE (gDpmacToPhyMap));

    if (DpmacId == WRIOP_DPMAC3 || DpmacId == WRIOP_DPMAC4) {
      WriopDpmacInit (DpmacId,
                     PHY_INTERFACE_XGMII,
                     gDpmacToPhyMap[DpmacId].MdioBus,
                     gDpmacToPhyMap[DpmacId].PhyAddress,
                     gDpmacToPhyMap[DpmacId].PhyMediaType,
                     gDpmacToPhyMap[DpmacId].PhyId,
                     Arg);
    }
  }

  if (LaneProtocol >= GE25_5 && LaneProtocol <= GE25_6) {
    GetDpMacId(LaneProtocol, &DpmacId);
    if (!DpmacId || AssignedMac[DpmacId]) {
      DEBUG((DEBUG_INFO, "Lane protocol %d, has mac %d, New Lane %d\n",
                           AssignedMac[DpmacId], DpmacId, LaneProtocol));
      return;
    }
    AssignedMac[DpmacId] = LaneProtocol;
    ASSERT (DpmacId < ARRAY_SIZE (gDpmacToPhyMap));

    if (DpmacId == WRIOP_DPMAC5 || DpmacId == WRIOP_DPMAC6 ) {
      WriopDpmacInit (DpmacId,
                     PHY_INTERFACE_25G_AUI,
                     gDpmacToPhyMap[DpmacId].MdioBus,
                     gDpmacToPhyMap[DpmacId].PhyAddress,
                     gDpmacToPhyMap[DpmacId].PhyMediaType,
                     gDpmacToPhyMap[DpmacId].PhyId,
                     Arg);
    }
  }
}

VOID ProbeDpaaLanes (
  VOID *Arg
  )
{
   WRIOP_DPMAC_ID DpmacId;
  // Probe SerDes Lanes
  SerDesProbeLanes (Dpaa2DiscoverWriopDpmac, Arg);
   for (DpmacId = WRIOP_DPMAC17; DpmacId < WRIOP_DPMAC18; DpmacId++) {
    WriopDpmacInit (DpmacId,
                   PHY_INTERFACE_RGMII,
                   gDpmacToPhyMap[DpmacId].MdioBus,
                   gDpmacToPhyMap[DpmacId].PhyAddress,
                   gDpmacToPhyMap[DpmacId].PhyMediaType,
                   gDpmacToPhyMap[DpmacId].PhyId,
                   Arg);
  }
}
