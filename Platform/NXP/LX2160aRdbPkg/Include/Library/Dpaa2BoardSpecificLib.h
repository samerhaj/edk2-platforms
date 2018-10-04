/** Dpaa2BoardSpecificLib.h
  DPAA2 Board Specific library interface

  Copyright 2017-2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __DPAA2_BOARD_SPECIFIC_LIB_H__
#define __DPAA2_BOARD_SPECIFIC_LIB_H__

#include <Library/Dpaa2EthernetPhyLib.h>
#include <Library/PcdLib.h>
#include <SerDes.h>
#include <Uefi.h>

/**
 * Number of Ethernet PHY MDIO buses
 */
#define DPAA2_MDIO_BUSES_COUNT                  FixedPcdGet8  (PcdMdioBustCount)
#define DPAA2_WRIOP1_MDIO1_ADDR                 FixedPcdGet64 (PcdDpaa2Wriop1Mdio1Addr)
#define DPAA2_WRIOP1_MDIO2_ADDR                 FixedPcdGet64 (PcdDpaa2Wriop1Mdio2Addr)

/*
 * DPAA2 Ethernet PHY MDIO addresses
 */
#define CORTINA_PHY_ADDR1   0x0
#define CORTINA_PHY_ADDR2   0x11
#define CORTINA_PHY_ADDR3   0x12
#define CORTINA_PHY_ADDR4   0x13

#define AQUANTIA_PHY_ADDR1  0x4
#define AQUANTIA_PHY_ADDR2  0x5

#define QC_PHY_ADDR1  0x1
#define QC_PHY_ADDR2  0x2


VOID
Dpaa2DiscoverWriopDpmac (
  SERDES_PROTOCOL LaneProtocol,
  VOID *Arg
  );

VOID ProbeDpaaLanes (
  VOID *Arg
  );

//extern DPAA2_PHY_MDIO_BUS gDpaa2MdioBuses[];

#endif /* __DPAA2_BOARD_SPECIFIC_LIB_H__ */

