/** Qualcomm.c
 Qualcomm  Phy 8035 PHY services implementation

  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/
#include <Library/DpaaDebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>

#include "Dpaa2EthernetPhyPrivate.h"


#define AR803x_PHY_DEBUG_ADDR_REG 0x1d
#define AR803x_PHY_DEBUG_DATA_REG 0x1e

#define AR803x_DEBUG_REG_5        0x5
#define AR803x_RGMII_TX_CLK_DLY   0x100

#define AR803x_DEBUG_REG_0        0x0
#define AR803x_RGMII_RX_CLK_DLY   0x8000

EFI_STATUS QC8035PhyConfig(DPAA2_PHY *Dpaa2Phy)
{
 UINT32 RegVal;
 return 0;
 Dpaa2PhyRegisterWrite(Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0xd, 0x0007);
 Dpaa2PhyRegisterWrite(Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0xe, 0x8016);
 Dpaa2PhyRegisterWrite(Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0xd, 0x4007);
 RegVal = Dpaa2PhyRegisterRead(Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0xe);
 Dpaa2PhyRegisterWrite(Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0xe, (RegVal|0x0018));

 Dpaa2PhyRegisterWrite(Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0x1d, 0x05);
 RegVal = Dpaa2PhyRegisterRead(Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0x1e);
 Dpaa2PhyRegisterWrite(Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0x1e, (RegVal|0x0100));
 /* select debug reg 5 */
 Dpaa2PhyRegisterWrite(Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0x1D, 0x5);
 /* enable tx delay */
 Dpaa2PhyRegisterWrite(Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0x1E, 0x0100);

 /* select debug reg 0 */
 Dpaa2PhyRegisterWrite(Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0x1D, 0x0);
 /* enable rx delay */
 Dpaa2PhyRegisterWrite(Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0x1E, 0x8000);

// Dpaa2Phy->supported = Dpaa2Phy->drv->features;

// genphy_config_aneg(Dpaa2Phy);
// genphy_restart_aneg(Dpaa2Phy);

 return 0;
}

