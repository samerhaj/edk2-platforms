/** Atheros.c
  Qualcomm's Atheros 8035 PHY services implementation

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


#define AR803x_PHY_SMART_EEE_CNTL3_REG 0x805d
#define AR803x_PHY_DEBUG_ADDR_REG      0x1d
#define AR803x_PHY_DEBUG_DATA_REG      0x1e

#define AR803x_DEBUG_REG_5             0x5
#define AR803x_RGMII_TX_CLK_DLY        0x100

#define AR803x_DEBUG_REG_0             0x0
#define AR803x_RGMII_RX_CLK_DLY        0x8000

#define AR803X_LPI_EN                  (1 << 8)

STATIC
VOID
Ar803xConfigureSmartEee (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  INT32  Enable
  )
{
  UINT32 Value;

  // 5.1.11 Smart_eee control3
  Value = Dpaa2PhyReadMmdIndirect (Dpaa2Phy, AR803x_PHY_SMART_EEE_CNTL3_REG, MDIO_MMD_PCS, MDIO_CTL_DEVAD_NONE);

  if (Enable) {
    Value |= AR803X_LPI_EN;
  } else {
    Value &= ~AR803X_LPI_EN;
    Dpaa2PhyWriteMmdIndirect (Dpaa2Phy, AR803x_PHY_SMART_EEE_CNTL3_REG, MDIO_MMD_PCS, MDIO_CTL_DEVAD_NONE, Value);
  }
}

EFI_STATUS
Ar8035PhyConfig (
  IN  DPAA2_PHY *Dpaa2Phy
  )
{
  UINT32 RegVal;

  /*  LX2160ARDB packet loss workaround: disabling Smart EEE */
  Ar803xConfigureSmartEee (Dpaa2Phy, 0);

  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0xd, 0x0007);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0xe, 0x8016);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0xd, 0x4007);

  RegVal = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0xe);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0xe, (RegVal|0x0018));

  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0x1d, 0x05);

  RegVal = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0x1e);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, 0x1e, (RegVal|0x0100));

  /* select debug reg 5 */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_CTL_DEVAD_NONE,
    AR803x_PHY_DEBUG_ADDR_REG,
    AR803x_DEBUG_REG_5);

  /* enable tx delay */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_CTL_DEVAD_NONE,
    AR803x_PHY_DEBUG_DATA_REG,
    AR803x_RGMII_TX_CLK_DLY);

  /* select debug reg 0 */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_CTL_DEVAD_NONE,
    AR803x_PHY_DEBUG_ADDR_REG,
    AR803x_DEBUG_REG_0);

  /* enable rx delay */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_CTL_DEVAD_NONE,
    AR803x_PHY_DEBUG_DATA_REG,
    AR803x_RGMII_RX_CLK_DLY);

  Dpaa2Phy->Support = PHY_GBIT_FEATURES;

  return EFI_SUCCESS;
}

/**
   Start Atheros PHY

   @param[in] Dpaa2Phy    Pointer to PHY object

   @Status EFI_SUCCESS    on success
   @Status error code     on failure

 **/
EFI_STATUS
Ar8035PhyStartup (
  IN  OUT  DPAA2_PHY *Dpaa2Phy
  )
{
  EFI_STATUS         Status;

  Status = UpdatePhyLink (Dpaa2Phy);
  if (Status) {
    return Status;
  }

  ParsePhyLink (Dpaa2Phy);

  return EFI_SUCCESS;
}
