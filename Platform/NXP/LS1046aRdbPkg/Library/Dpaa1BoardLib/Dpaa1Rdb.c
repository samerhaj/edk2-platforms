/** Dpaa1Rdb.c
  DPAA1 definitions specific for the RDB board

  Copyright (c) 2016, Freescale Semiconductor, Inc. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Base.h>
#include <Library/Dpaa1DebugLib.h>
#include <Library/IoLib.h>
#include <Library/Utils.h>
#include <Soc.h>

/*
 * Include board specific file for Flash timing
 */
#include <Dpaa1BoardLib.h>


/**
 * PHY MDIO buses
 */
DPAA1_PHY_MDIO_BUS gDpaa1MdioBuses[] = {
  [0] = {
    .Signature = DPAA1_PHY_MDIO_BUS_SIGNATURE,
    .IoRegs = (MEMAC_MDIO_BUS_REGS *)FixedPcdGet64(PcdDpaa1FmanMdio1Addr),
  },

  [1] = {
    .Signature = DPAA1_PHY_MDIO_BUS_SIGNATURE,
    .IoRegs = (MEMAC_MDIO_BUS_REGS *)FixedPcdGet64(PcdDpaa1FmanMdio2Addr),
  },
};

/**
   Returns MAC ID Array and PHY type for given SerDes lane protocol

   @param[in] LaneProtocol	SerDes lane protocol representing a device
 
   @retval MemacId, NULL if lane protocol not found
   @param[in],@retval Size of Mac ID Array.
   Input is the maximum size allocated for MAC ID Array.
   Output is the actual number of MAC ids associated with a serdes protocol
   @retval PhyInterfaceType, NULL if lane protocol not found

 **/
VOID
GetMemacIdAndPhyType(IN     SERDES_PROTOCOL      LaneProtocol,
                     OUT    FMAN_MEMAC_ID        *MemacId,
                     IN OUT UINT8                *MemacIdCount,
                     OUT    PHY_INTERFACE_TYPE   *PhyInterfaceType)
{
  ASSERT(*MemacIdCount >= MEMAC_ID_MAX_COUNT);

  if (LaneProtocol > NONE && LaneProtocol < SERDES_PRTCL_COUNT) {
    switch(LaneProtocol){
      case XFI_FM1_MAC9:
        *MemacId = FM1_DTSEC_9; // XFI on lane A, MAC 9
        *PhyInterfaceType = PHY_INTERFACE_XFI;
        *MemacIdCount = 1;
        break;
      case XFI_FM1_MAC10:
        *MemacId = FM1_DTSEC_10;
        *PhyInterfaceType = PHY_INTERFACE_XFI;
        *MemacIdCount = 1;
        break;
      case SGMII_FM1_DTSEC1:
        *MemacId = FM1_DTSEC_1;
        *PhyInterfaceType = PHY_INTERFACE_SGMII;
        *MemacIdCount = 1;
        break;
      case SGMII_FM1_DTSEC2:
        *MemacId = FM1_DTSEC_2;
        *PhyInterfaceType = PHY_INTERFACE_SGMII;
        *MemacIdCount = 1;
        break;
      case SGMII_FM1_DTSEC5:
        *MemacId = FM1_DTSEC_5;
        *PhyInterfaceType = PHY_INTERFACE_SGMII;
        *MemacIdCount = 1;
        break;
      case SGMII_FM1_DTSEC6:
        *MemacId = FM1_DTSEC_6;
        *PhyInterfaceType = PHY_INTERFACE_SGMII;
        *MemacIdCount = 1;
        break;
      case QSGMII_FM1_A: /* A indicates MACs 1,2,5,6 */
        *MemacId = FM1_DTSEC_1;
        *(++MemacId) = FM1_DTSEC_2;
        *(++MemacId) = FM1_DTSEC_5;
        *(++MemacId) = FM1_DTSEC_6;
        *PhyInterfaceType = PHY_INTERFACE_QSGMII;
        *MemacIdCount = 4;
        break;
      case SGMII_2500_FM1_DTSEC2:
        *MemacId = FM1_DTSEC_2;
        *PhyInterfaceType = PHY_INTERFACE_SGMII_2500;
        *MemacIdCount = 1;
        break;
      case SGMII_2500_FM1_DTSEC9:
        *MemacId = FM1_DTSEC_9;
        *PhyInterfaceType = PHY_INTERFACE_SGMII_2500;
        *MemacIdCount = 1;
        break;
      default:
        *MemacId = INVALID_FMAN_MEMAC_ID;
        *PhyInterfaceType = PHY_INTERFACE_NONE;
        *MemacIdCount = 0;
	 break;
    }
  }
}

CONST CHAR8 *SerdesPrtclToStr[] = {
  [NONE] = "NONE",
  [PCIE1] = "PCIE1",
  [PCIE2] = "PCIE2",
  [PCIE3] = "PCIE3",
  [SATA] = "SATA",
  [SGMII_FM1_DTSEC1] = "SGMII_FM1_DTSEC1",
  [SGMII_FM1_DTSEC2] = "SGMII_FM1_DTSEC2",
  [SGMII_FM1_DTSEC5] = "SGMII_FM1_DTSEC5",
  [SGMII_FM1_DTSEC6] = "SGMII_FM1_DTSEC6",
  [SGMII_FM1_DTSEC9] = "SGMII_FM1_DTSEC9",
  [QSGMII_FM1_A] = "QSGMII_FM1_A",        /* A indicates MACs 1,2,5,6 */
  [XFI_FM1_MAC9] = "XFI_FM1_MAC9",
  [XFI_FM1_MAC10] = "XFI_FM1_MAC10",
  [SGMII_2500_FM1_DTSEC2] = "SGMII_2500_FM1_DTSEC2",
  [SGMII_2500_FM1_DTSEC9] = "SGMII_2500_FM1_DTSEC9"
};

BOOLEAN
IsMemacEnabled (
  IN  FMAN_MEMAC_ID MemacId
  )
{
  UINT32 RegValue;
  CCSR_GUR *Gur = (VOID *)PcdGet64(PcdGutsBaseAddr);

  RegValue = MmioReadBe32((UINTN)&Gur->DevDisr2);
  //DPAA1_DEBUG_MSG(" IsMemacEnabled [%d] = 0x%x \n", MemacId, RegValue);
  return (!(RegValue & (1<<MemacId)));
}

/**
   SerDes lane probe callback

   @param[in] LaneProtocol	SerDes lane protocol representing a device
   @param[in] Arg		Memacs list pointer to be populated

   @retval Lane index, if found
   @retval -1, if not found

 **/
VOID
Dpaa1DiscoverFmanMemac(FMAN_MEMAC_ID MemacId,
                       PHY_INTERFACE_TYPE PhyInterfaceType,
                       VOID *Arg)
{
  if (MemacId != (FMAN_MEMAC_ID)INVALID_FMAN_MEMAC_ID &&
	PhyInterfaceType != (PHY_INTERFACE_TYPE)PHY_INTERFACE_NONE) {
    FmanMemacInit(MemacId,
                  PhyInterfaceType,
                  gMemacToPhyMap[MemacId].MdioBus,
                  gMemacToPhyMap[MemacId].PhyAddress,
                  Arg);
  }
}

VOID
Dpaa1ParseSerDes(SERDES_PROTOCOL LaneProtocol,
                 VOID *Arg)
{
  // QSGMII protocol combines 4 SGMII interfaces into one
  FMAN_MEMAC_ID MemacId[] = {(FMAN_MEMAC_ID)INVALID_FMAN_MEMAC_ID,
                             (FMAN_MEMAC_ID)INVALID_FMAN_MEMAC_ID,
                             (FMAN_MEMAC_ID)INVALID_FMAN_MEMAC_ID,
                             (FMAN_MEMAC_ID)INVALID_FMAN_MEMAC_ID};
  PHY_INTERFACE_TYPE PhyInterfaceType = (PHY_INTERFACE_TYPE)PHY_INTERFACE_NONE;
  UINT8              Index = 0;
  UINT8              MemacIdCount = ARRAY_SIZE(MemacId);

  GetMemacIdAndPhyType(LaneProtocol, &MemacId[0], &MemacIdCount ,&PhyInterfaceType);
  for (Index = 0; Index < MemacIdCount; Index++)
  {
    DPAA1_DEBUG_MSG("MemacId %d discovered on SerDes lane protocol %a"
                    "PhyInterfaceType %d \n",MemacId[Index],
                    SerdesPrtclToStr[LaneProtocol],PhyInterfaceType);
    Dpaa1DiscoverFmanMemac(MemacId[Index], PhyInterfaceType, Arg);
  }
}
