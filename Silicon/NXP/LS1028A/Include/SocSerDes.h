/** @file
 The Header file of SerDes Module

 Copyright 2019 NXP

 This program and the accompanying materials
 are licensed and made available under the terms and conditions of the BSD License
 which accompanies this distribution. The full text of the license may be found
 http://opensource.org/licenses/bsd-license.php

 THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __SOC_SERDES_H__
#define __SOC_SERDES_H__

#include <SerDes.h>

SERDES_CONFIG SerDes1ConfigTbl[] = {
        /* SerDes 1 */
        {0xE031, {SXGMII1, QXGMII2, NONE, SATA1} },
        {0xB991, {SXGMII1, SGMII1, SGMII2, PCIE1} },
        {0xBB31, {SXGMII1, QXGMII2, PCIE1, PCIE1} },
        {0xCC31, {SXGMII1, QXGMII2, PCIE2, PCIE2} },
        {0xBB51, {SXGMII1, QSGMII_B, PCIE2, PCIE1} },
        {0xCC51, {SXGMII1, QSGMII_B, PCIE2, PCIE2} },
        {0xCC3B, {PCIE1, QXGMII2, PCIE2, PCIE2} },
        {0xCC5B, {PCIE1, QSGMII_B, PCIE2, PCIE2} },
        {0xBB38, {SGMII_T1, QXGMII2, PCIE2, PCIE1} },
        {0xCC38, {SGMII_T1, QXGMII2, PCIE2, PCIE2} },
        {0xBB58, {SGMII_T1, QSGMII_B, PCIE2, PCIE1} },
        {0xEB99, {SGMII1, SGMII1, PCIE2, SATA1} },
        {0xCC99, {SGMII1, SGMII1, PCIE2, PCIE2} },
        {0xBB99, {SGMII1, SGMII1, PCIE2, PCIE1} },
        {0xCC58, {SGMII_T1, QSGMII_B, PCIE2, PCIE2} },
        {0xCC8B, {PCIE1, SGMII_T1, PCIE2, PCIE2} },
        {0x9999, {SGMII1, SGMII2, SGMII3, SGMII4} },
        {0xEB58, {SGMII_T1, QSGMII_B, PCIE2, SATA1} },
        {0xEB8B, {PCIE1, SGMII_T1, PCIE2, SATA1} },
        {0xE8CC, {PCIE1, PCIE1, SGMII_T1, SATA1} },
        {0xEBCC, {PCIE1, PCIE1, PCIE2, SATA1} },
        {0xCCCC, {PCIE1, PCIE1, PCIE2, PCIE2} },
        {0xDDDD, {PCIE1, PCIE1, PCIE1, PCIE1} },
        {}
};

SERDES_CONFIG *SerDesConfigTbl[] = {
        SerDes1ConfigTbl,
};

#endif /* __SOC_SERDES_H */
