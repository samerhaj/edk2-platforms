/** @file
 The Header file of SerDes Module

 Copyright 2017 NXP

 This program and the accompanying materials
 are licensed and made available under the terms and conditions of the BSD License
 which accompanies this distribution. The full text of the license may be found
 http://opensource.org/licenses/bsd-license.php

 THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __LS1046A_SERDES_H__
#define __LS1046A_SERDES_H__

#include <SerDes.h>

SERDES_CONFIG SerDes1ConfigTbl[] = {
        /* SerDes 1 */
        {0x3333, {SGMII_FM1_DTSEC9, SGMII_FM1_DTSEC10, SGMII_FM1_DTSEC5, SGMII_FM1_DTSEC6 } },
        {0x1133, {XFI_FM1_MAC9, XFI_FM1_MAC10, SGMII_FM1_DTSEC5, SGMII_FM1_DTSEC6 } },
        {0x1333, {XFI_FM1_MAC9, SGMII_FM1_DTSEC10, SGMII_FM1_DTSEC5, SGMII_FM1_DTSEC6 } },
        {0x2333, {SGMII_2500_FM1_DTSEC9, SGMII_FM1_DTSEC10, SGMII_FM1_DTSEC5, SGMII_FM1_DTSEC6 } },
        {0x2233, {SGMII_2500_FM1_DTSEC9, SGMII_2500_FM1_DTSEC10, SGMII_FM1_DTSEC5, SGMII_FM1_DTSEC6 } },
        {0x1040, {XFI_FM1_MAC9, NONE, QSGMII_FM1_A, NONE } },
        {0x2040, {SGMII_2500_FM1_DTSEC9, NONE, QSGMII_FM1_A, NONE } },
        {0x1163, {XFI_FM1_MAC9, XFI_FM1_MAC10, PCIE1, SGMII_FM1_DTSEC6 } },
        {0x2263, {SGMII_2500_FM1_DTSEC9, SGMII_2500_FM1_DTSEC10, PCIE1, SGMII_FM1_DTSEC6 } },
        {0x3363, {SGMII_FM1_DTSEC5, SGMII_FM1_DTSEC6, PCIE1, SGMII_FM1_DTSEC6 } },
        {0x2223, {SGMII_2500_FM1_DTSEC9, SGMII_2500_FM1_DTSEC10, SGMII_2500_FM1_DTSEC5, SGMII_FM1_DTSEC6 } },
        {}
};

SERDES_CONFIG SerDes2ConfigTbl[] = {
        /* SerDes 2 */
        {0x8888, {PCIE1, PCIE1, PCIE1, PCIE1 } },
        {0x5559, {PCIE1, PCIE2, PCIE3, SATA } },
        {0x5577, {PCIE1, PCIE2, PCIE3, PCIE3 } },
        {0x5506, {PCIE1, PCIE2, NONE, PCIE3 } },
        {0x0506, {NONE, PCIE2, NONE, PCIE3 } },
        {0x0559, {NONE, PCIE2, PCIE3, SATA } },
        {0x5A59, {PCIE1, SGMII_FM1_DTSEC2, PCIE3, SATA } },
        {0x5A06, {PCIE1, SGMII_FM1_DTSEC2, NONE, PCIE3 } },
        {}
};

SERDES_CONFIG *SerDesConfigTbl[] = {
        SerDes1ConfigTbl,
        SerDes2ConfigTbl
};

#endif /* __LS1046A_SERDES_H */
