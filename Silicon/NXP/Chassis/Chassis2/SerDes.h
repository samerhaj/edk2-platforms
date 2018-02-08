/** SerDes.h
 The Header file of SerDes Module for Chassis 2

 Copyright 2017 NXP

 This program and the accompanying materials
 are licensed and made available under the terms and conditions of the BSD License
 which accompanies this distribution. The full text of the license may be found
 http://opensource.org/licenses/bsd-license.php

 THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __SERDES_H__
#define __SERDES_H__

#include <Uefi/UefiBaseType.h>

#define SRDS_MAX_LANES     4

typedef enum {
  NONE = 0,
  PCIE1,
  PCIE2,
  PCIE3,
  SATA,
  SGMII_FM1_DTSEC1,
  SGMII_FM1_DTSEC2,
  SGMII_FM1_DTSEC5,
  SGMII_FM1_DTSEC6,
  SGMII_FM1_DTSEC9,
  SGMII_FM1_DTSEC10,
  QSGMII_FM1_A,
  XFI_FM1_MAC9,
  XFI_FM1_MAC10,
  SGMII_2500_FM1_DTSEC2,
  SGMII_2500_FM1_DTSEC5,
  SGMII_2500_FM1_DTSEC9,
  SGMII_2500_FM1_DTSEC10,
  SERDES_PRTCL_COUNT
} SERDES_PROTOCOL;

typedef enum {
  SRDS_1  = 0,
  SRDS_2,
  SRDS_MAX_NUM
} SERDES_NUMBER;

typedef struct {
  UINT16 Protocol;
  UINT8  SrdsLane[SRDS_MAX_LANES];
} SERDES_CONFIG;

typedef VOID
(*SERDES_PROBE_LANES_CALLBACK) (
  IN SERDES_PROTOCOL LaneProtocol,
  IN VOID *Arg
  );

VOID
SerDesProbeLanes(
  IN SERDES_PROBE_LANES_CALLBACK SerDesLaneProbeCallback,
  IN VOID *Arg
  );

#endif /* __SERDES_H */
