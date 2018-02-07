/** SerDes.h
 The Header file of SerDes Module for Chassis 3

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

#define SRDS_MAX_LANES    8

//
// SerDes lane protocols/devices
//
typedef enum {
  NONE = 0,
  PCIE1,
  PCIE2,
  PCIE3,
  PCIE4,
  SATA1,
  SATA2,
  XAUI1,
  XAUI2,
  XFI1,
  XFI2,
  XFI3,
  XFI4,
  XFI5,
  XFI6,
  XFI7,
  XFI8,
  SGMII1,
  SGMII2,
  SGMII3,
  SGMII4,
  SGMII5,
  SGMII6,
  SGMII7,
  SGMII8,
  SGMII9,
  SGMII10,
  SGMII11,
  SGMII12,
  SGMII13,
  SGMII14,
  SGMII15,
  SGMII16,
  QSGMII_A,
  QSGMII_B,
  QSGMII_C,
  QSGMII_D,
  // Number of entries in this enum
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
