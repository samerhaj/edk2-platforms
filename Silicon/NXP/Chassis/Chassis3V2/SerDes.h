/** SerDes.h
 The Header file of SerDes Module for Chassis 3

 Copyright 2018 NXP

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

/**
 * SerDes lane protocols/devices
 */
typedef enum {
  NONE = 0,
  PCIE1,
  PCIE2,
  PCIE3,
  PCIE4,
  PCIE5,
  PCIE6,
  SATA1,
  SATA2,
  SATA3,
  SATA4, //10
  XFI1,
  XFI2,
  XFI3,
  XFI4,
  XFI5,
  XFI6,
  XFI7,
  XFI8,
  XFI9,
  XFI10, //20
  XFI11,
  XFI12,
  XFI13,
  XFI14,
  SGMII1, //25
  SGMII2,
  SGMII3,
  SGMII4,
  SGMII5,
  SGMII6, //30
  SGMII7,
  SGMII8,
  SGMII9,
  SGMII10,
  SGMII11, //35
  SGMII12,
  SGMII13,
  SGMII14,
  SGMII15,
  SGMII16, //40
  SGMII17,
  SGMII18,
  GE100_1,
  GE100_2,
  GE50_1, //45
  GE50_2,
  GE40_1,
  GE40_2,
  GE25_1,
  GE25_2, //50
  GE25_3,
  GE25_4,
  GE25_5,
  GE25_6,
  GE25_7,//55
  GE25_8,
  GE25_9,
  GE25_10, //59
  /*
   * Number of entries in this enum
   */
  SERDES_PRTCL_COUNT
} SERDES_PROTOCOL;

typedef enum {
  SRDS_1  = 0,
  SRDS_2,
  SRDS_3,
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
