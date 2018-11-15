/** @MmcHost.h

  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __MMC_HOST_DXE_H__
#define __MMC_HOST_DXE_H__

#include <Protocol/MmcHost.h>

#define MMC_DEVICE_SIGNATURE                 SIGNATURE_32('s','d','x','c')

#define MMC_DEVICE_INSTANCE_FROM_HOST(a)    CR(a, MMC_DEVICE_INSTANCE, MmcHostProtocol, MMC_DEVICE_SIGNATURE)

#define EXTCSD_BUS_WIDTH        183

#define EMMC_BUS_WIDTH_DDR_4BIT 5
#define EMMC_BUS_WIDTH_DDR_8BIT 6

typedef enum {
  UNKNOWN_CARD,
  MMC_CARD,              //MMC card
  MMC_CARD_HIGH,         //MMC Card with High capacity
  EMMC_CARD,             //eMMC card
  SD_CARD,               //SD card
} CARD_TYPE;

typedef struct _MMC_DEVICE_INSTANCE    MMC_DEVICE_INSTANCE;

struct _MMC_DEVICE_INSTANCE {
  UINT32                             Signature;
  CARD_TYPE                          CardType;
  UINTN                              DeviceBaseAddress;
  MMC_CMD_INFO                       Cmd;
  EFI_MMC_HOST_PROTOCOL              MmcHostProtocol;
};

#endif /* __MMC_HOST_DXE_H__ */
