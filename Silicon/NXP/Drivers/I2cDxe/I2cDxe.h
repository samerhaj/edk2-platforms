/** I2cDxe.h
  Header defining the constant, base address amd function for I2C controller

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __I2C_DXE_H___
#define __I2C_DXE_H__

#include <Uefi.h>

#define I2C_CR_IIEN           (1 << 6)
#define I2C_CR_MSTA           (1 << 5)
#define I2C_CR_MTX            (1 << 4)
#define I2C_CR_TX_NO_AK       (1 << 3)
#define I2C_CR_RSTA           (1 << 2)

#define I2C_SR_ICF            (1 << 7)
#define I2C_SR_IBB            (1 << 5)
#define I2C_SR_IAL            (1 << 4)
#define I2C_SR_IIF            (1 << 1)
#define I2C_SR_RX_NO_AK       (1 << 0)

#define I2C_CR_IEN            (0 << 7)
#define I2C_CR_IDIS           (1 << 7)
#define I2C_SR_IIF_CLEAR      (1 << 1)

#define BUS_IDLE              (0 | (I2C_SR_IBB << 8))
#define BUS_BUSY              (I2C_SR_IBB | (I2C_SR_IBB << 8))
#define IIF                   (I2C_SR_IIF | (I2C_SR_IIF << 8))

#define I2C_FLAG_WRITE        0x0

typedef struct {
  VENDOR_DEVICE_PATH        Guid;
  EFI_DEVICE_PATH_PROTOCOL  End;
} I2C_DEVICE_PATH;

/**
  Record defining i2c registers
**/
typedef struct {
  UINT8     I2cAdr;
  UINT8     I2cFdr;
  UINT8     I2cCr;
  UINT8     I2cSr;
  UINT8     I2cDr;
} I2C_REGS ;

extern
UINT64
GetBusFrequency (
  VOID
  );

#endif
