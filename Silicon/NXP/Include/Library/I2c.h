/** @I2c.h
  Header defining the constant, base address amd function for I2C controller

  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __I2C_H___
#define __I2C_H__

#include <Uefi.h>
#include <Library/TimerLib.h>

///
/// Define the I2C flags
///
/// I2C read operation when set
#define I2C_READ_FLAG         0x1
#define I2C_WRITE_FLAG        0x2

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

#define I2C_IFDR_EARLY_DIV    0x7E

#define I2C1                  0
#define I2C2                  1
#define I2C3                  2
#define I2C4                  3

#define I2C_SPEED             100000

/**
  Record defining i2c registers
**/

typedef struct I2cRegs {
  UINT8     I2cAdr;
  UINT8     I2cFdr;
  UINT8     I2cCr;
  UINT8     I2cSr;
  UINT8     I2cDr;
}I2cRegs_t;

/**
  Function to initialize i2c bus

  @param   I2cBus     I2c Controller number
  @param   Speed      Value to be set

  @retval  EFI_SUCCESS
**/
EFI_STATUS
EFIAPI
I2cBusInit (
  IN   UINT32   I2cBus,
  IN   UINT32    Speed
  );

/**
  Function to read data usin i2c

  @param   I2cBus     I2c Controller number
  @param   Chip       Address of slave device from where data to be read
  @param   Offset     Offset of slave memory
  @param   Alen       Address length of slave
  @param   Buffer     A pointer to the destination buffer for the data
  @param   Len        Length of data to be read

  @retval  EFI_NOT_READY    Arbitration lost
  @retval  EFI_TIMEOUT      Failed to initialize data transfer in predefined time
  @retval  EFI_NOT_FOUND    ACK was not recieved
  @retval  EFI_SUCCESS      Read was successful

**/
EFI_STATUS
I2cDataRead (
  IN   UINT32  I2cBus,
  IN   UINT8    Chip,
  IN   UINT32  Offset,
  IN   UINT32  Alen,
  OUT  UINT8  *Buffer,
  IN   UINT32  Len
  );

/**
  Function to set I2c bus speed

  @param   I2cBus           I2c Controller number
  @param   Speed            Value to be set

  @retval  EFI_SUCCESS
**/
EFI_STATUS
EFIAPI
I2cSetBusSpeed (
  IN   UINT32   I2cBus,
  IN   UINT32    Speed
  );

/**
  Function to stop transaction on i2c bus

  @param   I2cRegs          Pointer to i2c registers

  @retval  EFI_NOT_READY    Arbitration was lost
  @retval  EFI_TIMEOUT      Timeout occured
  @retval  EFI_SUCCESS      Stop operation was successful

**/
EFI_STATUS
I2cStop (
  IN   struct I2cRegs       *I2cRegs
  );

/**
  Function to initiate data transfer on i2c bus

  @param   I2cRegs   Pointer to i2c base registers
  @param   Chip      Chip Address
  @param   Offset    Slave memory's offset
  @param   Alen      length of chip address

  @retval  EFI_NOT_READY           Arbitration lost
  @retval  EFI_TIMEOUT             Failed to initialize data transfer in predefined time
  @retval  EFI_NOT_FOUND           ACK was not recieved
  @retval  EFI_SUCCESS             Read was successful

**/
EFI_STATUS
I2cBusInitTransfer (
  IN   struct I2cRegs       *I2cRegs,
  IN   UINT8                Chip,
  IN   UINT32               Offset,
  IN   INT32                Alen
  );


/**
  Function to write data using i2c bus

  @param   I2cBus                  I2c Controller number
  @param   Chip                    Address of slave device where data to be written
  @param   Offset                  Offset of slave memory
  @param   Alen                    Address length of slave
  @param   Buffer                  A pointer to the source buffer for the data
  @param   Len                     Length of data to be write

  @retval  EFI_NOT_READY           Arbitration lost
  @retval  EFI_TIMEOUT             Failed to initialize data transfer in predefined time
  @retval  EFI_NOT_FOUND           ACK was not recieved
  @retval  EFI_SUCCESS             Read was successful

**/
EFI_STATUS
I2cDataWrite (
  IN   UINT32        I2cBus,
  IN   UINT8         Chip,
  IN   UINT32        Offset,
  IN   INT32         Alen,
  OUT  UINT8         *Buffer,
  IN   INT32         Len
  );


/**
  Function to reset I2c
  @param   I2cBus                    I2c Controller number

  @return  VOID
**/
VOID
I2cReset (
  UINT32 I2cBus
  );

/**
  Function to Probe I2c slave device
  @param    I2cBus                    I2c Controller number

  @retval   EFI_INVALID_PARAMETER     Invalid I2c Controller number
  @retval   EFI_SUCCESS               I2c device probed successfully

**/
EFI_STATUS
EFIAPI
I2cProbeDevices (
  IN   INT16    I2cBus,
  IN   UINT8    Chip
  );

/**
  Declare I2C registers' region as runtime Space

  @param  I2cBus      I2c Controller number
**/
EFI_STATUS
EFIAPI
I2cBusDeclareRuntime (
  IN   UINT32  I2cBus
  );

/**
  Convert I2C registers' runtime region to virtual address

  @param  I2cBus      I2c Controller number
**/
VOID
EFIAPI
I2cBusVirtualNotify (
  IN   UINT32  I2cBus
  );

/**
  Function to early initialize i2c bus to prepare read the clock through i2c

  @param   I2cBus     I2c Controller number
  @param   Speed      Value to be set

  @retval  EFI_SUCCESS
**/
VOID
EFIAPI
I2cBusEarlyInit (
  IN   UINT32    I2cBus
  );

#endif
