/** BeIoLib.h
 *
 *  Copyright 2017 NXP
 *
 *  This program and the accompanying materials
 *  are licensed and made available under the terms and conditions of the BSD License
 *  which accompanies this distribution.  The full text of the license may be found at
 *  http://opensource.org/licenses/bsd-license.php
 *
 *  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
 *
 **/

#ifndef __BE_IOLIB_H__
#define __BE_IOLIB_H__

#include <Base.h>

/**
  MmioRead8 for Big-Endian modules.

  @param  Address The MMIO register to read.

  @return The value read.

**/
UINT8
EFIAPI
BeMmioRead8 (
  IN  UINTN     Address
  );

/**
  MmioRead16 for Big-Endian modules.

  @param  Address The MMIO register to read.

  @return The value read.

**/
UINT16
EFIAPI
BeMmioRead16 (
  IN  UINTN     Address
  );

/**
  MmioRead32 for Big-Endian modules.

  @param  Address The MMIO register to read.

  @return The value read.

**/
UINT32
EFIAPI
BeMmioRead32 (
  IN  UINTN     Address
  );

/**
  MmioRead64 for Big-Endian modules.

  @param  Address The MMIO register to read.

  @return The value read.

**/
UINT64
EFIAPI
BeMmioRead64 (
  IN  UINTN     Address
  );

/**
  MmioWrite8 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  Value   The value to write to the MMIO register.

**/
UINT8
EFIAPI
BeMmioWrite8 (
  IN  UINTN     Address,
  IN  UINT8     Value
  );

/**
  MmioWrite16 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  Value   The value to write to the MMIO register.

**/
UINT16
EFIAPI
BeMmioWrite16 (
  IN  UINTN     Address,
  IN  UINT16    Value
  );

/**
  MmioWrite32 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  Value   The value to write to the MMIO register.

**/
UINT32
EFIAPI
BeMmioWrite32 (
  IN  UINTN     Address,
  IN  UINT32    Value
  );

/**
  MmioWrite64 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  Value   The value to write to the MMIO register.

**/
UINT64
EFIAPI
BeMmioWrite64 (
  IN  UINTN     Address,
  IN  UINT64    Value
  );

/**
  MmioAndThenOr8 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  AndData The value to AND with the read value from the MMIO register.
  @param  OrData  The value to OR with the result of the AND operation.

  @return The value written back to the MMIO register.

**/
UINT8
EFIAPI
BeMmioAndThenOr8 (
  IN  UINTN     Address,
  IN  UINT8     AndData,
  IN  UINT8     OrData
  );

/**
  MmioAndThenOr16 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  AndData The value to AND with the read value from the MMIO register.
  @param  OrData  The value to OR with the result of the AND operation.

  @return The value written back to the MMIO register.

**/
UINT16
EFIAPI
BeMmioAndThenOr16 (
  IN  UINTN     Address,
  IN  UINT16    AndData,
  IN  UINT16    OrData
  );

/**
  MmioAndThenOr32 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  AndData The value to AND with the read value from the MMIO register.
  @param  OrData  The value to OR with the result of the AND operation.

  @return The value written back to the MMIO register.

**/
UINT32
EFIAPI
BeMmioAndThenOr32 (
  IN  UINTN     Address,
  IN  UINT32    AndData,
  IN  UINT32    OrData
  );

/**
  MmioAndThenOr64 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  AndData The value to AND with the read value from the MMIO register.
  @param  OrData  The value to OR with the result of the AND operation.

  @return The value written back to the MMIO register.

**/
UINT64
EFIAPI
BeMmioAndThenOr64 (
  IN  UINTN     Address,
  IN  UINT64    AndData,
  IN  UINT64    OrData
  );

/**
  MmioOr8 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  OrData  The value to OR with the read value from the MMIO register.

  @return The value written back to the MMIO register.

**/
UINT8
EFIAPI
BeMmioOr8 (
  IN  UINTN     Address,
  IN  UINT8     OrData
  );

/**
  MmioOr16 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  OrData  The value to OR with the read value from the MMIO register.

  @return The value written back to the MMIO register.

**/
UINT16
EFIAPI
BeMmioOr16 (
  IN  UINTN     Address,
  IN  UINT16    OrData
  );

/**
  MmioOr32 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  OrData  The value to OR with the read value from the MMIO register.

  @return The value written back to the MMIO register.

**/
UINT32
EFIAPI
BeMmioOr32 (
  IN  UINTN     Address,
  IN  UINT32    OrData
  );

/**
  MmioOr64 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  OrData  The value to OR with the read value from the MMIO register.

  @return The value written back to the MMIO register.

**/
UINT64
EFIAPI
BeMmioOr64 (
  IN  UINTN     Address,
  IN  UINT64    OrData
  );

/**
  MmioAnd8 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  AndData The value to AND with the read value from the MMIO register.

  @return The value written back to the MMIO register.

**/
UINT8
EFIAPI
BeMmioAnd8 (
  IN  UINTN     Address,
  IN  UINT8     AndData
  );

/**
  MmioAnd16 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  AndData The value to AND with the read value from the MMIO register.

  @return The value written back to the MMIO register.

**/
UINT16
EFIAPI
BeMmioAnd16 (
  IN  UINTN     Address,
  IN  UINT16    AndData
  );

/**
  MmioAnd32 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  AndData The value to AND with the read value from the MMIO register.

  @return The value written back to the MMIO register.

**/
UINT32
EFIAPI
BeMmioAnd32 (
  IN  UINTN     Address,
  IN  UINT32    AndData
  );

/**
  MmioAnd64 for Big-Endian modules.

  @param  Address The MMIO register to write.
  @param  AndData The value to AND with the read value from the MMIO register.

  @return The value written back to the MMIO register.

**/
UINT64
EFIAPI
BeMmioAnd64 (
  IN  UINTN     Address,
  IN  UINT64    AndData
  );

#endif /* _BE_IOLIB_H */
