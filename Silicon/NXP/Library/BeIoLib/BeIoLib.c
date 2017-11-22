/** BeIoLib.c

  Provide MMIO APIs for BE modules.

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Base.h>
#include <Library/BaseLib.h>
#include <Library/IoLib.h>

/**
  MmioRead8 for Big-Endian modules.

  @param  Address The MMIO register to read.

  @return The value read.

**/
UINT8
EFIAPI
BeMmioRead8 (
  IN  UINTN     Address
  )
{
  return MmioRead8 (Address);
}

/**
  MmioRead16 for Big-Endian modules.

  @param  Address The MMIO register to read.

  @return The value read.

**/
UINT16
EFIAPI
BeMmioRead16 (
  IN  UINTN     Address
  )
{
  return SwapBytes16 (MmioRead16 (Address));
}

/**
  MmioRead32 for Big-Endian modules.

  @param  Address The MMIO register to read.

  @return The value read.

**/
UINT32
EFIAPI
BeMmioRead32 (
  IN  UINTN     Address
  )
{
  return SwapBytes32 (MmioRead32 (Address));
}

/**
  MmioRead64 for Big-Endian modules.

  @param  Address The MMIO register to read.

  @return The value read.

**/
UINT64
EFIAPI
BeMmioRead64 (
  IN  UINTN     Address
  )
{
  return SwapBytes64 (MmioRead64 (Address));
}

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
  )
{
  return MmioWrite8 (Address, Value);
}

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
  )
{
  return MmioWrite16 (Address, SwapBytes16 (Value));
}

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
  )
{
  return MmioWrite32 (Address, SwapBytes32 (Value));
}

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
  )
{
  return MmioWrite64 (Address, SwapBytes64 (Value));
}

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
  )
{
  return MmioAndThenOr8 (Address, AndData, OrData);
}

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
  )
{
  AndData = SwapBytes16 (AndData);
  OrData = SwapBytes16 (OrData);

  return MmioAndThenOr16 (Address, AndData, OrData);
}

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
  )
{
  AndData = SwapBytes32 (AndData);
  OrData = SwapBytes32 (OrData);

  return MmioAndThenOr32 (Address, AndData, OrData);
}

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
  )
{
  AndData = SwapBytes64 (AndData);
  OrData = SwapBytes64 (OrData);

  return MmioAndThenOr64 (Address, AndData, OrData);
}

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
  )
{
  return MmioOr8 (Address, OrData);
}

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
  )
{
  return MmioOr16 (Address, SwapBytes16 (OrData));
}

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
  )
{
  return MmioOr32 (Address, SwapBytes32 (OrData));
}

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
  )
{
  return MmioOr64 (Address, SwapBytes64 (OrData));
}

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
  )
{
  return MmioAnd8 (Address, AndData);
}

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
  )
{
  return MmioAnd16 (Address, SwapBytes16 (AndData));
}

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
  )
{
  return MmioAnd32 (Address, SwapBytes32 (AndData));
}

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
  )
{
  return MmioAnd64 (Address, SwapBytes64 (AndData));
}
