/** @file
  PCI Segment Library for NXP SoCs with multiple RCs

  Copyright 2018 NXP

  This program and the accompanying materials are
  licensed and made available under the terms and conditions of
  the BSD License which accompanies this distribution.  The full
  text of the license may be found at
  http://opensource.org/licenses/bsd-license.php.

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/
#include <PiDxe.h>
#include <Base.h>
#include <Library/PciSegmentLib.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Pcie.h>

typedef enum {
  PciCfgWidthUint8      = 0,
  PciCfgWidthUint16,
  PciCfgWidthUint32,
  PciCfgWidthMax
} PCI_CFG_WIDTH;

/**
  Assert the validity of a PCI Segment address.
  A valid PCI Segment address should not contain 1's in bits 28..31 and 48..63

  @param  A The address to validate.
  @param  M Additional bits to assert to be zero.

**/
#define ASSERT_INVALID_PCI_SEGMENT_ADDRESS(A,M) \
  ASSERT (((A) & (0xffff0000f0000000ULL | (M))) == 0)

/**
  Function to return PCIe Physical Address(PCIe view) or Controller
  Address(CPU view) for different RCs

  @param  Address Address passed from bus layer.
  @param  Segment Segment number for Root Complex.

  @return Return PCIe CPU or Controller address.

**/
STATIC
UINT64
PciSegmentLibGetConfigBase (
  IN  UINT64      Address,
  IN  UINT16      Segment
  )
{

  switch (Segment) {
    // Root Complex 1
    case PCI_SEG0_NUM:
      // Reading bus number(bits 20-27)
      if ((Address >> 20) & 1) {
        return PCI_SEG0_MMIO_MEMBASE;
      } else {
        // On Bus 0 RCs are connected
        return PCI_SEG0_DBI_BASE;
      }
    // Root Complex 2
    case PCI_SEG1_NUM:
      // Reading bus number(bits 20-27)
      if ((Address >> 20) & 1) {
        return PCI_SEG1_MMIO_MEMBASE;
      } else {
        // On Bus 0 RCs are connected
        return PCI_SEG1_DBI_BASE;
      }
    // Root Complex 3
    case PCI_SEG2_NUM:
      // Reading bus number(bits 20-27)
      if ((Address >> 20) & 1) {
        return PCI_SEG2_MMIO_MEMBASE;
      } else {
        // On Bus 0 RCs are connected
        return PCI_SEG2_DBI_BASE;
      }
    // Root Complex 4
    case PCI_SEG3_NUM:
      // Reading bus number(bits 20-27)
      if ((Address >> 20) & 1) {
        return PCI_SEG3_MMIO_MEMBASE;
      } else {
        // On Bus 0 RCs are connected
        return PCI_SEG3_DBI_BASE;
      }
    // Root Complex 5
    case PCI_SEG4_NUM:
      // Reading bus number(bits 20-27)
      if ((Address >> 20) & 1) {
        return PCI_SEG4_MMIO_MEMBASE;
      } else {
        // On Bus 0 RCs are connected
        return PCI_SEG4_DBI_BASE;
      }
    // Root Complex 6
    case PCI_SEG5_NUM:
      // Reading bus number(bits 20-27)
      if ((Address >> 20) & 1) {
        return PCI_SEG5_MMIO_MEMBASE;
      } else {
        // On Bus 0 RCs are connected
        return PCI_SEG5_DBI_BASE;
      }
    default:
      return 0;
  }

}

/**
  Function to select page among the 48 1KB pages for
  AXI access

  @param[in]  Dbi    GPEX host controller address.
  @param[in]  PgIdx  The page index to select

**/
STATIC
VOID
CcsrSetPg (
  IN EFI_PHYSICAL_ADDRESS Dbi,
  IN UINT8 PgIdx
  )
{
  UINT32 Val;
  Val = MmioRead32 (Dbi + PAB_CTRL);
  // Bit 18:13 of Bridge Control Register(PAB) denotes page select
  // Mask is 6 bits and shift is 13 to select page
  Val &= ~(PAB_CTRL_PAGE_SEL_MASK << PAB_CTRL_PAGE_SEL_SHIFT);
  Val |= (PgIdx & PAB_CTRL_PAGE_SEL_MASK) << PAB_CTRL_PAGE_SEL_SHIFT;

  MmioWrite32 (Dbi + PAB_CTRL, Val);
}

/**
  Internal worker function to read a PCI configuration register.

  @param  Address The address that encodes the Segment, PCI Bus, Device,
                  Function and Register.
  @param  Width   The width of data to read

  @return The value read from the PCI configuration register.

**/
STATIC
UINT32
PciSegmentLibReadWorker (
  IN  UINT64                      Address,
  IN  PCI_CFG_WIDTH               Width
  )
{
  UINT64    Base;
  UINT16    Offset;
  UINT16    Segment;

  //
  // Reading Segment number(47-32) bits in Address
  //
  Segment = (Address >> 32);
  //
  // Reading Function(12-0) bits in Address
  //
  Offset = (Address & 0xfff );

  Base = PciSegmentLibGetConfigBase (Address, Segment);

  if (PcdGetBool (PcdPcieConfigurePex)) {
    if ((Address >> 20) & 1) {
      if (Offset < INDIRECT_ADDR_BNDRY) {
        CcsrSetPg ((UINTN)Base, 0);
      } else {
        CcsrSetPg ((UINTN)Base, OFFSET_TO_PAGE_IDX (Offset));
        Offset = OFFSET_TO_PAGE_ADDR (Offset);
      }
    }
  }

  //
  // ignore devices > 0 on bus 0
  //
  if ((Address & 0xff00000) == 0 && (Address & 0xf8000) != 0) {
    return MAX_UINT32;
  }

  //
  // ignore device > 0 on bus 1
  //
  if ((Address & 0xfe00000) == 0 && (Address & 0xf8000) != 0) {
    return MAX_UINT32;
  }

  switch (Width) {
  case PciCfgWidthUint8:
    return MmioRead8 (Base + (UINT8)Offset);
  case PciCfgWidthUint16:
    return MmioRead16 (Base + (UINT16)Offset);
  case PciCfgWidthUint32:
    return MmioRead32 (Base + (UINT32)Offset);
  default:
    ASSERT (FALSE);
  }

  return CHAR_NULL;
}

/**
  Internal worker function to writes a PCI configuration register.

  @param  Address The address that encodes the Segment, PCI Bus, Device,
                  Function and Register.
  @param  Width   The width of data to write
  @param  Data    The value to write.

  @return The value written to the PCI configuration register.

**/
STATIC
UINT32
PciSegmentLibWriteWorker (
  IN  UINT64                      Address,
  IN  PCI_CFG_WIDTH               Width,
  IN  UINT32                      Data
  )
{
  UINT64    Base;
  UINT32    Offset;
  UINT16    Segment;

  //
  // Reading Segment number(47-32 bits) in Address
  Segment = (Address >> 32);
  //
  // Reading Function(12-0 bits) in Address
  //
  Offset = (Address & 0xfff );

  Base = PciSegmentLibGetConfigBase (Address, Segment);

  if (PcdGetBool (PcdPcieConfigurePex)) {
    if ((Address >> 20) & 1) {
      if (Offset < INDIRECT_ADDR_BNDRY) {
        CcsrSetPg ((UINTN)Base, 0);
      } else {
        CcsrSetPg ((UINTN)Base, OFFSET_TO_PAGE_IDX (Offset));
        Offset = OFFSET_TO_PAGE_ADDR (Offset);
      }
    }
  }

  //
  // ignore devices > 0 on bus 0
  //
  if ((Address & 0xff00000) == 0 && (Address & 0xf8000) != 0) {
    return Data;
  }

  //
  // ignore device > 0 on bus 1
  //
  if ((Address & 0xfe00000) == 0 && (Address & 0xf8000) != 0) {
    return MAX_UINT32;
  }

  switch (Width) {
  case PciCfgWidthUint8:
    MmioWrite8 (Base + (UINT8)Offset, Data);
    break;
  case PciCfgWidthUint16:
    MmioWrite16 (Base + (UINT16)Offset, Data);
    break;
  case PciCfgWidthUint32:
    MmioWrite32 (Base + (UINT16)Offset, Data);
    break;
  default:
    ASSERT (FALSE);
  }

  return Data;
}

/**
  Register a PCI device so PCI configuration registers may be accessed after
  SetVirtualAddressMap().

  If any reserved bits in Address are set, then ASSERT().

  @param  Address                  The address that encodes the PCI Bus, Device,
                                   Function and Register.

  @retval RETURN_SUCCESS           The PCI device was registered for runtime access.
  @retval RETURN_UNSUPPORTED       An attempt was made to call this function
                                   after ExitBootServices().
  @retval RETURN_UNSUPPORTED       The resources required to access the PCI device
                                   at runtime could not be mapped.
  @retval RETURN_OUT_OF_RESOURCES  There are not enough resources available to
                                   complete the registration.

**/
RETURN_STATUS
EFIAPI
PciSegmentRegisterForRuntimeAccess (
  IN UINTN  Address
  )
{
  ASSERT_INVALID_PCI_SEGMENT_ADDRESS (Address, 0);
  return RETURN_UNSUPPORTED;
}

/**
  Reads an 8-bit PCI configuration register.

  Reads and returns the 8-bit PCI configuration register specified by Address.

  If any reserved bits in Address are set, then ASSERT().

  @param  Address   The address that encodes the PCI Segment, Bus, Device, Function,
                    and Register.

  @return The 8-bit PCI configuration register specified by Address.

**/
UINT8
EFIAPI
PciSegmentRead8 (
  IN UINT64                    Address
  )
{
  ASSERT_INVALID_PCI_SEGMENT_ADDRESS (Address, 0);

  return (UINT8) PciSegmentLibReadWorker (Address, PciCfgWidthUint8);
}

/**
  Writes an 8-bit PCI configuration register.

  Writes the 8-bit PCI configuration register specified by Address with the value specified by Value.
  Value is returned.  This function must guarantee that all PCI read and write operations are serialized.

  If any reserved bits in Address are set, then ASSERT().

  @param  Address     The address that encodes the PCI Segment, Bus, Device, Function, and Register.
  @param  Value       The value to write.

  @return The value written to the PCI configuration register.

**/
UINT8
EFIAPI
PciSegmentWrite8 (
  IN UINT64                    Address,
  IN UINT8                     Value
  )
{
  ASSERT_INVALID_PCI_SEGMENT_ADDRESS (Address, 0);

  return (UINT8) PciSegmentLibWriteWorker (Address, PciCfgWidthUint8, Value);
}

/**
  Reads a 16-bit PCI configuration register.

  Reads and returns the 16-bit PCI configuration register specified by Address.

  If any reserved bits in Address are set, then ASSERT().
  If Address is not aligned on a 16-bit boundary, then ASSERT().

  @param  Address   The address that encodes the PCI Segment, Bus, Device, Function, and Register.

  @return The 16-bit PCI configuration register specified by Address.

**/
UINT16
EFIAPI
PciSegmentRead16 (
  IN UINT64                    Address
  )
{
  ASSERT_INVALID_PCI_SEGMENT_ADDRESS (Address, 1);

  return (UINT16) PciSegmentLibReadWorker (Address, PciCfgWidthUint16);
}

/**
  Writes a 16-bit PCI configuration register.

  Writes the 16-bit PCI configuration register specified by Address with the
  value specified by Value.

  Value is returned.

  If any reserved bits in Address are set, then ASSERT().
  If Address is not aligned on a 16-bit boundary, then ASSERT().

  @param  Address     The address that encodes the PCI Segment, Bus, Device, Function, and Register.
  @param  Value       The value to write.

  @return The parameter of Value.

**/
UINT16
EFIAPI
PciSegmentWrite16 (
  IN UINT64                    Address,
  IN UINT16                    Value
  )
{
  ASSERT_INVALID_PCI_SEGMENT_ADDRESS (Address, 1);

  return (UINT16) PciSegmentLibWriteWorker (Address, PciCfgWidthUint16, Value);
}

/**
  Reads a 32-bit PCI configuration register.

  Reads and returns the 32-bit PCI configuration register specified by Address.

  If any reserved bits in Address are set, then ASSERT().
  If Address is not aligned on a 32-bit boundary, then ASSERT().

  @param  Address   The address that encodes the PCI Segment, Bus, Device, Function,
                    and Register.

  @return The 32-bit PCI configuration register specified by Address.

**/
UINT32
EFIAPI
PciSegmentRead32 (
  IN UINT64                    Address
  )
{
  ASSERT_INVALID_PCI_SEGMENT_ADDRESS (Address, 3);

  return PciSegmentLibReadWorker (Address, PciCfgWidthUint32);
}

/**
  Writes a 32-bit PCI configuration register.

  Writes the 32-bit PCI configuration register specified by Address with the
  value specified by Value.

  Value is returned.

  If any reserved bits in Address are set, then ASSERT().
  If Address is not aligned on a 32-bit boundary, then ASSERT().

  @param  Address     The address that encodes the PCI Segment, Bus, Device,
                      Function, and Register.
  @param  Value       The value to write.

  @return The parameter of Value.

**/
UINT32
EFIAPI
PciSegmentWrite32 (
  IN UINT64                    Address,
  IN UINT32                    Value
  )
{
  ASSERT_INVALID_PCI_SEGMENT_ADDRESS (Address, 3);

  return PciSegmentLibWriteWorker (Address, PciCfgWidthUint32, Value);
}

/**
  Reads a range of PCI configuration registers into a caller supplied buffer.

  Reads the range of PCI configuration registers specified by StartAddress and
  Size into the buffer specified by Buffer. This function only allows the PCI
  configuration registers from a single PCI function to be read. Size is
  returned.

  If any reserved bits in StartAddress are set, then ASSERT().
  If ((StartAddress & 0xFFF) + Size) > 0x1000, then ASSERT().
  If Size > 0 and Buffer is NULL, then ASSERT().

  @param  StartAddress  The starting address that encodes the PCI Segment, Bus,
                        Device, Function and Register.
  @param  Size          The size in bytes of the transfer.
  @param  Buffer        The pointer to a buffer receiving the data read.

  @return Size

**/
UINTN
EFIAPI
PciSegmentReadBuffer (
  IN  UINT64                   StartAddress,
  IN  UINTN                    Size,
  OUT VOID                     *Buffer
  )
{
  UINTN                             ReturnValue;

  ASSERT_INVALID_PCI_SEGMENT_ADDRESS (StartAddress, 0);
  // 0xFFF is used as limit for 4KB config space
  ASSERT (((StartAddress & 0xFFF) + Size) <= SIZE_4KB);

  if (Size == 0) {
    return Size;
  }

  ASSERT (Buffer != NULL);

  //
  // Save Size for return
  //
  ReturnValue = Size;

  if ((StartAddress & BIT0) != 0) {
    //
    // Read a byte if StartAddress is byte aligned
    //
    *(volatile UINT8 *)Buffer = PciSegmentRead8 (StartAddress);
    StartAddress += sizeof (UINT8);
    Size -= sizeof (UINT8);
    Buffer = (UINT8*)Buffer + BIT0;
  }

  if (Size >= sizeof (UINT16) && (StartAddress & BIT1) != 0) {
    //
    // Read a word if StartAddress is word aligned
    //
    WriteUnaligned16 (Buffer, PciSegmentRead16 (StartAddress));
    StartAddress += sizeof (UINT16);
    Size -= sizeof (UINT16);
    Buffer = (UINT16*)Buffer + BIT0;
  }

  while (Size >= sizeof (UINT32)) {
    //
    // Read as many double words as possible
    //
    WriteUnaligned32 (Buffer, PciSegmentRead32 (StartAddress));
    StartAddress += sizeof (UINT32);
    Size -= sizeof (UINT32);
    Buffer = (UINT32*)Buffer + BIT0;
  }

  if (Size >= sizeof (UINT16)) {
    //
    // Read the last remaining word if exist
    //
    WriteUnaligned16 (Buffer, PciSegmentRead16 (StartAddress));
    StartAddress += sizeof (UINT16);
    Size -= sizeof (UINT16);
    Buffer = (UINT16*)Buffer + BIT0;
  }

  if (Size >= sizeof (UINT8)) {
    //
    // Read the last remaining byte if exist
    //
    *(volatile UINT8 *)Buffer = PciSegmentRead8 (StartAddress);
  }

  return ReturnValue;
}


/**
  Copies the data in a caller supplied buffer to a specified range of PCI
  configuration space.

  Writes the range of PCI configuration registers specified by StartAddress and
  Size from the buffer specified by Buffer. This function only allows the PCI
  configuration registers from a single PCI function to be written. Size is
  returned.

  If any reserved bits in StartAddress are set, then ASSERT().
  If ((StartAddress & 0xFFF) + Size) > 0x1000, then ASSERT().
  If Size > 0 and Buffer is NULL, then ASSERT().

  @param  StartAddress  The starting address that encodes the PCI Segment, Bus,
                        Device, Function and Register.
  @param  Size          The size in bytes of the transfer.
  @param  Buffer        The pointer to a buffer containing the data to write.

  @return The parameter of Size.

**/
UINTN
EFIAPI
PciSegmentWriteBuffer (
  IN UINT64                    StartAddress,
  IN UINTN                     Size,
  IN VOID                      *Buffer
  )
{
  UINTN                             ReturnValue;

  ASSERT_INVALID_PCI_SEGMENT_ADDRESS (StartAddress, 0);
  // 0xFFF is used as limit for 4KB config space
  ASSERT (((StartAddress & 0xFFF) + Size) <= SIZE_4KB);

  if (Size == 0) {
    return Size;
  }

  ASSERT (Buffer != NULL);

  //
  // Save Size for return
  //
  ReturnValue = Size;

  if ((StartAddress & BIT0) != 0) {
    //
    // Write a byte if StartAddress is byte aligned
    //
    PciSegmentWrite8 (StartAddress, *(UINT8*)Buffer);
    StartAddress += sizeof (UINT8);
    Size -= sizeof (UINT8);
    Buffer = (UINT8*)Buffer + BIT0;
  }

  if (Size >= sizeof (UINT16) && (StartAddress & BIT1) != 0) {
    //
    // Write a word if StartAddress is word aligned
    //
    PciSegmentWrite16 (StartAddress, ReadUnaligned16 (Buffer));
    StartAddress += sizeof (UINT16);
    Size -= sizeof (UINT16);
    Buffer = (UINT16*)Buffer + BIT0;
  }

  while (Size >= sizeof (UINT32)) {
    //
    // Write as many double words as possible
    //
    PciSegmentWrite32 (StartAddress, ReadUnaligned32 (Buffer));
    StartAddress += sizeof (UINT32);
    Size -= sizeof (UINT32);
    Buffer = (UINT32*)Buffer + BIT0;
  }

  if (Size >= sizeof (UINT16)) {
    //
    // Write the last remaining word if exist
    //
    PciSegmentWrite16 (StartAddress, ReadUnaligned16 (Buffer));
    StartAddress += sizeof (UINT16);
    Size -= sizeof (UINT16);
    Buffer = (UINT16*)Buffer + BIT0;
  }

  if (Size >= sizeof (UINT8)) {
    //
    // Write the last remaining byte if exist
    //
    PciSegmentWrite8 (StartAddress, *(UINT8*)Buffer);
  }

  return ReturnValue;
}
