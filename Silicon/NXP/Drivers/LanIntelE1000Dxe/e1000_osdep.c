/**************************************************************************

Copyright (c) 2001-2010, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 3. Neither the name of the Intel Corporation nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/

#include "e1000.h"


INT32
e1000_read_pcie_cap_reg(
  struct e1000_hw *hw,
  UINT32          reg,
  UINT16         *value
  )
/*++

Routine Description:
  Reads from the PCI capabality region.

Arguments:
  hw                - Pointer to the shared code hw structure.
  reg               - The register offset within the cap region.
  value             - The value at the register offset.

Returns:
  Error code

--*/
{
  // Start at the first cap pointer of PCI space
  UINT16 next_ptr_offset = PCI_CAP_PTR;
  UINT16 next_ptr_value = 0;

  e1000_read_pci_cfg(hw, next_ptr_offset, &next_ptr_value);
  // Keep only the first byte of the returned value
  next_ptr_value &= 0xFF;
  // Traverse the capabilities linked regions until the end
  while (next_ptr_value != PCI_CAP_PTR_ENDPOINT)
  {
    next_ptr_offset = next_ptr_value;
    e1000_read_pci_cfg(hw, next_ptr_offset, &next_ptr_value);
    // Check if we found the requested capabilities region
    if ((next_ptr_value & 0xFF) != PCI_EX_CAP_ID)
    {
      // Jump to the next capabilities region
      next_ptr_value &= 0xFF00;
      next_ptr_value = next_ptr_value >> 8;
      next_ptr_value &= 0xFF;
    }
    else
    {
      // Read the value from the request offset
      e1000_read_pci_cfg(hw, next_ptr_offset + reg, &next_ptr_value);
      *value = next_ptr_value;
      return E1000_SUCCESS;
    }
  }
  // The requested region was not found in PCI space
  DEBUGPRINT(IO, ("Cap ID 0x10 was not found in PCI space.\n"));
  return E1000_ERR_CONFIG;
}

#ifndef NO_PCIE_SUPPORT
INT32
e1000_write_pcie_cap_reg(
  struct e1000_hw *hw,
  UINT32          reg,
  UINT16         *value
  )
/*++

Routine Description:
  Writes into the PCI capabality region.

Arguments:
  hw                - Pointer to the shared code hw structure.
  reg               - The register offset within the cap region.
  value             - The value at the register offset.

Returns:
  Error code

--*/
{
  // Start at the first cap pointer of PCI space
  UINT16 next_ptr_offset = PCI_CAP_PTR;
  UINT16 next_ptr_value = 0;

  e1000_read_pci_cfg(hw, next_ptr_offset, &next_ptr_value);
  // Keep only the first byte of the returned value
  next_ptr_value &= 0xFF;
  // Traverse the capabilities linked regions until the end
  while (next_ptr_value != PCI_CAP_PTR_ENDPOINT)
  {
    next_ptr_offset = next_ptr_value;
    e1000_read_pci_cfg(hw, next_ptr_offset, &next_ptr_value);
    // Check if we found the requested capabilities region
    if ((next_ptr_value & 0xFF) != PCI_EX_CAP_ID)
    {
      // Jump to the next capabilities region
      next_ptr_value &= 0xFF00;
      next_ptr_value = next_ptr_value >> 8;
      next_ptr_value &= 0xFF;
    }
    else
    {
      // Write the value from the request offset
      e1000_write_pci_cfg(hw, next_ptr_offset + reg, value);
      return E1000_SUCCESS;
    }
  }
  // The requested region was not found in PCI space
  DEBUGPRINT(IO, ("Cap ID 0x10 was not found in PCI space.\n"));
  return E1000_ERR_CONFIG;
}
#endif

VOID
e1000_write_reg_io(
  struct e1000_hw *hw,
  UINT32          offset,
  UINT32          value
  )
/*++

Routine Description:
  Writes a value to one of the devices registers using port I/O (as opposed to
  memory mapped I/O). Only 82544 and newer devices support port I/O.

Arguments:
  hw                - Pointer to the shared code hw structure.
  offset            - The register offset to write.
  value             - The value to write to the register.

Returns:
  VOID

--*/
{
  GIG_DRIVER_DATA *Adapter;

  Adapter  = hw->back;

  DEBUGPRINT(IO, ("e1000_write_reg_io\n"));
  DEBUGPRINT(IO, ("IO bAR INDEX = %d\n", Adapter->IoBarIndex));
  DEBUGWAIT(IO);

  MemoryFence  ();
  Adapter->PciIo->Io.Write (
    Adapter->PciIo,
    EfiPciIoWidthUint32,
    Adapter->IoBarIndex,
    0,                          // IO location offset
    1,
    (VOID *) (&offset)
    );
  MemoryFence  ();
  Adapter->PciIo->Io.Write (
    Adapter->PciIo,
    EfiPciIoWidthUint32,
    Adapter->IoBarIndex,
    4,                          // IO data offset
    1,
    (VOID *) (&value)
    );
  MemoryFence  ();
  return;
}

VOID
e1000_read_pci_cfg (
  struct e1000_hw *hw,
  UINT32          port,
  UINT16         *value
  )
/*++

Routine Description:
  This function calls the EFI PCI IO protocol to read a value from the device's PCI
  register space.

Arguments:
  hw                - Pointer to the shared code hw structure.
  Port              - Which register to read from.
  value             - Returns the value read from the PCI register.

Returns:
  VOID

--*/
{
  GIG_DRIVER_DATA *Adapter;
  Adapter = hw->back;

  MemoryFence  ();

  Adapter->PciIo->Pci.Read (
    Adapter->PciIo,
    EfiPciIoWidthUint16,
    port,
    1,
    (VOID *) value
    );
  MemoryFence  ();
  return ;
}

VOID
e1000_write_pci_cfg (
  struct e1000_hw *hw,
  UINT32          port,
  UINT16         *value
  )
/*++

Routine Description:
  This function calls the EFI PCI IO protocol to write a value to the device's PCI
  register space.

Arguments:
  hw                - Pointer to the shared code hw structure.
  Port              - Which register to write to.
  value             - Value to write to the PCI register.

Returns:
  VOID

--*/
{
  GIG_DRIVER_DATA *Adapter;

  Adapter = hw->back;

  MemoryFence  ();
  Adapter->PciIo->Pci.Write (
    Adapter->PciIo,
    EfiPciIoWidthUint16,
    port,
    1,
    (VOID *) value
    );
  MemoryFence  ();

  return ;
}

VOID
DelayInMicroseconds (
  IN GIG_DRIVER_DATA  *Adapter,
  UINTN               MicroSeconds
  )
/*++

Routine Description:

Arguments:
  AdapterInfo                     - Pointer to the NIC data structure information
                                    which the UNDI driver is layering on..
  MicroSeconds                    - Time to delay in Microseconds.

Returns:

--*/
{
  if (Adapter->Delay != NULL) {
    (*Adapter->Delay) (Adapter->Unique_ID, MicroSeconds);
  } else {
    gBS->Stall(MicroSeconds);
  }
}

VOID
uSecDelay (
  struct e1000_hw *hw,
  UINTN  usecs
  )
/*++

Routine Description:
  Delay a specified number of microseconds.

Arguments:
  hw                - Pointer to hardware instance.
  usecs             - Number of microseconds to delay

--*/
{
  DelayInMicroseconds(hw->back, usecs);
}

UINT32
e1000_InDword (
  IN struct e1000_hw *hw,
  IN UINT32         Port
  )
/*++

Routine Description:
  This function calls the MemIo callback to read a dword from the device's
  address space
  Since UNDI3.0 uses the TmpMemIo function (instead of the callback routine)
  which also takes the UniqueId parameter (as in UNDI3.1 spec) we don't have
  to make undi3.0 a special case

Arguments:
  hw                - Pointer to hardware instance.
  Port              - Which port to read from.

Returns:
  Results           - The data read from the port.

--*/
{
  UINT32  Results;
  GIG_DRIVER_DATA *Adapter;
  Adapter = hw->back;

  MemoryFence  ();
  Adapter->PciIo->Mem.Read (
    Adapter->PciIo,
    EfiPciIoWidthUint32,
    0,
    Port,
    1,
    (VOID *) (&Results)
    );
  MemoryFence  ();
  return Results;
}

VOID
e1000_OutDword (
  IN struct e1000_hw *hw,
  IN UINT32          Port,
  IN UINT32          Data
  )
/*++

Routine Description:
  This function calls the MemIo callback to write a word from the device's
  address space
  Since UNDI3.0 uses the TmpMemIo function (instead of the callback routine)
  which also takes the UniqueId parameter (as in UNDI3.1 spec) we don't have
  to make undi3.0 a special case

Arguments:
  hw                - Pointer to hardware instance.
  Data              - Data to write to Port.
  Port              - Which port to write to.

Returns:
  none

--*/
{
  UINT32  Value;
  GIG_DRIVER_DATA *Adapter;

  Adapter = hw->back;
  Value = Data;

  MemoryFence  ();

  Adapter->PciIo->Mem.Write (
    Adapter->PciIo,
    EfiPciIoWidthUint32,
    0,
    Port,
    1,
    (VOID *) (&Value)
    );

  MemoryFence  ();
  return ;
}

UINT32
e1000_FlashRead (
  IN struct e1000_hw *hw,
  IN UINT32          Port
  )
/*++

Routine Description:
  This function calls the MemIo callback to read a dword from the device's
  address space
  Since UNDI3.0 uses the TmpMemIo function (instead of the callback routine)
  which also takes the UniqueId parameter (as in UNDI3.1 spec) we don't have
  to make undi3.0 a special case

Arguments:
  hw                - Pointer to hardware instance.
  Port              - Which port to read from.

Returns:
  Results           - The data read from the port.

--*/
{
  UINT32  Results;
  GIG_DRIVER_DATA *Adapter;
  Adapter = hw->back;

  MemoryFence  ();
  Adapter->PciIo->Mem.Read (
    Adapter->PciIo,
    EfiPciIoWidthUint32,
    1,
    Port,
    1,
    (VOID *) (&Results)
    );
  MemoryFence  ();
  return Results;
}


UINT16
e1000_FlashRead16 (
  IN struct e1000_hw *hw,
  IN UINT32         Port
  )
/*++

Routine Description:
  This function calls the MemIo callback to read a dword from the device's
  address space
  Since UNDI3.0 uses the TmpMemIo function (instead of the callback routine)
  which also takes the UniqueId parameter (as in UNDI3.1 spec) we don't have
  to make undi3.0 a special case

Arguments:
  hw                - Pointer to hardware instance.
  Port              - Which port to read from.

Returns:
  Results           - The data read from the port.

--*/
{
  UINT16  Results;
  GIG_DRIVER_DATA *Adapter;
  Adapter = hw->back;

  MemoryFence  ();
  Adapter->PciIo->Mem.Read (
    Adapter->PciIo,
    EfiPciIoWidthUint16,
    1,
    Port,
    1,
    (VOID *) (&Results)
    );
  MemoryFence  ();
  return Results;
}


VOID
e1000_FlashWrite (
  IN struct e1000_hw *hw,
  IN UINT32          Port,
  IN UINT32          Data
  )
/*++

Routine Description:
  This function calls the MemIo callback to write a word from the device's
  address space
  Since UNDI3.0 uses the TmpMemIo function (instead of the callback routine)
  which also takes the UniqueId parameter (as in UNDI3.1 spec) we don't have
  to make undi3.0 a special case

Arguments:
  hw                - Pointer to hardware instance.
  Data              - Data to write to Port.
  Port              - Which port to write to.

Returns:
  none

--*/
{
  UINT32  Value;
  GIG_DRIVER_DATA *Adapter;

  Adapter = hw->back;
  Value = Data;

  MemoryFence  ();

  Adapter->PciIo->Mem.Write (
    Adapter->PciIo,
    EfiPciIoWidthUint32,
    1,
    Port,
    1,
    (VOID *) (&Value)
    );

  MemoryFence  ();
  return ;
}

VOID
e1000_FlashWrite16 (
  IN struct e1000_hw *hw,
  IN UINT32         Port,
  IN UINT16         Data
  )
/*++

Routine Description:
  This function calls the MemIo callback to write a word from the device's
  address space
  Since UNDI3.0 uses the TmpMemIo function (instead of the callback routine)
  which also takes the UniqueId parameter (as in UNDI3.1 spec) we don't have
  to make undi3.0 a special case

Arguments:
  Data              - Data to write to Port.
  Port              - Which port to write to.

Returns:
  none

--*/
{
  GIG_DRIVER_DATA *GigAdapter;

  GigAdapter = hw->back;

  MemoryFence  ();

  GigAdapter->PciIo->Mem.Write (
    GigAdapter->PciIo,
    EfiPciIoWidthUint16,
    1,
    Port,
    1,
    (VOID *) (&Data)
    );

  MemoryFence  ();
  return ;
}

VOID
e1000_PciFlush (
  IN struct e1000_hw *hw
  )
/*++

Routine Description:
  Flushes a PCI write transaction to system memory.

Arguments:
  hw - Pointer to hardware structure.

Returns:
  none

--*/
{
  GIG_DRIVER_DATA *Adapter;
  Adapter = hw->back;

  MemoryFence  ();

  Adapter->PciIo->Flush (Adapter->PciIo);

  MemoryFence  ();

  return ;
}

VOID
e1000_pci_set_mwi (
  struct e1000_hw *hw
  )
/*++

Routine Description:
  Sets the memory write and invalidate bit in the device's PCI command register.

Arguments:
  hw                - Pointer to the shared code hw structure.

Returns:
  VOID

--*/
{
  GIG_DRIVER_DATA *Adapter;
  UINT32          CommandReg;

  Adapter = hw->back;

  MemoryFence  ();
  Adapter->PciIo->Pci.Read (
    Adapter->PciIo,
    EfiPciIoWidthUint16,
    PCI_COMMAND,
    1,
    (VOID *) (&CommandReg)
    );

  CommandReg |= PCI_COMMAND_MWI;

  Adapter->PciIo->Pci.Write (
    Adapter->PciIo,
    EfiPciIoWidthUint16,
    PCI_COMMAND,
    1,
    (VOID *) (&CommandReg)
    );
  MemoryFence  ();

  return ;
}

VOID
e1000_pci_clear_mwi (
  struct e1000_hw *hw
  )
/*++

Routine Description:
  Clears the memory write and invalidate bit in the device's PCI command register.

Arguments:
  hw                - Pointer to the shared code hw structure.

Returns:
  VOID

--*/
{
  GIG_DRIVER_DATA *Adapter;
  UINT32          CommandReg;

  Adapter = hw->back;

  Adapter->PciIo->Pci.Read (
    Adapter->PciIo,
    EfiPciIoWidthUint16,
    PCI_COMMAND,
    1,
    (VOID *) (&CommandReg)
    );

  CommandReg &= ~PCI_COMMAND_MWI;

  Adapter->PciIo->Pci.Write (
    Adapter->PciIo,
    EfiPciIoWidthUint16,
    PCI_COMMAND,
    1,
    (VOID *) (&CommandReg)
    );

  return ;
}


