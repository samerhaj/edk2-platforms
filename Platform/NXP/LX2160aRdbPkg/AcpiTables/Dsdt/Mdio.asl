/** @file
  Differentiated System Description Table Fields (DSDT)

  Copyright 2019 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

  Based on the files under ArmPlatformPkg/ArmJunoPkg/AcpiTables/

**/

Scope(_SB)
{
  Device(MDI0) {
    Name(_HID, "NXP0006")
    Name(_CCA, 1)
    Name(_UID, 0)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, MDI0_BASE, MDI_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared)
       {
         MDI0_IT
       }
    }) // end of _CRS for MDI0
    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
         Package () {"little-endian", 1},
      }
    })
  } // end of MDI0
}
