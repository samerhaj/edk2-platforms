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
    Device(PHY1) {
      Name (_ADR, 0x1)
      Name (_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
          Package () {
            Package () {"reg", 1},
            Package () {"phy-addr", 1},
            Package () {"compatible", "ethernet-phy-id004d.d072"}
        }
      })
    } // end of PHY1
    Device(PHY2) {
      Name (_ADR, 0x2)
      Name (_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
          Package () {
            Package () {"reg", 2},
            Package () {"phy-addr", 2},
            Package () {"compatible", "ethernet-phy-id004d.d072"}
        }
      })
    } // end of PHY2
    Device(PHY4) {
      Name (_ADR, 0x4)
      Name (_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
          Package () {
            Package () {"reg", 4},
            Package () {"phy-addr", 4},
            Package () {"compatible", "ethernet-phy-ieee802.3-c45"}
        }
      })
    } // end of PHY4
    Device(PHY5) {
      Name (_ADR, 0x5)
      Name (_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
          Package () {
            Package () {"reg", 5},
            Package () {"phy-addr", 5},
            Package () {"compatible", "ethernet-phy-ieee802.3-c45"}
        }
      })
    } // end of PHY4
  } // end of MDI0
  Device(MDI1) {
    Name(_HID, "NXP0006")
    Name(_CCA, 1)
    Name(_UID, 1)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, MDI1_BASE, MDI_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared)
       {
         MDI1_IT
       }
    }) // end of _CRS for MDI1
    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
         Package () {"little-endian", 1},
      }
    })
    Device(PHY0) {
      Name (_ADR, 0x0)
      Name (_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
          Package () {
            Package () {"reg", 0},
            Package () {"phy-addr", 0},
            Package () {"compatible", "ethernet-phy-id0210.7440"}
        }
      })
    } // end of PHY0
  } // end of MDI1
}
