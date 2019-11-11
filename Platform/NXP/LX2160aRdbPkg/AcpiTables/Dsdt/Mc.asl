/** @file
  Differentiated System Description Table Fields (DSDT)

  Copyright 2019 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

Scope(_SB)
{
  Device(MCE0) {
    Name(_HID, "NXP0008")
    Name(_CCA, 1)
    Name(_UID, 0)
    Name(_CRS, ResourceTemplate() {
      QwordMemory (
        ResourceConsumer,
        PosDecode,      // _DEC
        MinFixed,       // _MIF
        MaxFixed,       // _MAF
        NonCacheable,   // _MEM
        ReadWrite,      // _RW
        0,              // _GRA
        0x80c000000,    // _MIN MinAddress
        0x80c00003f,    // _MAX MaxAddress
        0,              // _TRA
        0x40,           // _LEN
          ,)
      Memory32Fixed(ReadWrite, 0x08340000, 0x40000)
    }) // end of _CRS for fsl-mc device
    //DPMACs
    Device(PR17) { // 1G
      Name (_ADR, 0x11)
      Name (_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
            Package () {
                Package () {"reg", 17},
                Package () {"phy-connection-type", "rgmii-id"},
                Package () {"phy-handle", Package (){\_SB.MDI0.PHY1}}
        }
      })
    }

    Device(PR18) { // 1G
      Name (_ADR, 0x12)
      Name (_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
            Package () {
                Package () {"reg", 18},
                Package () {"phy-connection-type", "rgmii-id"},
                Package () {"phy-handle", Package (){\_SB.MDI0.PHY2}}
        }
      })
    } // end of DPMAC
  } // end of fsl-mc device
} // end of fsl-mc controller
