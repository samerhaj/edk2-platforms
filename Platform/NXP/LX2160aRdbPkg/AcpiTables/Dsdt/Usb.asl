/** @file

  Dfferentiated System Description Table Fields (DSDT)

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
  Device(USB0){
    Name(_HID, "808622B7")
    Name(_UID, 1)
    Name(_CCA, 0)

    Name(_CRS, ResourceTemplate(){
      Memory32Fixed(ReadWrite, USB0_BASE, USB_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) {USB0_IT}
    })

    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package () {"dr_mode", "host"}, // Mode of controller
        Package () {"snps,quirk-frame-length-adjustment", 0x20},
        Package () {"snps,dis_rxdet_inp3_quirk",1},
      }
    })
    //
    // Root Hub
    //
    Device(RHUB){
      Name(_ADR, 0x00000000)  // Address of Root Hub should be 0
      //
      // Ports connected to Root Hub
      // Port 1 is connectable
      //
      Device(PRT1){
        Name(_ADR, 0x00000001)
        Name(_UPC, Package(){
          0xFF,        // Port is connectable
          0x03,        // Port connector is USB3.0 Type A
          0x00000000,
          0x00000000
        })
        Name(_PLD, Package(){
          Buffer(0x10){
            0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x31, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
          }
        })
      } // USB0_RHUB_PRT1
      Device(PRT2){
        Name(_ADR, 0x00000002)
        Name(_UPC, Package(){
          0x00,        // Port is not connectable
          0xFF,        // Port connector is A
          0x00000000,
          0x00000000
        })
      } // USB0_RHUB_PRT2
    } // USB0_RHUB
  } // USB0

  Device(USB1){
    Name(_HID, "808622B7")
    Name(_UID, 1)
    Name(_CCA, 0)

    Name(_CRS, ResourceTemplate(){
      Memory32Fixed(ReadWrite, USB1_BASE, USB_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) {USB1_IT}
    })

    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package () {"dr_mode", "host"}, // Mode of controller
        Package () {"snps,quirk-frame-length-adjustment", 0x20},
        Package () {"snps,dis_rxdet_inp3_quirk",1},
      }
    })
    //
    // Root Hub
    //
    Device(RHUB){
      Name(_ADR, 0x00000000)  // Address of Root Hub should be 0
      //
      // Ports connected to Root Hub
      // Port 1 is connectable
      //
      Device(PRT1){
        Name(_ADR, 0x00000001)
        Name(_UPC, Package(){
          0xFF,        // Port is connectable
          0x06,        // Port connector is Micro USB3.0 Type AB
          0x00000000,
          0x00000000
        })
        Name(_PLD, Package(){
          Buffer(0x10){
            0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x31, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
          }
        })
      } // USB1_RHUB_PRT1
      Device(PRT2){
        Name(_ADR, 0x00000002)
        Name(_UPC, Package(){
          0x00,        // Port is not connectable
          0xFF,        // Port connector is A
          0x00000000,
          0x00000000
        })
      } // USB1_RHUB_PRT2
    } // USB1_RHUB
  } // USB1
}
