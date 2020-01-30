/** @file
  Differentiated System Description Table Fields (DSDT)

  Copyright (c) 2014, ARM Ltd. All rights reserved.<BR>
  Copyright (c) 2015, Linaro Limited. All rights reserved.<BR>
  Copyright 2017-2019 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

  Based on the files under ArmPlatformPkg/ArmJunoPkg/AcpiTables/

**/

/*
  i2c connections on this board and users
  I2C1
  Slaves - CPLD, MUX, SEC Flash, SPD
           SYSFlash, AQR107 Phy, EMC2305 Fan Controller
           Si52147 PCIe Clock Gen, Si5341B Clock Synthesizer
           INA220 Power Measurement, LTC3882 regulator
           SA56004E Thermal Monitor, PCF2129 RTC
           CS4223 BootFlash,
           Mux 1, zQSFP+ Cage, SFP+ Cage, PCIe Slot

Most of devices are used for boot, except few to be
exposed to OS Like
      INA220 Power Measurement
      SA56004E Thermal Monitor,

Rest Devices on Mux1 are for debug purpose.
These could be added in case of *debug only*

*/

Scope(_SB)
{
  Device(I2C0) {
    Name(_HID, "NXP0001")
    Name(_UID, 0)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, I2C0_BASE, I2C_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { I2C0_IT }
    }) // end of _CRS for i2c0 device
    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
         Package () {"clock-frequency", DEFAULT_PLAT_FREQ}, //This is device specific data, Need to see how to pass clk stuff
      }
    })
    Device(MUX0) {
      NAME(_HID, "NXP0002")
      Name(_CRS, ResourceTemplate()
      {
        I2CSerialBus(0x77, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C0", 0, ResourceConsumer, ,)
      }) // end of CRS for mux device
      Device (CH02) {
        Name(_ADR, 2)
        Device(POW1) {
        Name (_HID, "PRP0001")
          Name(_CRS, ResourceTemplate() {
            I2CSerialBus(0x40, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C0.MUX0.CH02", 0, ResourceConsumer, ,)
          })
          Name (_DSD, Package () {
            ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
            Package() {
              Package() {"compatible", "ina2xx"},
            }
          })
        } //end of fan device
      } // end of channel 2 of mux
      Device (CH03) {
        Name(_ADR, 3)
        Device(THE1) {
          Name(_ADR, 1)
          Name (_HID, "PRP0001")
            Name(_CRS, ResourceTemplate() {
              I2CSerialBus(0x4C, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C0.MUX0.CH03", 0, ResourceConsumer, ,)
            })
            Name (_DSD, Package () {
              ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
              Package() {
                Package() {"compatible", "smsc,emc2305"},
                Package() {"compatible", "nxp,sa56004"},
                Package() {"vcc-supply", 0xe},
              }
            })
        } //end of temperature sensor device
        Device(THE2) {
          Name(_ADR, 2)
          Name (_HID, "PRP0001")
            Name(_CRS, ResourceTemplate() {
              I2CSerialBus(0x4D, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C0.MUX0.CH03", 0, ResourceConsumer, ,)
            })
          Name (_DSD, Package () {
            ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
            Package() {
              Package() {"compatible", "cage"},
            }
          })
        } //end of temperature sensor device 
      } // end of channel 3 of mux
    } //end of mux-0 device
  } // end of i2c device

  Device(I2C4) {
    Name(_HID, "NXP0001")
    Name(_UID, 4)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, I2C4_BASE, I2C_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { I2C4_IT }
    }) // end of _CRS for i2c4 device
    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
         Package () {"clock-frequency", DEFAULT_PLAT_FREQ}, //This is device specific data, Need to see how to pass clk stuff
      }
    })
  } // end of i2c device

} // end of i2c controllers
