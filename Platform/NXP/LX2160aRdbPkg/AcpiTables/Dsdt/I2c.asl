/** @file
  Differentiated System Description Table Fields (DSDT)

  Copyright (c) 2014, ARM Ltd. All rights reserved.<BR>
  Copyright (c) 2015, Linaro Limited. All rights reserved.<BR>
  Copyright 2017-2018 NXP

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
      EMC2305 Fan Controller
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
        I2CSerialBus(0x75, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C0", 0, ResourceConsumer, ,)
      }) // end of CRS for mux device
     Device (CH01) {
        Name(_ADR, 1)
        Device(FAN1) {
        Name (_HID, "PRP0001")
          Name(_CRS, ResourceTemplate() {
            I2CSerialBus(0x4D, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C0.MUX0.CH01", 0, ResourceConsumer, ,)
          })
         Name (_DSD, Package () {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package() {
                        Package() {"compatible", "smsc,emc2305"},
                }
          })
        } //end of fan device
      } // end of channel 1 of mux
/*
 Need to see, how to add ina2xx
Currently no matching even for DT
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
      } // end of channel 1 of mux
*/
/*
   temp sensor .. no linux driver yet
   should we expose as ACPI method for tempature sense.
  need to check how ???
     Device (CH03) {
        Name(_ADR, 3)
        Device(THE1) {
        Name (_HID, "PRP0001")
          Name(_CRS, ResourceTemplate() {
            I2CSerialBus(0x4D, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C0.MUX0.CH01", 0, ResourceConsumer, ,)
          })
         Name (_DSD, Package () {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package() {
                        Package() {"compatible", "smsc,emc2305"},
                }
          })
        } //end of fan device
      } // end of channel 1 of mux
*/
     Device (CH07) {
        Name(_ADR, 7)
        Device(MUX1) {
        Name (_HID, "NXP0002")
          Name(_CRS, ResourceTemplate() {
            I2CSerialBus(0x75, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C0.MUX0.CH01", 0, ResourceConsumer, ,)
          })
         Name (_DSD, Package () {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package() {
                        Package() {"compatible", "smsc,emc2305"},
                }
          })

     Device (CH00) {
        Name(_ADR, 0)
        Device(CAG1) {
        Name (_HID, "PRP0001")
          Name(_CRS, ResourceTemplate() {
            I2CSerialBus(0x4D, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C0.MUX0.CH07.MUX1.CH00", 0, ResourceConsumer, ,)
          })
         Name (_DSD, Package () {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package() {
                        Package() {"compatible", "cage"},
                }
          })
        } //end of fan device
      } // end of channel 1 of mux
        } //end of mux-1 device
      } // end of channel 7 of mux-0
      Device(MUX1) {
      }
    } // end of Mux 0
  } // end of i2c device

  Device(I2C1) {
    Name(_HID, "NXP0001")
    Name(_UID, 1)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, I2C1_BASE, I2C_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { I2C1_IT }
    }) // end of _CRS for i2c1 device
    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
         Package () {"clock-frequency", DEFAULT_PLAT_FREQ}, //This is device specific data, Need to see how to pass clk stuff
      }
    })
  } // end of i2c device

  Device(I2C2) {
    Name(_HID, "NXP0001")
    Name(_UID, 2)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, I2C2_BASE, I2C_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { I2C2_IT }
    }) // end of _CRS for i2c2 device
    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
         Package () {"clock-frequency", DEFAULT_PLAT_FREQ}, //This is device specific data, Need to see how to pass clk stuff
      }
    })
  } // end of i2c device

  Device(I2C3) {
    Name(_HID, "NXP0001")
    Name(_UID, 3)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, I2C3_BASE, I2C_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { I2C3_IT }
    }) // end of _CRS for i2c3 device
    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
         Package () {"clock-frequency", DEFAULT_PLAT_FREQ}, //This is device specific data, Need to see how to pass clk stuff
      }
    })
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

  Device(I2C5) {
    Name(_HID, "NXP0001")
    Name(_UID, 0)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, I2C5_BASE, I2C_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { I2C5_IT }
    }) // end of _CRS for i2c5 device
  } // end of i2c device

  Device(I2C6) {
    Name(_HID, "NXP0001")
    Name(_UID, 6)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, I2C6_BASE, I2C_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { I2C6_IT }
    }) // end of _CRS for i2c6 device
    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
         Package () {"clock-frequency", DEFAULT_PLAT_FREQ}, //This is device specific data, Need to see how to pass clk stuff
      }
    })
  } // end of i2c device

  Device(I2C7) {
    Name(_HID, "NXP0001")
    Name(_UID, 7)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, I2C7_BASE, I2C_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { I2C7_IT }
    }) // end of _CRS for i2c7 device
    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
         Package () {"clock-frequency", DEFAULT_PLAT_FREQ}, //This is device specific data, Need to see how to pass clk stuff
      }
    })
  } // end of i2c device

} // end of i2c controllers
