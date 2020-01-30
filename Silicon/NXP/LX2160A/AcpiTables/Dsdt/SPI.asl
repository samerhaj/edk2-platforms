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

Scope(_SB)
{
  Device(SPI0) {
    Name(_HID, "NXP0005")
    Name(_UID, 0)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, SPI0_BASE, SPI_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { SPI_IT }
    }) // end of _CRS for spi device
    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
         Package () {"clock-frequency", DEFAULT_PLAT_FREQ},
         Package () {"spi-num-chipselects", 4},
         Package () {"bus-num", 0},
      }
    }) // end of DSD SPI device
  } // end of SPI device

  Device(SPI1) {
    Name(_HID, "NXP0005")
    Name(_UID, 1)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, SPI1_BASE, SPI_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { SPI_IT }
    }) // end of _CRS for spi device
    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
         Package () {"clock-frequency", DEFAULT_PLAT_FREQ},
         Package () {"spi-num-chipselects", 4},
         Package () {"bus-num", 0},
      }
    }) // end of DSD SPI device
  } // end of SPI device

  Device(SPI2) {
    Name(_HID, "NXP0005")
    Name(_UID, 2)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, SPI2_BASE, SPI_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { SPI_IT }
    }) // end of _CRS for spi device
    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
         Package () {"clock-frequency", DEFAULT_PLAT_FREQ},
         Package () {"spi-num-chipselects", 4},
         Package () {"bus-num", 0},
      }
    }) // end of DSD SPI device
  } // end of SPI device
}
