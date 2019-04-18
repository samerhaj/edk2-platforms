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
  Device(SDC0) {
    Name(_HID, "NXP0003")
    Name(_CID, "PNP0D40")
    Name(_CCA, 0)
    Name(_UID, 0)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, SDC0_BASE, SDC_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive)
       {
         SDC0_IT
       }
    })
    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
         Package () {"clock-frequency", DEFAULT_PLAT_FREQ},
         Package () {"little-endian", 1},
         Package () {"voltage-range", "1800,3300,330"},
         Package () {"sdhci,auto-cmd12", 1},
         Package () {"sd-uhs-sdr104", 1},
         Package () {"sd-uhs-sdr50", 1},
         Package () {"sd-uhs-sdr25", 1},
         Package () {"sd-uhs-sdr12", 1},

      }
    })
  }

  Device(SDC1) {
    Name(_HID, "NXP0003")
    Name(_CID, "PNP0D40")
    Name(_CCA, 0)
    Name(_UID, 1)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, SDC1_BASE, SDC_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive)
       {
         SDC1_IT
       }
    })
    Name (_DSD, Package () {
      ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
         Package () {"clock-frequency", DEFAULT_PLAT_FREQ},
         Package () {"little-endian", 1},
         Package () {"voltage-range", "1800,1800,3300,3330"},
         Package () {"sdhci,auto-cmd12", 1},
         Package () {"broken-cd", 1},
         Package () {"bus-width", 8},
         Package () {"mmc-hs200-1_8v", 1},
         Package () {"mmc-hs400-1_8v", 1},
      }
    })
  }
}
