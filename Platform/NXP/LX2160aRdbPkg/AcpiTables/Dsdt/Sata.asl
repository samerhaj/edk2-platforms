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
  Device(SAT0) {
    Name(_HID, "NXP0004")
    Name(_CCA, 1)
    Name(_UID, 0)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, SATA0_BASE, SATA_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive)
       {
         SATA0_IT_1, SATA0_IT_2, SATA0_IT_3
       }
    })
  }

  Device(SAT1) {
    Name(_HID, "NXP0004")
    Name(_CCA, 1)
    Name(_UID, 1)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, SATA1_BASE, SATA_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive)
       {
         SATA1_IT_1, SATA1_IT_2, SATA1_IT_3
       }
    })
  }

  Device(SAT2) {
    Name(_HID, "NXP0004")
    Name(_CCA, 1)
    Name(_UID, 2)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, SATA2_BASE, SATA_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive)
       {
         SATA2_IT_1, SATA2_IT_2, SATA2_IT_3
       }
    })
  }

  Device(SAT3) {
    Name(_HID, "NXP0004")
    Name(_CCA, 1)
    Name(_UID, 3)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, SATA3_BASE, SATA_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive)
       {
         SATA3_IT_1, SATA3_IT_2, SATA3_IT_3
       }
    })
  }
}
