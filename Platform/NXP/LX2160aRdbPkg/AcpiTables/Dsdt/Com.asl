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
  Device(COM0) {
    Name(_HID, "ARMH0011")
    Name(_UID, 0)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, UART0_BASE, UART_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { UART0_IT }
    })
  }
  Device(COM1) {
    Name(_HID, "ARMH0011")
    Name(_UID, 1)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, UART1_BASE, UART_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { UART1_IT }
    })
  }
  Device(COM2) {
    Name(_HID, "ARMH0011")
    Name(_UID, 2)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, UART2_BASE, UART_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { UART2_IT }
    })
  }
  Device(COM3) {
    Name(_HID, "ARMH0011")
    Name(_UID, 3)
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, UART3_BASE, UART_LEN)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { UART3_IT }
    })
  }
}
