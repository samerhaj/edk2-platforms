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

#include "Platform.h"

DefinitionBlock("DsdtTable.aml", "DSDT", 1, "NXP  ", "LX2160  ", EFI_ACPI_ARM_OEM_REVISION) {
  include ("Com.asl")
  include ("CPU.asl")
  include ("Esdhc.asl")
  include ("FSPI.asl")
  include ("I2c.asl")
  include ("Sata.asl")
  include ("SPI.asl")
  include ("Usb.asl")
  include ("Mc.asl")
  include ("Mdio.asl")
  include ("Pci.asl")
}
