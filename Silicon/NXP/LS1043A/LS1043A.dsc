#  LS1043A.dsc
#  LS1043A Soc package.
#
#  Copyright 2017 NXP
#
#  This program and the accompanying materials
#  are licensed and made available under the terms and conditions of the BSD License
#  which accompanies this distribution. The full text of the license may be found at
#  http://opensource.org/licenses/bsd-license.php
#
#  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
#  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
#
#

################################################################################
#
# Pcd Section - list of all EDK II PCD Entries defined by this Platform
#
################################################################################
[PcdsDynamicDefault.common]

  #
  # ARM General Interrupt Controller
  gArmTokenSpaceGuid.PcdGicDistributorBase|0x01401000
  gArmTokenSpaceGuid.PcdGicInterruptInterfaceBase|0x01402000

[LibraryClasses.common]
  SocClockLib|Silicon/NXP/LS1043A/Library/SocClockLib/SocClockLib.inf
  SocFixupLib|Silicon/NXP/LS1043A/Library/SocFixupLib/SocFixupLib.inf

[PcdsFixedAtBuild.common]

  #
  # CCSR Address Space and other attached Memories
  #
  gNxpQoriqLsTokenSpaceGuid.PcdCcsrBaseAddr|0x01000000
  gNxpQoriqLsTokenSpaceGuid.PcdCcsrSize|0x0F000000
  gNxpQoriqLsTokenSpaceGuid.PcdClkBaseAddr|0x01EE1000
  gNxpQoriqLsTokenSpaceGuid.PcdIfcRegion1BaseAddr|0x60000000
  gNxpQoriqLsTokenSpaceGuid.PcdIfcRegion1Size|0x20000000
  gNxpQoriqLsTokenSpaceGuid.PcdIfcRegion2BaseAddr|0x0620000000
  gNxpQoriqLsTokenSpaceGuid.PcdIfcRegion2Size|0x00E0000000
  gNxpQoriqLsTokenSpaceGuid.PcdIfcNandReservedSize|0x2EA
  gNxpQoriqLsTokenSpaceGuid.PcdQmanSwpBaseAddr|0x0500000000
  gNxpQoriqLsTokenSpaceGuid.PcdQmanSwpSize|0x0080000000
  gNxpQoriqLsTokenSpaceGuid.PcdBmanSwpBaseAddr|0x0508000000
  gNxpQoriqLsTokenSpaceGuid.PcdBmanSwpSize|0x0080000000
  gNxpQoriqLsTokenSpaceGuid.PcdPciExp1BaseAddr|0x4000000000
  gNxpQoriqLsTokenSpaceGuid.PcdPciExp1BaseSize|0x800000000
  gNxpQoriqLsTokenSpaceGuid.PcdPciExp2BaseAddr|0x4800000000
  gNxpQoriqLsTokenSpaceGuid.PcdPciExp2BaseSize|0x800000000
  gNxpQoriqLsTokenSpaceGuid.PcdPciExp3BaseAddr|0x5000000000
  gNxpQoriqLsTokenSpaceGuid.PcdPciExp3BaseSize|0x800000000
  gNxpQoriqLsTokenSpaceGuid.PcdPciFdtCompatible|"fsl,ls1043a-pcie"
  gNxpQoriqLsTokenSpaceGuid.PcdScfgBaseAddr|0x1570000
  gNxpQoriqLsTokenSpaceGuid.PcdScfgIntPol|0x40000000
  gNxpQoriqLsTokenSpaceGuid.PcdGutsBaseAddr|0x01EE0000
  gNxpQoriqLsTokenSpaceGuid.PcdWdog1BaseAddr|0x02AD0000
  gNxpQoriqLsTokenSpaceGuid.PcdSdxcBaseAddr|0x01560000
  gNxpQoriqLsTokenSpaceGuid.PcdI2c0BaseAddr|0x02180000
  gNxpQoriqLsTokenSpaceGuid.PcdI2cSize|0x10000
  gNxpQoriqLsTokenSpaceGuid.PcdNumI2cController|4
  gNxpQoriqLsTokenSpaceGuid.PcdQspiRegionBaseAddr|0x40000000
  gNxpQoriqLsTokenSpaceGuid.PcdQspiRegionSize|0x20000000
  gNxpQoriqLsTokenSpaceGuid.PcdIfcBaseAddr|0x1530000
  gNxpQoriqLsTokenSpaceGuid.PcdNumPciController|3
  gNxpQoriqLsTokenSpaceGuid.PcdRomBaseAddr|0x00000000
  gNxpQoriqLsTokenSpaceGuid.PcdRomSize|0x00100000
  gNxpQoriqLsTokenSpaceGuid.PcdUsbPhy1BaseAddress|0x084F0000
  gNxpQoriqLsTokenSpaceGuid.PcdUsbPhy2BaseAddress|0x08500000
  gNxpQoriqLsTokenSpaceGuid.PcdUsbPhy3BaseAddress|0x08510000

##
