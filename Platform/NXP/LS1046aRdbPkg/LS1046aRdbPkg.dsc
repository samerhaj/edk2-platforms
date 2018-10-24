#  LS1046aRdbPkg.dsc
#
#  LS1046ARDB Board package.
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

################################################################################
#
# Defines Section - statements that will be processed to create a Makefile.
#
################################################################################
[Defines]
  #
  # Defines for default states.  These can be changed on the command line.
  # -D FLAG=VALUE
  #
  PLATFORM_NAME                  = LS1046aRdbPkg
  PLATFORM_GUID                  = 43920156-3f3b-4199-9b29-c6db1fb792b0
  OUTPUT_DIRECTORY               = Build/LS1046aRdbPkg
  FLASH_DEFINITION               = Platform/NXP/LS1046aRdbPkg/LS1046aRdbPkg.fdf

!include Platform/NXP/NxpQoriqLs.dsc
!include Silicon/NXP/Chassis/Chassis2/Chassis2.dsc
!include Silicon/NXP/LS1046A/LS1046A.dsc

[LibraryClasses.common]
  ArmPlatformLib|Platform/NXP/LS1046aRdbPkg/Library/PlatformLib/ArmPlatformLib.inf
  ResetSystemLib|ArmPkg/Library/ArmSmcPsciResetSystemLib/ArmSmcPsciResetSystemLib.inf
  SerialPortLib|Silicon/NXP/Library/DUartPortLib/DUartPortLib.inf
  BeIoLib|Silicon/NXP/Library/BeIoLib/BeIoLib.inf
  SocLib|Silicon/NXP/Chassis/LS1046aSocLib.inf
  RealTimeClockLib|Silicon/NXP/Library/Pcf2129RtcLib/Pcf2129RtcLib.inf
  IfcLib|Silicon/NXP/Library/IfcLib/IfcLib.inf
  BoardLib|Platform/NXP/LS1046aRdbPkg/Library/BoardLib/BoardLib.inf
  FpgaLib|Platform/NXP/LS1046aRdbPkg/Library/FpgaLib/FpgaLib.inf
  PciSegmentLib|Silicon/NXP/Library/PciSegmentLib/PciSegmentLib.inf
  PciHostBridgeLib|Silicon/NXP/Library/PciHostBridgeLib/PciHostBridgeLib.inf
  MmcLib|Silicon/NXP/Library/MmcLib/MmcLib.inf
  SecureMonRngLib|Silicon/NXP/Library/SecureMonRngLib/SecureMonRngLib.inf
  MemoryInitPeiLib|Silicon/NXP/Library/MemoryInitPei/MemoryInitPeiLib.inf

  #
  # DPAA1
  #
  Dpaa1Lib|Silicon/NXP/Library/Dpaa1Lib/Dpaa1Lib.inf
  Dpaa1EthernetMacLib|Silicon/NXP/Library/Dpaa1EthernetMacLib/Dpaa1EthernetMacLib.inf
  Dpaa1EthernetPhyLib|Silicon/NXP/Library/Dpaa1EthernetPhyLib/Dpaa1EthernetPhyLib.inf
  Dpaa1BoardLib|Platform/NXP/LS1046aRdbPkg/Library/Dpaa1BoardLib/Dpaa1BoardLib.inf

[PcdsFixedAtBuild.common]

  #
  # LS1046a board Specific PCDs
  # XX (DRAM - Region 1 2GB - 66 MB)
  # (NOR - IFC Region 1 512MB)
  gArmTokenSpaceGuid.PcdSystemMemoryBase|0x80000000
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x007BE00000
  gArmPlatformTokenSpaceGuid.PcdSystemMemoryUefiRegionSize|0x02000000

  #
  # Board Specific Pcds
  #
  gEfiMdeModulePkgTokenSpaceGuid.PcdSerialRegisterBase|0x021c0500
  gNxpQoriqLsTokenSpaceGuid.PcdSerdes2Enabled|TRUE
  gNxpQoriqLsTokenSpaceGuid.PcdPlatformFreqDiv|0x2

  #
  # Big Endian IPs
  #
  gNxpQoriqLsTokenSpaceGuid.PcdGurBigEndian|TRUE
  gNxpQoriqLsTokenSpaceGuid.PcdWdogBigEndian|TRUE
  gNxpQoriqLsTokenSpaceGuid.PcdIfcBigEndian|TRUE
  gNxpQoriqLsTokenSpaceGuid.PcdPciLutBigEndian|TRUE
  gNxpQoriqLsTokenSpaceGuid.PcdMmcBigEndian|TRUE

  #
  # I2C controller Pcds
  #
  gNxpQoriqLsTokenSpaceGuid.PcdI2cBus|3

  gNxpQoriqLsTokenSpaceGuid.PcdSysEepromI2cBus|0
  gNxpQoriqLsTokenSpaceGuid.PcdSysEepromI2cAddress|0x53

  #
  # RTC Pcds
  #
  gNxpQoriqLsTokenSpaceGuid.PcdI2cSlaveAddress|0x51
  gNxpQoriqLsTokenSpaceGuid.PcdI2cSpeed|100000

  #
  # PCI PCDs.
  #
  gNxpQoriqLsTokenSpaceGuid.PcdPciDebug|FALSE
  gNxpQoriqLsTokenSpaceGuid.PcdPcieLutBase|0x80000
  gNxpQoriqLsTokenSpaceGuid.PcdPcieLutDbg|0x407FC

  #
  # SATA Pcds
  #
  gNxpQoriqLsTokenSpaceGuid.PcdSataErratumA009185|TRUE
  gNxpQoriqLsTokenSpaceGuid.PcdSataErratumA010554|TRUE
  gNxpQoriqLsTokenSpaceGuid.PcdSataErratumA010635|TRUE

  gNxpQoriqLsTokenSpaceGuid.PcdFmanFwFlashAddr|0x40900000

 #
 # DPAA1 Pcds
 #
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa1Initialize|TRUE
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa1DebugFlags|0x0
  gNxpQoriqLsTokenSpaceGuid.PcdFManFwFlashAddr|0x40900000
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa1UsedMemacsMask|0x33C
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa1FmanMdio1Addr|0x01AFC000
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa1FmanMdio2Addr|0x01AFD000
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa1FmanAddr|0x01a00000
  gNxpQoriqLsTokenSpaceGuid.PcdFManFwFlashAddr|0x40900000
  gNxpQoriqLsTokenSpaceGuid.PcdSgmiiPrtclInit|TRUE


################################################################################
#
# Components Section - list of all EDK II Modules needed by this Platform
#
################################################################################
[Components.common]
  #
  # Architectural Protocols
  #
  MdeModulePkg/Universal/Variable/EmuRuntimeDxe/EmuVariableRuntimeDxe.inf

  Silicon/NXP/Drivers/WatchDog/WatchDogDxe.inf
  Silicon/NXP/Drivers/I2cDxe/I2cDxe.inf
  EmbeddedPkg/RealTimeClockRuntimeDxe/RealTimeClockRuntimeDxe.inf
  Silicon/NXP/Drivers/PciCpuIo2Dxe/PciCpuIo2Dxe.inf
  MdeModulePkg/Bus/Pci/PciHostBridgeDxe/PciHostBridgeDxe.inf {
    <PcdsFixedAtBuild>
  gEfiMdePkgTokenSpaceGuid.PcdDebugPrintErrorLevel|0x8010004F
  }
  MdeModulePkg/Bus/Pci/PciBusDxe/PciBusDxe.inf

  Silicon/NXP/Drivers/LanIntelE1000Dxe/LanIntelE1000Dxe.inf

  Silicon/NXP/Drivers/QspiDxe/QspiDxe.inf
  Silicon/NXP/Drivers/SpiBusDxe/SpiBusDxe.inf
  Silicon/NXP/Drivers/SpiNorFlashDxe/SpiNorFlashDxe.inf
  Silicon/NXP/Drivers/SpiConfigurationDxe/SpiConfigurationDxe.inf

  Silicon/NXP/Drivers/SataInitDxe/SataInitDxe.inf

  #
  # DPAA1 Ethernet driver
  #
  Silicon/NXP/Drivers/Dpaa1Ethernet/Dpaa1EthernetDxe.inf

  #
  # DT support
  #
  Platform/NXP/LS1046aRdbPkg/DeviceTree/DeviceTree.inf
  Silicon/NXP/Drivers/DtPlatformDxe/DtPlatformDxe.inf
  Silicon/NXP/Drivers/MmcHostDxe/MmcHostDxe.inf
  Silicon/NXP/Drivers/NandFlashDxe/NandFlashDxe.inf
  Silicon/NXP/Drivers/RngDxe/RngDxe.inf
  Silicon/NXP/Drivers/Dpaa1EthernetDxe/Dpaa1EthernetDxe.inf

 ##
