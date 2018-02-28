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

!include ../NxpQoriqLs.dsc
!include ../../../Silicon/NXP/LS1046A/LS1046A.dsc

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

[PcdsFixedAtBuild.common]

  #
  # LS1046a board Specific PCDs
  # XX (DRAM - Region 1 2GB)
  # (NOR - IFC Region 1 512MB)
  gArmTokenSpaceGuid.PcdSystemMemoryBase|0x80000000
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x80000000
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

  #
  # I2C controller Pcds
  #
  gNxpQoriqLsTokenSpaceGuid.PcdI2cBus|3

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
  Silicon/NXP/Drivers/PciCpuIo2Dxe/PciCpuIo2Dxe.inf
  MdeModulePkg/Bus/Pci/PciHostBridgeDxe/PciHostBridgeDxe.inf {
    <PcdsFixedAtBuild>
  gEfiMdePkgTokenSpaceGuid.PcdDebugPrintErrorLevel|0x8010004F
  }
  MdeModulePkg/Bus/Pci/PciBusDxe/PciBusDxe.inf

  Silicon/NXP/Drivers/QspiDxe/QspiDxe.inf
  Silicon/NXP/Drivers/SpiBusDxe/SpiBusDxe.inf
  Silicon/NXP/Drivers/SpiNorFlashDxe/SpiNorFlashDxe.inf

  Silicon/NXP/Drivers/SataInitDxe/SataInitDxe.inf
  #
  # DT support
  #
  Platform/NXP/LS1046aRdbPkg/DeviceTree/DeviceTree.inf

 ##
