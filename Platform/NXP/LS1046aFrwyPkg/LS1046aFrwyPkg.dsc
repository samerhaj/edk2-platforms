#  LS1046aFrwyPkg.dsc
#
#  LS1046AFRWY Board package.
#
#  Copyright 2019 NXP
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
  PLATFORM_NAME                  = LS1046aFrwyPkg
  PLATFORM_GUID                  = 79adaa48-5f50-49f0-aa9a-544ac9260ef8
  OUTPUT_DIRECTORY               = Build/LS1046aFrwyPkg
  FLASH_DEFINITION               = Platform/NXP/LS1046aFrwyPkg/LS1046aFrwyPkg.fdf

!include Platform/NXP/NxpQoriqLs.dsc
!include Silicon/NXP/Chassis/Chassis2/Chassis2.dsc
!include Silicon/NXP/LS1046A/LS1046A.dsc

[LibraryClasses.common]
  ArmPlatformLib|Platform/NXP/LS1046aFrwyPkg/Library/PlatformLib/ArmPlatformLib.inf
  ResetSystemLib|ArmPkg/Library/ArmSmcPsciResetSystemLib/ArmSmcPsciResetSystemLib.inf
  SerialPortLib|Silicon/NXP/Library/DUartPortLib/DUartPortLib.inf
  BeIoLib|Silicon/NXP/Library/BeIoLib/BeIoLib.inf
  SocLib|Silicon/NXP/Chassis/LS1046aSocLib.inf
  RealTimeClockLib|Silicon/NXP/Library/Pcf2129RtcLib/Pcf2129RtcLib.inf
  IfcLib|Silicon/NXP/Library/IfcLib/IfcLib.inf
  BoardLib|Platform/NXP/LS1046aFrwyPkg/Library/BoardLib/BoardLib.inf
  FpgaLib|Platform/NXP/LS1046aFrwyPkg/Library/FpgaLib/FpgaLib.inf
  PciSegmentLib|Silicon/NXP/Library/PciSegmentLib/PciSegmentLib.inf
  PciHostBridgeLib|Silicon/NXP/Library/PciHostBridgeLib/PciHostBridgeLib.inf
  MmcLib|Silicon/NXP/Library/MmcLib/MmcLib.inf
  SecureMonRngLib|Silicon/NXP/Library/SecureMonRngLib/SecureMonRngLib.inf
  MemoryInitPeiLib|Silicon/NXP/Library/MemoryInitPei/MemoryInitPeiLib.inf

[PcdsFixedAtBuild.common]

  #
  # FRWY-LS1046A board Specific PCDs
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
  gNxpQoriqLsTokenSpaceGuid.PcdI2cBus|0

  gNxpQoriqLsTokenSpaceGuid.PcdSysEepromI2cBus|0
  gNxpQoriqLsTokenSpaceGuid.PcdSysEepromI2cAddress|0x52

  #
  # RTC Pcds
  #
  gNxpQoriqLsTokenSpaceGuid.PcdI2cSlaveAddress|0x51
  gNxpQoriqLsTokenSpaceGuid.PcdI2cSpeed|100000
  gNxpQoriqLsTokenSpaceGuid.PcdIsRtcDeviceMuxed|TRUE
  gNxpQoriqLsTokenSpaceGuid.PcdMuxDeviceAddress|0x77
  gNxpQoriqLsTokenSpaceGuid.PcdMuxControlRegOffset|0x00
  gNxpQoriqLsTokenSpaceGuid.PcdMuxRtcChannelValue|0x01
  gNxpQoriqLsTokenSpaceGuid.PcdMuxDefaultChannelValue|0x01

  #
  # PCI PCDs.
  #
  gNxpQoriqLsTokenSpaceGuid.PcdPciDebug|FALSE
  gNxpQoriqLsTokenSpaceGuid.PcdPcieLutBase|0x80000
  gNxpQoriqLsTokenSpaceGuid.PcdPcieLutDbg|0x407FC

  gNxpQoriqLsTokenSpaceGuid.PcdFmanFwFlashAddr|0x40900000

  gNxpQoriqLsTokenSpaceGuid.PcdFdtAddress|0x40F00000

################################################################################
#
# Components Section - list of all EDK II Modules needed by this Platform
#
################################################################################
[Components.common]
  #
  # Architectural Protocols
  #
  MdeModulePkg/Universal/Variable/RuntimeDxe/VariableRuntimeDxe.inf{
     <LibraryClasses>
     NULL|MdeModulePkg/Library/VarCheckUefiLib/VarCheckUefiLib.inf
     NULL|EmbeddedPkg/Library/NvVarStoreFormattedLib/NvVarStoreFormattedLib.inf
  }
  #MdeModulePkg/Universal/Variable/EmuRuntimeDxe/EmuVariableRuntimeDxe.inf
  MdeModulePkg/Universal/FaultTolerantWriteDxe/FaultTolerantWriteDxe.inf

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

  #
  # DT support
  #
  Platform/NXP/LS1046aFrwyPkg/DeviceTree/DeviceTree.inf
  Silicon/NXP/Drivers/DtPlatformDxe/DtPlatformDxe.inf
  Silicon/NXP/Drivers/MmcHostDxe/MmcHostDxe.inf
  Silicon/NXP/Drivers/RngDxe/RngDxe.inf

 ##
