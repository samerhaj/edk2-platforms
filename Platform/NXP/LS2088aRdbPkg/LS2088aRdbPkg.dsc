#  LS2088aRdbPkg.dsc
#
#  LS2088ARDB Board package.
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
  PLATFORM_NAME                  = LS2088aRdbPkg
  PLATFORM_GUID                  = be06d8bc-05eb-44d6-b39f-191e93617ebd
  OUTPUT_DIRECTORY               = Build/LS2088aRdbPkg
  FLASH_DEFINITION               = Platform/NXP/LS2088aRdbPkg/LS2088aRdbPkg.fdf
  DEFINE MC_HIGH_MEM             = TRUE

!include ../NxpQoriqLs.dsc
!include ../../../Silicon/NXP/LS2088A/LS2088A.dsc

[LibraryClasses.common]
  ArmPlatformLib|Platform/NXP/LS2088aRdbPkg/Library/PlatformLib/ArmPlatformLib.inf
  ResetSystemLib|ArmPkg/Library/ArmSmcPsciResetSystemLib/ArmSmcPsciResetSystemLib.inf
  SerialPortLib|Silicon/NXP/Library/DUartPortLib/DUartPortLib.inf
  BeIoLib|Silicon/NXP/Library/BeIoLib/BeIoLib.inf
  SocLib|Silicon/NXP/Chassis/LS2088aSocLib.inf
  RealTimeClockLib|Silicon/Maxim/Library/Ds3232RtcLib/Ds3232RtcLib.inf
  IfcLib|Silicon/NXP/Library/IfcLib/IfcLib.inf
  BoardLib|Platform/NXP/LS2088aRdbPkg/Library/BoardLib/BoardLib.inf
  FpgaLib|Platform/NXP/LS2088aRdbPkg/Library/FpgaLib/FpgaLib.inf
  NorFlashLib|Silicon/NXP/Library/NorFlashLib/NorFlashLib.inf
  PciSegmentLib|Silicon/NXP/Library/PciSegmentLib/PciSegmentLib.inf
  PciHostBridgeLib|Silicon/NXP/Library/PciHostBridgeLib/PciHostBridgeLib.inf
  MmcLib|Silicon/NXP/Library/MmcLib/MmcLib.inf
  Dpaa2BoardSpecificLib|Platform/NXP/LS2088aRdbPkg/Library/Dpaa2BoardSpecificLib/Dpaa2BoardSpecificLib.inf
  Dpaa2EthernetMacLib|Silicon/NXP/Library/Dpaa2EthernetMacLib/Dpaa2EthernetMacLib.inf
  Dpaa2EthernetPhyLib|Silicon/NXP/Library/Dpaa2EthernetPhyLib/Dpaa2EthernetPhyLib.inf
  Dpaa2ManagementComplexLib|Silicon/NXP/Library/Dpaa2ManagementComplexLib/Dpaa2ManagementComplexLib.inf
  Dpaa2McInterfaceLib|Silicon/NXP/Library/Dpaa2McInterfaceLib/Dpaa2McInterfaceLib.inf
  SecureMonRngLib|Silicon/NXP/Library/SecureMonRngLib/SecureMonRngLib.inf
  MemoryInitPeiLib|Silicon/NXP/Library/MemoryInitPei/MemoryInitPeiLib.inf

[PcdsFixedAtBuild.common]

!if $(MC_HIGH_MEM) == TRUE                                        # Management Complex loaded at the end of DDR2
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2McHighRamSize|0x80000000      # 2GB (must be 512MB aligned)
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2McLowRamSize|0x0
  gNxpQoriqLsTokenSpaceGuid.PcdMcHighMemSupport|1
  gArmTokenSpaceGuid.PcdSystemMemoryBase|0x0080000000             # Actual base
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x007BE00000             # 2G - 66MB (ATF)
!else
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2McHighRamSize|0x0             # 512MB (Fixed)
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2McLowRamSize|0x20000000       # 512MB (Fixed)
  gNxpQoriqLsTokenSpaceGuid.PcdMcHighMemSupport|0
  gArmTokenSpaceGuid.PcdSystemMemoryBase|0x00A0000000             # Actual base + 512MB
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x005BE00000             # 2G - 512MB - 66MB (ATF)
!endif
  gArmPlatformTokenSpaceGuid.PcdSystemMemoryUefiRegionSize|0x02000000

  #
  # Board Specific Pcds
  #
  gEfiMdeModulePkgTokenSpaceGuid.PcdSerialRegisterBase|0x21c0600
  gNxpQoriqLsTokenSpaceGuid.PcdSerdes2Enabled|TRUE
  gNxpQoriqLsTokenSpaceGuid.PcdPlatformFreqDiv|0x2
  gNxpQoriqLsTokenSpaceGuid.PcdDdrClk|133333333

  #
  # I2C controller Pcds
  #
  gNxpQoriqLsTokenSpaceGuid.PcdI2cBus|0

  gNxpQoriqLsTokenSpaceGuid.PcdSysEepromI2cBus|0
  gNxpQoriqLsTokenSpaceGuid.PcdSysEepromI2cAddress|0x57

  #
  # RTC Pcds
  #
  gDs3232RtcLibTokenSpaceGuid.PcdI2cSlaveAddress|0x68
  gDs3232RtcLibTokenSpaceGuid.PcdI2cBusFrequency|100000
  gDs3232RtcLibTokenSpaceGuid.PcdIsRtcDeviceMuxed|TRUE
  gDs3232RtcLibTokenSpaceGuid.PcdMuxDeviceAddress|0x75
  gDs3232RtcLibTokenSpaceGuid.PcdMuxControlRegOffset|0x09
  gDs3232RtcLibTokenSpaceGuid.PcdMuxRtcChannelValue|0x09
  gDs3232RtcLibTokenSpaceGuid.PcdMuxDefaultChannelValue|0x08

  #
  # NV Storage PCDs.
  #
  gArmTokenSpaceGuid.PcdVFPEnabled|1
  gNxpQoriqLsTokenSpaceGuid.PcdFlashDeviceBase64|0x580000000
  gNxpQoriqLsTokenSpaceGuid.PcdFlashReservedRegionBase64|0x580300000

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
  
  #
  # DPAA2 Pcds
  #
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2Initialize|TRUE
  gNxpQoriqLsTokenSpaceGuid.PcdDisableMcLogging|FALSE
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2McFwSrc|0x01
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2StreamIdStart|23
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2StreamIdEnd|63
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2McBootTimeoutMs|200000
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2UsedDpmacsMask|0xff
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2McLogMcDramOffset|0x1000000

  # Valid values for PcdDpaa2McLogLevel:
  # - 0x01: LOG_LEVEL_DEBUG
  # - 0x02: LOG_LEVEL_INFO
  # - 0x03: LOG_LEVEL_WARNING
  # - 0x04: LOG_LEVEL_ERROR
  # - 0x05: LOG_LEVEL_CRITICAL
  # - 0x06: LOG_LEVEL_ASSERT
  # - 0xFF: LOG_LEVEL_DEFAULT (default from DPC)
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2McLogLevel|0xff

  # Valid values for PcdDpaaDebugFlags:
  # - 0x0      DPAA debug logs are disabled.
  # - 0x1      Enable DPAA debugging messages
  # - 0x2      Dump values of RAM words or registers
  # - 0x4      Trace commands sent to the MC
  # - 0x8      Dump MC log fragment
  # - 0x10     Dump contents of the root DPRC
  # - 0x20     Perform extra checks
  # - 0x40     Trace network packets sent/received
  gNxpQoriqLsTokenSpaceGuid.PcdDpaaDebugFlags|0x0

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
  }
  MdeModulePkg/Universal/FaultTolerantWriteDxe/FaultTolerantWriteDxe.inf
  ArmPlatformPkg/Drivers/SP805WatchdogDxe/SP805WatchdogDxe.inf
  Silicon/NXP/Drivers/I2cDxe/I2cDxe.inf
  Silicon/NXP/Drivers/NorFlashDxe/NorFlashDxe.inf
  Silicon/NXP/Drivers/UsbHcdInitDxe/UsbHcd.inf
  Silicon/NXP/Drivers/PciCpuIo2Dxe/PciCpuIo2Dxe.inf
  MdeModulePkg/Bus/Pci/PciHostBridgeDxe/PciHostBridgeDxe.inf {
    <PcdsFixedAtBuild>
  gEfiMdePkgTokenSpaceGuid.PcdDebugPrintErrorLevel|0x8010004F
  }
  MdeModulePkg/Bus/Pci/PciBusDxe/PciBusDxe.inf

  Silicon/NXP/Drivers/LanIntelE1000Dxe/LanIntelE1000Dxe.inf

  Silicon/NXP/Drivers/SataInitDxe/SataInitDxe.inf
  Silicon/NXP/Drivers/MmcHostDxe/MmcHostDxe.inf
  Silicon/NXP/Drivers/Dpaa2EthernetDxe/Dpaa2EthernetDxe.inf
  Silicon/NXP/Drivers/NandFlashDxe/NandFlashDxe.inf
  Silicon/NXP/Drivers/RngDxe/RngDxe.inf

 ##
