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

[PcdsFixedAtBuild.common]

!if $(MC_HIGH_MEM) == TRUE                                        # Management Complex loaded at the end of DDR2
  gNxpQoriqLsTokenSpaceGuid.PcdDram1BaseAddr|0x0080000000         # Actual base address (0x0080000000)
  gNxpQoriqLsTokenSpaceGuid.PcdDram1Size|0x0080000000             # 2 GB
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2McRamSize|0x80000000          # 2GB (PcdDpaa2McRamSize must be 512MB aligned)
  gNxpQoriqLsTokenSpaceGuid.PcdMcHighMemSupport|1
  gArmTokenSpaceGuid.PcdSystemMemoryBase|0x0080000000             # Actual base
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x0080000000             # 2G
!else
  gNxpQoriqLsTokenSpaceGuid.PcdDram1BaseAddr|0x00A0000000         # Actual base address (0x0080000000) + 512MB
  gNxpQoriqLsTokenSpaceGuid.PcdDram1Size|0x0060000000             # 2GB - 512MB
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2McRamSize|0x20000000          # 512MB (Fixed)
  gNxpQoriqLsTokenSpaceGuid.PcdMcHighMemSupport|0
  gArmTokenSpaceGuid.PcdSystemMemoryBase|0x00A0000000             # Actual base + 512MB
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x0060000000             # 2G - 512MB
!endif
  gNxpQoriqLsTokenSpaceGuid.PcdDramMemSize|0x380000000            # 14 GB
  gNxpQoriqLsTokenSpaceGuid.PcdDram2BaseAddr|0x8080000000
  gNxpQoriqLsTokenSpaceGuid.PcdDram2Size|0x8800000000             # 512 GB
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
