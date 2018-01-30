#  LS1043aRdbPkg.dsc
#
#  LS1043ARDB Board package.
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
  PLATFORM_NAME                  = LS1043aRdbPkg
  PLATFORM_GUID                  = 60169ec4-d2b4-44f8-825e-f8684fd42e4f
  OUTPUT_DIRECTORY               = Build/LS1043aRdbPkg
  FLASH_DEFINITION               = Platform/NXP/LS1043aRdbPkg/LS1043aRdbPkg.fdf

!include ../NxpQoriqLs.dsc
!include ../../../Silicon/NXP/LS1043A/LS1043A.dsc

[LibraryClasses.common]
  ArmPlatformLib|Platform/NXP/LS1043aRdbPkg/Library/PlatformLib/ArmPlatformLib.inf
  ResetSystemLib|ArmPkg/Library/ArmSmcPsciResetSystemLib/ArmSmcPsciResetSystemLib.inf
  SerialPortLib|Silicon/NXP/Library/DUartPortLib/DUartPortLib.inf
  BeIoLib|Silicon/NXP/Library/BeIoLib/BeIoLib.inf
  SocLib|Silicon/NXP/Chassis/LS1043aSocLib.inf
  RealTimeClockLib|Silicon/Maxim/Library/Ds1307RtcLib/Ds1307RtcLib.inf
  IfcLib|Silicon/NXP/Library/IfcLib/IfcLib.inf
  BoardLib|Platform/NXP/LS1043aRdbPkg/Library/BoardLib/BoardLib.inf
  FpgaLib|Platform/NXP/LS1043aRdbPkg/Library/FpgaLib/FpgaLib.inf

[PcdsFixedAtBuild.common]

  #
  # LS1043a board Specific PCDs
  # XX (DRAM - Region 1 2GB)
  # (NOR - IFC Region 1 512MB)
  gArmTokenSpaceGuid.PcdSystemMemoryBase|0x80000000
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x80000000
  gArmPlatformTokenSpaceGuid.PcdSystemMemoryUefiRegionSize|0x02000000

  #
  # Board Specific Pcds
  #
  gEfiMdeModulePkgTokenSpaceGuid.PcdSerialRegisterBase|0x021c0500
  gNxpQoriqLsTokenSpaceGuid.PcdSerdes2Enabled|FALSE
  gNxpQoriqLsTokenSpaceGuid.PcdPlatformFreqDiv|0x1

  #
  # Big Endian IPs
  #
  gNxpQoriqLsTokenSpaceGuid.PcdGurBigEndian|TRUE
  gNxpQoriqLsTokenSpaceGuid.PcdWdogBigEndian|TRUE
  gNxpQoriqLsTokenSpaceGuid.PcdIfcBigEndian|TRUE

  #
  # I2C controller Pcds
  #
  gNxpQoriqLsTokenSpaceGuid.PcdI2cBus|0

  #
  # RTC Pcds
  #
  gDs1307RtcLibTokenSpaceGuid.PcdI2cSlaveAddress|0x68
  gDs1307RtcLibTokenSpaceGuid.PcdI2cBusFrequency|100000

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

 ##
