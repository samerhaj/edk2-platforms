/** FpgaLib.h
*  Header defining the LS1046a Fpga specific constants (Base addresses, sizes, flags)
*
*  Copyright 2017 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#ifndef __LS1046A_FPGA_H__
#define __LS1046A_FPGA_H__

/**
   FPGA register set of LS1046ARDB board-specific.
 **/
typedef struct {
  UINT8  FpgaVersionMajor; // 0x0 - FPGA Major Revision Register
  UINT8  FpgaVersionMinor; // 0x1 - FPGA Minor Revision Register
  UINT8  PcbaVersion;      // 0x2 - PCBA Revision Register
  UINT8  SystemReset;      // 0x3 - system reset register
  UINT8  SoftMuxOn;        // 0x4 - Switch Control Enable Register
  UINT8  RcwSource1;       // 0x5 - Reset config word 1
  UINT8  RcwSource2;       // 0x6 - Reset config word 1
  UINT8  Vbank;            // 0x7 - Flash bank selection Control
  UINT8  SysclkSelect;     // 0x8 - System clock selection Control
  UINT8  UartSel;          // 0x9 - Uart selection Control
  UINT8  Sd1RefClkSel;     // 0xA - Serdes1 reference clock selection Control
  UINT8  TdmClkMuxSel;     // 0xB - TDM Clock Mux selection Control
  UINT8  SdhcSpiCsSel;     // 0xC - SDHC/SPI Chip select selection Control
  UINT8  StatusLed;        // 0xD - Status Led
  UINT8  GlobalReset;      // 0xE - Global reset
  UINT8  SdEmmc;           // 0xF - SD or EMMC Interface Control Regsiter
  UINT8  VddEn;            // 0x10 - VDD Voltage Control Enable Register
  UINT8  VddSel;           // 0x11 - VDD Voltage Control Register
} FPGA_REG_SET;

/**
   Function to read FPGA register.
**/
UINT8
FpgaRead (
  UINTN  Reg
  );

/**
   Function to write FPGA register.
**/
VOID
FpgaWrite (
  UINTN  Reg,
  UINT8  Value
  );

/**
   Function to read FPGA revision.
**/
VOID
FpgaRevBit (
  UINT8  *Value
  );

/**
   Function to initialize FPGA timings.
**/
VOID
FpgaInit (
  VOID
  );

#define FPGA_BASE_PHYS          0x7fb00000

#define SRC_VBANK               0x25
#define SRC_NAND                0x106
#define SRC_QSPI                0x44
#define SRC_SD                  0x40

#define SERDES_FREQ1            "100.00 MHz"
#define SERDES_FREQ2            "156.25 MHz"

#define FPGA_READ(Reg)          FpgaRead (OFFSET_OF (FPGA_REG_SET, Reg))
#define FPGA_WRITE(Reg, Value)  FpgaWrite (OFFSET_OF (FPGA_REG_SET, Reg), Value)

#endif
