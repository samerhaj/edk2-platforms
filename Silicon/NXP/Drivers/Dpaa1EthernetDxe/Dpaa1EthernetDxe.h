/** @file
*
*  Copyright 2018 NXP
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

#ifndef __DPAA1_ETHERNET_DXE_H__
#define __DPAA1_ETHERNET_DXE_H__

#define BMAN_IP_REV_1 0xBF8
#define BMAN_IP_REV_2 0xBFC
#define QMAN_IP_REV_1 0xBF8
#define QMAN_IP_REV_2 0xBFC

// structure That Defines QE Firmware Binary Files.
//
// See Doc/README.Qe_Firmware For A Description Of These Fields.
//
#pragma pack (1)
typedef struct _QeFirmware {
  struct QeHeader {
    UINT32 Length;  // Length Of The Entire structure, In Bytes
    UINT8 Magic[3];  // Set To { 'Q', 'E', 'F' }
    UINT8 Version;  // Version Of This Layout. First Ver Is '1'
  } Header;
  UINT8 Id[62];    // Null-Terminated Identifier String
  UINT8 Split;    // 0 = Shared I-RAM, 1 = Split I-RAM
  UINT8 Count;    // Number Of Microcode[] structures
  struct {
    UINT16 Model;  // The SOC Model
    UINT8 Major;  // The SOC Revision Major
    UINT8 Minor;  // The SOC Revision Minor
  } Soc;
  UINT8 Padding[4];    // Reserved, For Alignment
  UINT64 ExtendedModes;  // Extended Modes
  UINT32 Vtraps[8];    // Virtual Trap Addresses
  UINT8 Reserved[4];    // Reserved, For Future Expansion
  struct QeMicrocode {
    UINT8 Id[32];  // Null-Terminated Identifier
    UINT32 Traps[16];  // Trap Addresses, 0 == Ignore
    UINT32 Eccr;  // The Value For The ECCR Register
    UINT32 IramOffset;// Offset Into I-RAM For The Code
    UINT32 Count;  // Number Of 32-Bit Words Of The Code
    UINT32 CodeOffset;// Offset Of The Actual Microcode
    UINT8 Major;  // The Microcode Version Major
    UINT8 Minor;  // The Microcode Version Minor
    UINT8 Revision;  // The Microcode Version Revision
    UINT8 Padding;  // Reserved, For Alignment
    UINT8 Reserved[4];  // Reserved, For Future Expansion
  } Microcode[1];
  // All Microcode Binaries Should Be Located Here
  // CRC32 Should Be Located Here, After The Microcode Binaries
} QeFirmware;
#pragma pack (1)

#endif // __DPAA1_ETHERNET_DXE_H__

