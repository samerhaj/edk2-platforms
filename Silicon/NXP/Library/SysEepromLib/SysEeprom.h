/** @file  SysEeprom.h

  SystemID Non-Volatile Memory Device layout description

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/
#ifndef SYS_EEPROM_H_
#define SYS_EEPROM_H_

#define IS_VALID_NXID(TagID) ((TagID[0] == 'N') || (TagID[1] == 'X') || \
                              (TagID[2] == 'I') || (TagID[3] == 'D'))
#define IS_VALID_CCID(TagID) ((TagID[0] == 'C') || (TagID[1] == 'C') || \
                              (TagID[2] == 'I') || (TagID[3] == 'D'))
#define EEPROM_NO_FORCE_READ   FALSE
#define EEPROM_FORCE_READ      TRUE

#pragma pack(2)
// CDS-boards with attached processor daughter cards
typedef struct CC_SYSTEM_ID {
  // Offset in hex
  UINT8   TagID[4]; // 00-03: literal CCID
  UINT8   Major; // 04 : major revision
  UINT8   Minor; // 05 : minor revision
  UINT8   SerialNo[10]; // 06-0F: serial number
  UINT8   Errata[2]; // 10-11: errata label
  UINT8   Date[6]; // 12-17: build date/time
  UINT8   Res0[40]; // 18-3F: reserved
  UINT8   MacSize; // 40 : number of valid MAC addresses
  UINT8   MacFlag; // 41 : MAC table flags
  UINT8   Mac[8][6]; // 42-71: MAC addresses, array of 6-byte
  UINT32  Crc32; // 72-75: crc-32 checksum
  UINT8   Res1[138]; // 76-FF: reserved
} CC_SYSTEM_ID;
#pragma pack()

#pragma pack(4)
// MPC8xxx-based development systems
// All QorIQ development systems (P1xxx, P2xxx, P4xxx, etc.)
typedef struct NX_SYSTEM_ID {
  // Offset in hex
  UINT8   TagID[4]; // 00-03: literal NXID
  UINT8   SerialNo[12]; // 04-0F: serial number
  UINT8   Errata[5]; // 10-14: errata label
  UINT8   Date[6]; // 15-1A: build date/time
  UINT8   Res0; // 1B : reserved
  UINT32  Version; // 1C-1F: NXID structure version
  UINT8   TempCal[8]; // 20-27: CPU temp. calibration factors
  UINT8   TempCalSys[2]; // 28-29: Board temp. calibration factors
  UINT8   TempCalFlags; // 2A : Temp cal enable/qty/etc.
  UINT8   Res1[21]; // 2B-3F: reserved
  UINT8   MacSize; // 40 : number of valid MAC addresses
  UINT8   MacFlag; // 41 : MAC table flags
  UINT8   Mac[31][6]; // 42-FB: MAC addresses, array of 6-byte
  UINT32  Crc32; // FC-FF: crc-32 checksum
} NX_SYSTEM_ID;
#pragma pack()

typedef union SYSTEM_ID {
  CC_SYSTEM_ID  CCSystemID;
  NX_SYSTEM_ID  NXSystemID;
} SYSTEM_ID;

/**
 * Bit masks for first byte of a MAC address
 */
# define MAC_MULTICAST_ADDRESS_MASK 0x1
# define MAC_PRIVATE_ADDRESS_MASK   0x2

#endif /* SYS_EEPROM_H_ */

