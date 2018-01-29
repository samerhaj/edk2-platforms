/** @CfiCommand.h

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __CFI_COMMAND_H__
#define __CFI_COMMAND_H__

// CFI Data "QRY"
#define CFI_QRY_Q                               0x51
#define CFI_QRY_R                               0x52
#define CFI_QRY_Y                               0x59
#define CFI_QRY                                 0x515259

#define ENTER_CFI_QUERY_MODE_ADDR               0x0055
#define ENTER_CFI_QUERY_MODE_CMD                0x0098

#define CFI_QUERY_UNIQUE_QRY_STRING             0x10

// Offsets for CFI queries
#define CFI_QUERY_TYP_TIMEOUT_WORD_WRITE        0x1F
#define CFI_QUERY_TYP_TIMEOUT_MAX_BUFFER_WRITE  0x20
#define CFI_QUERY_TYP_TIMEOUT_BLOCK_ERASE       0x21
#define CFI_QUERY_TYP_TIMEOUT_CHIP_ERASE        0x22
#define CFI_QUERY_MAX_TIMEOUT_WORD_WRITE        0x23
#define CFI_QUERY_MAX_TIMEOUT_MAX_BUFFER_WRITE  0x24
#define CFI_QUERY_MAX_TIMEOUT_BLOCK_ERASE       0x25
#define CFI_QUERY_MAX_TIMEOUT_CHIP_ERASE        0x26
#define CFI_QUERY_DEVICE_SIZE                   0x27
#define CFI_QUERY_MAX_NUM_BYTES_WRITE           0x2A
#define CFI_QUERY_BLOCK_SIZE                    0x2F

// Unlock Address
#define CMD_UNLOCK_1_ADDR                       0x555
#define CMD_UNLOCK_2_ADDR                       0x2AA

// RESET Command
#define CMD_RESET_FIRST                         0xAA
#define CMD_RESET_SECOND                        0x55
#define CMD_RESET                               0xF0

// READ Command

// Manufacturer ID
#define CMD_READ_M_ID_FIRST                     0xAA
#define CMD_READ_M_ID_SECOND                    0x55
#define CMD_READ_M_ID_THIRD                     0x90
#define CMD_READ_M_ID_FOURTH                    0x01

// Device ID
#define CMD_READ_D_ID_FIRST                     0xAA
#define CMD_READ_D_ID_SECOND                    0x55
#define CMD_READ_D_ID_THIRD                     0x90
#define CMD_READ_D_ID_FOURTH                    0x7E
#define CMD_READ_D_ID_FIFTH                     0x13
#define CMD_READ_D_ID_SIXTH                     0x00

// WRITE Command

// PROGRAM Command
#define CMD_PROGRAM_FIRST                       0xAA
#define CMD_PROGRAM_SECOND                      0x55
#define CMD_PROGRAM_THIRD                       0xA0

// Write Buffer Command
#define CMD_WRITE_TO_BUFFER_FIRST               0xAA
#define CMD_WRITE_TO_BUFFER_SECOND              0x55
#define CMD_WRITE_TO_BUFFER_THIRD               0x25
#define CMD_WRITE_TO_BUFFER_CONFIRM             0x29

// ERASE Command

// UNLOCK COMMANDS FOR ERASE
#define CMD_ERASE_FIRST                         0xAA
#define CMD_ERASE_SECOND                        0x55
#define CMD_ERASE_THIRD                         0x80
#define CMD_ERASE_FOURTH                        0xAA
#define CMD_ERASE_FIFTH                         0x55

// Chip Erase Command
#define CMD_CHIP_ERASE_SIXTH                    0x10

// Sector Erase Command
#define CMD_SECTOR_ERASE_SIXTH                  0x30

// SUSPEND Command
#define CMD_PROGRAM_OR_ERASE_SUSPEND            0xB0
#define CMD_PROGRAM_OR_ERASE_RESUME             0x30

#endif // __CFI_COMMAND_H__
