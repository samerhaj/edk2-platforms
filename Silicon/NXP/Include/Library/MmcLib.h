/** @file
  Header Defining The MMC Memory Controller Constants, Function Prototype, Structures Etc

  Copyright 2017 NXP

  This Program And The Accompanying Materials
  Are Licensed And Made Available Under The Terms And Conditions Of The BSD
  License Which Accompanies This Distribution. The Full Text Of The License
  May Be Found At
  Http://Opensource.Org/Licenses/Bsd-License.Php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __MMCLIB_H__
#define __MMCLIB_H__

#include <Uefi.h>
#include <Library/DebugLib.h>

#ifdef MMC_DEBUG
#define DEBUG_MSG(_Fmt,...)   DEBUG ((DEBUG_ERROR, "MMC: " _Fmt, ##__VA_ARGS__));
#else
#define DEBUG_MSG(_Fmt,...)
#endif

/**
 MMC RESPONSE TYPE
**/
#define MMC_RSP_PRESENT  (1 << 0)
#define MMC_RSP_136      (1 << 1)  // 136 Bit Response
#define MMC_RSP_CRC      (1 << 2)  // Expect Valid Crc
#define MMC_RSP_BUSY     (1 << 3)  // Card May Send Busy
#define MMC_RSP_OPCODE   (1 << 4)  // Response Contains Opcode

#define MMC_RSP_NONE     (0)
#define MMC_RSP_R1       (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R1b      (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE| \
                         MMC_RSP_BUSY)
#define MMC_RSP_R2       (MMC_RSP_PRESENT|MMC_RSP_136|MMC_RSP_CRC)
#define MMC_RSP_R3       (MMC_RSP_PRESENT)
#define MMC_RSP_R4       (MMC_RSP_PRESENT)
#define MMC_RSP_R5       (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R6       (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R7       (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)

/**
  Find most significant set

  @param   X  word to search

  @retval most significant set bit number

**/
STATIC
inline
INT32 GenericFls (INT32 X)
{

  INT32 I = 31;

  for (I=31; I >= 0; I--) {
    if(X & (1<<I))
      return (I+1);
  }

  return 0;
}

typedef struct {
  UINT16 CmdIdx;
  UINT32 RespType;
  UINT32 CmdArg;
} MMC_CMD_INFO;

typedef struct {
  UINT32 Flags;
  UINT32 Blocks;
  UINT32 Blocksize;
  VOID   *Addr;
} MMC_DATA;

/**
  Helper Function to initialize MMC

  1. Reset MMC controller
  2. Set host voltage capabilities
  3. Set MMC clock

**/
EFI_STATUS
MmcInitialize (
  IN  VOID    *BaseAddress
  );

/**
  Function to receive command response

  @param  RespType Type of response
  @param  Data     Data of response

  @param  Response Pointer to response buffer

  @retval          Returns the command response status

**/
EFI_STATUS
RcvResp (
  IN  VOID    *BaseAddress,
  IN  UINT32  RespType,
  OUT UINT32* Response,
  IN  UINT8   Data
  );

/**
  Function to detect card presence by checking host controller
  present state register

  @retval  Returns the card presence as TRUE/FALSE

**/
BOOLEAN
DetectCardPresence (
  IN  VOID           *BaseAddress
  );

/**
  Function to check whether card is read only by verifying host controller
  present state register

  @retval  Returns the card read only or not as TRUE/FALSE

**/
BOOLEAN
IsCardReadOnly (
  IN  VOID          *BaseAddress
  );

/**
  Function to prepare state(Wait for bus,Set up host controller
  data,Transfer type)  for command to be send

  @param  Cmd  Command to be used
  @param  Data Data with command

  @retval      Returns the command status

**/
EFI_STATUS
SendCmd (
  IN  VOID          *BaseAddress,
  IN  MMC_CMD_INFO  *Cmd,
  IN  MMC_DATA      *Data
  );

/**
  Function to set MMC clock speed

  @param  BusClockFreq Bus clock frequency to be set Offset to write to
  @param  BusWidth     Bus width
  @param  TimingMode   Timing mode to be set

**/
EFI_STATUS
SetIos (
  IN  VOID    *BaseAddress,
  IN  UINT32  BusClockFreq,
  IN  UINT32  BusWidth,
  IN  UINT32  TimingMode
  );

/**
  Function to Write MMC Block

  @param  Offset Offset to write to
  @param  Length Length of block
  @param  Buffer Pointer to buffer for data to be written
  @param  Cmd    Pointer to command structure

  @retval        Returns the write block command status

**/
EFI_STATUS
WriteBlock (
  IN  VOID         *BaseAddress,
  IN  UINTN        Offset,
  IN  UINTN        Length,
  IN  UINT32*      Buffer,
  IN  MMC_CMD_INFO Cmd
  );

/**
  Function to Read MMC Block

  @param  Offset Offset to read from
  @param  Length Length of block
  @param  Cmd    Pointer to command structure

  @param  Buffer Pointer to buffer for data read

  @retval        Returns the read block command status

**/
EFI_STATUS
ReadBlock (
  IN  VOID         *BaseAddress,
  IN  UINTN        Offset,
  IN  UINTN        Length,
  IN  UINT32*      Buffer,
  IN  MMC_CMD_INFO Cmd
  );

#endif
