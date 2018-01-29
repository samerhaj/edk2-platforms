/** @CfiNorFlashLib.c

 Copyright (c) 2015, Freescale Semiconductor, Inc. All rights reserved.
 Copyright 2017 NXP

 This program and the accompanying materials
 are licensed and made available under the terms and conditions of the BSD License
 which accompanies this distribution.  The full text of the license may be found at
 http://opensource.org/licenses/bsd-license.php

 THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

 **/

#include <PiDxe.h>
#include <Library/ArmLib.h>
#include <Library/BaseLib.h>

#include "CfiCommand.h"
#include "CfiNorFlashLib.h"

FLASH_DATA
IfcNorFlashData (
  IN  OUT  FLASH_DATA  Value
  )
{
  if (FixedPcdGetBool (PcdIfcBigEndian)) {
    return SwapBytes16 (Value);
  } else {
    return Value;
  }
}

/**
  Write Val at given address.

  @param  Val     Data to be written.
  @param  Addr    Address where data is to be written.

**/
VOID
FlashWriteData (
  IN  FLASH_DATA  Val,
  IN  UINTN       Addr
  )
{
  *(volatile FLASH_DATA *)(Addr) = (Val);
}

/**
  Check endianness of IFC Controller and depending on swap
  the data and write on given address.

  @param  Val     Data to be written.
  @param  Addr    Address where data is to be written.

**/
VOID
FlashWrite (
  IN  FLASH_DATA  Val,
  IN  UINTN       Addr
   )
{
  FLASH_DATA      ShiftVal;

  ShiftVal = IfcNorFlashData (Val);

  *(volatile FLASH_DATA *)(Addr) = (ShiftVal);
}

/**
  Read data from given address.

  @param  Addr  Address from where data is to be read.

  @return       Read Data
**/
FLASH_DATA
FlashReadData (
  IN  UINTN     Addr
  )
{
  FLASH_DATA Val;

  Val = *(volatile FLASH_DATA *)(Addr);

  return (Val);
}

/**
  Read data from given address and depending on endianness of IFC Controller
  swap the read data.

  @param  Addr  Address from where data is to be read.

  @return       Read Data
**/
FLASH_DATA
FlashRead (
  IN  UINTN     Addr
  )
{
  FLASH_DATA Val;
  FLASH_DATA ShiftVal;

  Val = *(volatile FLASH_DATA *)(Addr);
  ShiftVal = IfcNorFlashData (Val);

  return (ShiftVal);
}

STATIC
VOID
NorFlashReadCfiData (
  IN  UINTN  DeviceBaseAddress,
  IN  UINTN  CfiOffset,
  IN  UINT32 NumberOfShorts,
  OUT VOID   *Data
  )
{
  UINT32     Count;
  FLASH_DATA *TmpData = (FLASH_DATA *)Data;

  for (Count = 0; Count < NumberOfShorts; Count++, TmpData++) {
    *TmpData = FLASH_READ ((UINTN)((FLASH_DATA*)DeviceBaseAddress + CfiOffset));
    CfiOffset++;
  }
}

/*
  Currently we support only CFI flash devices; Bail-out otherwise
*/
EFI_STATUS
CfiNorFlashFlashGetAttributes (
  OUT NorFlashDescription  *NorFlashDevices,
  IN  UINT32               Index
  )
{
  UINT32                   Count;
  FLASH_DATA               QryData[3];
  FLASH_DATA               BlockSize[2];
  UINTN                    DeviceBaseAddress;
  FLASH_DATA               MaxNumBytes[2];
  FLASH_DATA               Size;
  FLASH_DATA               HighByteMask;  // Masks High byte in a UIN16 word
  FLASH_DATA               HighByteShift; // Bitshifts needed to make a byte High Byte in a UIN16 word
  FLASH_DATA               Temp1;
  FLASH_DATA               Temp2;

  HighByteMask  = 0xFF;
  HighByteShift = 8;

  for (Count = 0; Count < Index; Count++) {

    NorFlashDevices[Count].DeviceBaseAddress = DeviceBaseAddress = PcdGet64 (PcdFlashDeviceBase64);

    // Reset flash first
    NorFlashPlatformReset (DeviceBaseAddress);

    // Enter the CFI Query Mode
    SEND_NOR_COMMAND (DeviceBaseAddress, ENTER_CFI_QUERY_MODE_ADDR,
            ENTER_CFI_QUERY_MODE_CMD);

    ArmDataSynchronizationBarrier ();

    // Query the unique QRY
    NorFlashReadCfiData (DeviceBaseAddress,
            CFI_QUERY_UNIQUE_QRY_STRING,
            3,
            &QryData);
    if (QryData[0] != (FLASH_DATA)CFI_QRY_Q || QryData[1] !=
            (FLASH_DATA)CFI_QRY_R || QryData[2] != (FLASH_DATA)CFI_QRY_Y ) {
      DEBUG ((DEBUG_ERROR, "Not a CFI flash (QRY not recvd): "
                   "Got = 0x%04x, 0x%04x, 0x%04x\n",
                   QryData[0], QryData[1], QryData[2]));
        return EFI_DEVICE_ERROR;
     }

    NorFlashReadCfiData (DeviceBaseAddress, CFI_QUERY_DEVICE_SIZE,
                            1, &Size);
    // Refer CFI Specification
    NorFlashDevices[Count].Size = 1 << Size;

    NorFlashReadCfiData (DeviceBaseAddress, CFI_QUERY_BLOCK_SIZE,
                            2, &BlockSize);
    // Refer CFI Specification
    NorFlashDevices[Count].BlockSize = 256 * ((FLASH_DATA) ((BlockSize[1] <<
                    HighByteShift) | (BlockSize[0] & HighByteMask)));

    NorFlashReadCfiData (DeviceBaseAddress,
            CFI_QUERY_MAX_NUM_BYTES_WRITE, 2, &MaxNumBytes);
    // Refer CFI Specification
    /* from CFI query we get the Max. number of BYTE in multi-byte write = 2^N.
       But our Flash Library is able to read/write in WORD size (2 bytes) which
       is why we need to CONVERT MAX BYTES TO MAX WORDS by diving it by
       width of word size */
    NorFlashDevices[Count].MultiByteWordCount =\
    (1 << ((FLASH_DATA)((MaxNumBytes[1] << HighByteShift) |
                        (MaxNumBytes[0] & HighByteMask))))/sizeof(FLASH_DATA);

    NorFlashReadCfiData (DeviceBaseAddress,
            CFI_QUERY_TYP_TIMEOUT_WORD_WRITE, 1, &Temp1);
    NorFlashReadCfiData (DeviceBaseAddress,
            CFI_QUERY_MAX_TIMEOUT_WORD_WRITE, 1, &Temp2);
    NorFlashDevices[Count].WordWriteTimeOut = (1U << Temp1) * (1U << Temp2);

    NorFlashReadCfiData (DeviceBaseAddress,
            CFI_QUERY_TYP_TIMEOUT_MAX_BUFFER_WRITE, 1, &Temp1);
    NorFlashReadCfiData (DeviceBaseAddress,
            CFI_QUERY_MAX_TIMEOUT_MAX_BUFFER_WRITE, 1, &Temp2);
    NorFlashDevices[Count].BufferWriteTimeOut = (1U << Temp1) * (1U << Temp2);

    NorFlashReadCfiData (DeviceBaseAddress,
            CFI_QUERY_TYP_TIMEOUT_BLOCK_ERASE, 1, &Temp1);
    NorFlashReadCfiData (DeviceBaseAddress,
            CFI_QUERY_MAX_TIMEOUT_BLOCK_ERASE, 1, &Temp2);
    NorFlashDevices[Count].BlockEraseTimeOut =
            (1U << Temp1) * (1U << Temp2) * 1000;

    NorFlashReadCfiData (DeviceBaseAddress,
            CFI_QUERY_TYP_TIMEOUT_CHIP_ERASE, 1, &Temp1);
    NorFlashReadCfiData (DeviceBaseAddress,
            CFI_QUERY_MAX_TIMEOUT_CHIP_ERASE, 1, &Temp2);
    NorFlashDevices[Count].ChipEraseTimeOut =
            (1U << Temp1) * (1U << Temp2) * 1000;

    // Put device back into Read Array mode (via Reset)
    NorFlashPlatformReset (DeviceBaseAddress);
  }

  return EFI_SUCCESS;
}
