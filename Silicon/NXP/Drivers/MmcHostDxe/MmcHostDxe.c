/** @file

  This file implement the MMC Host Protocol for the NXP SDHC controller.

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/
#include <libfdt.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DevicePathLib.h>
#include <Library/MmcLib.h>
#include <Library/SocClockLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Protocol/MmcHost.h>

STATIC SD_CMD Cmd;

EFI_GUID mMmcDevicePathGuid = EFI_CALLER_ID_GUID;

/**
  Function to call library function to detect card presence

  @param  This Pointer to MMC host protocol structure

**/
BOOLEAN
MmcIsCardPresent (
  IN EFI_MMC_HOST_PROTOCOL     *This
  )
{
  return DetectCardPresence ();
}

/**
  Function to call library function to verify card is read only

  @param  This Pointer to MMC host protocol structure

**/
BOOLEAN
MmcIsReadOnly (
  IN EFI_MMC_HOST_PROTOCOL     *This
  )
{
  return IsCardReadOnly ();
}

/**
  Function to create command reseponse depeping upon
  input command parameter

  @param  MmcCmd MMC Command

**/
STATIC
UINT32
CreateResponseType (
  IN  UINT32                   MmcCmd
  )
{
  UINT32                       RespType;

  RespType = 0;

  if (MmcCmd & MMC_CMD_WAIT_RESPONSE) {
    RespType = MMC_RSP_PRESENT;

    if (MmcCmd & MMC_CMD_LONG_RESPONSE) {
      RespType |= (MMC_RSP_136|MMC_RSP_CRC);
    }
    else if (!(MmcCmd & MMC_CMD_NO_CRC_RESPONSE)) {
      RespType |= (MMC_RSP_CRC|MMC_RSP_OPCODE);
    }
  }

  if (MMC_GET_INDX(MmcCmd) == MMC_INDX(12)) {
    RespType |= MMC_RSP_BUSY;
  }

  return RespType;
}

/**
  Function to send MMC command

  @param  This     Pointer to MMC host protocol structure
  @param  MmcCmd   MMC Command
  @param  Argument Argument if any

**/
EFI_STATUS
MmcSendCommand (
  IN  EFI_MMC_HOST_PROTOCOL    *This,
  IN  MMC_CMD                  MmcCmd,
  IN  UINT32                   Argument
  )
{
  EFI_STATUS                   Status;
  UINT8                        LastCmd;

  LastCmd = Cmd.CmdIdx;

  Cmd.CmdIdx = MMC_GET_INDX (MmcCmd);
  Cmd.CmdArg = Argument;
  Cmd.RespType = CreateResponseType (MmcCmd);

  //Saved data in Cmd struct for commands that need a read/write.
  //This is done because which setting Xfertype register we need
  //information of block number and blocksize.
  if ((Cmd.CmdIdx == MMC_INDX (6)) || (Cmd.CmdIdx == MMC_INDX (51)) ||
       (Cmd.CmdIdx == MMC_INDX (17)) || (Cmd.CmdIdx == MMC_INDX (18)) ||
       (Cmd.CmdIdx == MMC_INDX (24)) || (Cmd.CmdIdx == MMC_INDX (25))) {

      if ((Cmd.CmdIdx == MMC_INDX (6)) && (LastCmd == MMC_INDX (55))) {
        Status = SendCmd (&Cmd, NULL);
      }
      else {
        Status = EFI_SUCCESS;
      }
   } else {
      Status = SendCmd (&Cmd, NULL);
   }

  return Status;
}

/**
  Function to receive MMC command response

  @param  This   Pointer to MMC host protocol structure
  @param  Type   MMC Command response type
  @param  Buffer Pointer to response Buffer

**/
EFI_STATUS
MmcReceiveResponse (
  IN  EFI_MMC_HOST_PROTOCOL    *This,
  IN  MMC_RESPONSE_TYPE        Type,
  IN  UINT32*                  Buffer
  )
{
  EFI_STATUS                   Status;

  if (Buffer == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  if (Type == MMC_RESPONSE_TYPE_R2) {
    Type |= MMC_RSP_136;
  } else if (Type == MMC_RESPONSE_TYPE_R1b) {
    Type |= MMC_RSP_BUSY;
  }

  DEBUG_MSG ("MMC_RESPONSE_TYPE 0x%x for cmd %d \n", Type, Cmd.CmdIdx);

  // if Last sent command is one among 6, 51, 17, 18, 24 and 25, then
  // set data to 1 else 0
  if ((Cmd.CmdIdx == MMC_INDX (6)) || (Cmd.CmdIdx == MMC_INDX (51)) ||
      (Cmd.CmdIdx == MMC_INDX (17)) || (Cmd.CmdIdx == MMC_INDX (18)) ||
      (Cmd.CmdIdx == MMC_INDX (24)) || (Cmd.CmdIdx == MMC_INDX (25))) {
    Status = RcvResp (Type, Buffer, 1);
  } else {
    Status = RcvResp (Type, Buffer, 0);
  }

  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to receive response for %d \n", Cmd.CmdIdx));
    return Status;
  }

  return Status;
}

/**
  Function to dump MMC data

  @param  Buffer  Pointer to MMC data
  @param  Length  Length of MMC data

**/
VOID
DumpData (
  IN  UINT32*                  Buffer,
  IN  UINTN                    Length
  )
{
#ifdef MMC_DEBUG_READ
  UINT32                       Temp;

  DEBUG ((DEBUG_ERROR, "DATA IS (%d) :", Length));
  for (Temp = 1; Temp <= Length; Temp++) {
      DEBUG ((DEBUG_ERROR, "0x%x ", Buffer[Temp-1]));
      if ((Temp != 0) && !(Temp % 8))
          DEBUG ((DEBUG_ERROR, "--- %d\n", Temp-1));
  }
  DEBUG ((DEBUG_ERROR, "\n"));
#endif
}

/**
  Function to read MMC Data Block

  @param  This    Pointer to MMC host protocol structure
  @param  Lba
  @param  Length  Length of MMC data
  @param  Buffer  Pointer to MMC data

**/
EFI_STATUS
MmcReadBlockData (
  IN  EFI_MMC_HOST_PROTOCOL    *This,
  IN  EFI_LBA                  Lba,
  IN  UINTN                    Length,
  IN  UINT32*                  Buffer
  )
{
  EFI_STATUS                   RetVal;
  EFI_TPL                      Tpl;
  UINT8                        Temp;

  // Raise the TPL at the highest level to disable Interrupts.
  Tpl = gBS->RaiseTPL (TPL_NOTIFY);

  //send Cmd structure here, library will send this command.
  RetVal = ReadBlock (Lba, Length, Buffer, Cmd);

  if (Cmd.CmdIdx == MMC_INDX (6)) {
    for (Temp = 0; Temp < Length/8; Temp++) {
    Buffer[Temp] = SwapBytes32 (Buffer[Temp]);
    }
  }

  if (RetVal == EFI_SUCCESS) {
    DumpData (Buffer, Length);
  }

  // Restore Tpl
  gBS->RestoreTPL (Tpl);

  return RetVal;
}

/**
  Function to write MMC Data Block

  @param  This    Pointer to MMC host protocol structure
  @param  Lba
  @param  Length  Length of MMC data block to be written
  @param  Buffer  Pointer to MMC data

**/
EFI_STATUS
MmcWriteBlockData (
  IN  EFI_MMC_HOST_PROTOCOL    *This,
  IN  EFI_LBA                  Lba,
  IN  UINTN                    Length,
  IN  UINT32*                  Buffer
  )
{
  EFI_STATUS                   RetVal;
  EFI_TPL                      Tpl;

  // Raise the TPL at the highest level to disable Interrupts.
  Tpl = gBS->RaiseTPL (TPL_NOTIFY);

  //send Cmd structure here, library will send this command.
  RetVal = WriteBlock (Lba, Length, Buffer, Cmd);

  // Restore Tpl
  gBS->RestoreTPL (Tpl);

  return RetVal;
}

/**
  Function to notify for different MMC states

  @param  This  Pointer to MMC host protocol structure
  @param  State MMC State

**/
EFI_STATUS
MmcNotifyState (
  IN  EFI_MMC_HOST_PROTOCOL    *This,
  IN  MMC_STATE                State
  )
{
  EFI_STATUS                   Status;

  switch (State) {
  case MmcInvalidState:
    ASSERT (0);
    break;
  case MmcHwInitializationState:
    DEBUG ((DEBUG_ERROR, "MmcNotifyState(MmcHwInitializationState)\n"));

    Status = MmcInitialize ();
    if (Status != EFI_SUCCESS) {
      DEBUG ((DEBUG_ERROR,"Failed to init MMC\n"));
      return Status;
    }
    break;
  case MmcIdleState:
  case MmcReadyState:
  case MmcIdentificationState:
  case MmcStandByState:
  case MmcTransferState:
  case MmcSendingDataState:
  case MmcReceiveDataState:
  case MmcProgrammingState:
  case MmcDisconnectState:
    break;
  default:
    ASSERT (0);
  }
  return EFI_SUCCESS;
}

/**
  Function to create device PATH for MMC node

  @param  This       Pointer to MMC host protocol structure
  @param  DevicePath Pointer to device path protocol structure

**/
EFI_STATUS
MmcBuildDevicePath (
  IN  EFI_MMC_HOST_PROTOCOL    *This,
  IN  EFI_DEVICE_PATH_PROTOCOL **DevicePath
  )
{
  EFI_DEVICE_PATH_PROTOCOL     *NewDevicePathNode;

  NewDevicePathNode = CreateDeviceNode (HARDWARE_DEVICE_PATH,
          HW_VENDOR_DP, sizeof (VENDOR_DEVICE_PATH));
  CopyGuid (&((VENDOR_DEVICE_PATH*)NewDevicePathNode)->Guid, &mMmcDevicePathGuid);

  *DevicePath = NewDevicePathNode;
  return EFI_SUCCESS;
}

/**
  Function to set MMC speed

  @param  This         Pointer to MMC host protocol structure
  @param  BusClockFreq Bus clock frequency
  @param  BusWidth     Bus width
  @param  TimingMode   Timing mode

**/
EFI_STATUS
MmcSetIos (
  IN  EFI_MMC_HOST_PROTOCOL    *This,
  IN  UINT32                   BusClockFreq,
  IN  UINT32                   BusWidth,
  IN  UINT32                   TimingMode
  )
{
  SetIos (BusClockFreq, BusWidth, TimingMode);
  return EFI_SUCCESS;
}

BOOLEAN
MmcIsMultBlk (
  IN  EFI_MMC_HOST_PROTOCOL    *This
  )
{
  return TRUE;
}

EFI_MMC_HOST_PROTOCOL gMmcHost = {
  MMC_HOST_PROTOCOL_REVISION,
  MmcIsCardPresent,
  MmcIsReadOnly,
  MmcBuildDevicePath,
  MmcNotifyState,
  MmcSendCommand,
  MmcReceiveResponse,
  MmcReadBlockData,
  MmcWriteBlockData,
  MmcSetIos,
  MmcIsMultBlk
};

/**
  insert the MMC peripheral clock info into the device tree

  @param[in] Dtb     Dtb Image into which MMC peripheral clock info is to be inserted.

  @retval EFI_DEVICE_ERROR    Fail to add MMC peripheral clock info to Device tree.
  @retval EFI_SUCCES          MMC peripheral clock info inserted into Device tree.
**/
EFI_STATUS
FdtFixupMmc (
  IN VOID *Dtb
  )
{
  UINT64   MmcClk;
  INT32    NodeOffset;
  INT32    FdtStatus;

  NodeOffset = fdt_node_offset_by_compatible (Dtb, -1, "fsl,esdhc");
  // if node not found, silently return
  if (NodeOffset < 0) {
    return EFI_SUCCESS;
  }

  MmcClk = SocGetClock (IP_ESDHC, 0);
  if (!MmcClk) {
    DEBUG ((DEBUG_ERROR, "Invalid Mmc clock\n"));
    return EFI_SUCCESS;
  }

  FdtStatus = fdt_setprop_u32 (Dtb, NodeOffset, "clock-frequency", MmcClk);
  if (FdtStatus) {
    DEBUG ((DEBUG_ERROR, "fdt_setprop/esdhc: Could not add property, %a!!\n", fdt_strerror (FdtStatus)));
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}

/**
  Function to install MMC Host Protocol gEfiMmcHostProtocolGuid
**/
EFI_STATUS
MmcHostDxeEntryPoint (
  IN  EFI_HANDLE               ImageHandle,
  IN  EFI_SYSTEM_TABLE         *SystemTable
  )
{
  EFI_STATUS                   Status;
  EFI_HANDLE                   Handle;
  VOID                         *Dtb;

  Handle = NULL;
  Dtb = NULL;

  Status = gBS->InstallMultipleProtocolInterfaces (
                  &Handle,
                  &gEfiMmcHostProtocolGuid, &gMmcHost,
                  NULL
                  );

  if (EFI_ERROR(Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to install gEfiMmcHostProtocolGuid\n"));
  }

  Status = EfiGetSystemConfigurationTable (&gFdtTableGuid, &Dtb);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Did not find the Dtb Blob.\n"));
    return EFI_NOT_FOUND;
  }

  Status = FdtFixupMmc (Dtb);

  return Status;
}
