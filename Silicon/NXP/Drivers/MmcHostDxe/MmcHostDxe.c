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
#include <Library/MemoryAllocationLib.h>
#include <Library/MmcLib.h>
#include <Library/SocClockLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>

#include "MmcHostDxe.h"

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
  MMC_DEVICE_INSTANCE          *Instance;

  Instance = MMC_DEVICE_INSTANCE_FROM_HOST (This);

  if (Instance->CardType == SD_CARD) {
    return DetectCardPresence ((VOID *)Instance->DeviceBaseAddress);
  }

  return TRUE;
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
  MMC_DEVICE_INSTANCE          *Instance;

  Instance = MMC_DEVICE_INSTANCE_FROM_HOST (This);

  if (Instance->CardType == SD_CARD) {
    return IsCardReadOnly ((VOID *)Instance->DeviceBaseAddress);
  }

  return FALSE;
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
  UINT8                        CmdIdx;
  MMC_DEVICE_INSTANCE          *Instance;

  Status = EFI_SUCCESS;
  Instance = MMC_DEVICE_INSTANCE_FROM_HOST (This);

  LastCmd = Instance->Cmd.CmdIdx;

  Instance->Cmd.CmdIdx = CmdIdx = MMC_GET_INDX (MmcCmd);
  Instance->Cmd.CmdArg = Argument;
  Instance->Cmd.RespType = CreateResponseType (MmcCmd);

  ///
  /// Saved data in Cmd struct for commands that need a read/write.
  /// This is done because which setting Xfertype register we need
  /// information of block number and blocksize.
  ///
  if ((CmdIdx == MMC_INDX (6)) || (CmdIdx == MMC_INDX (51)) ||
       (CmdIdx == MMC_INDX (17)) || (CmdIdx == MMC_INDX (18)) ||
       (CmdIdx == MMC_INDX (8)) ||
       (CmdIdx == MMC_INDX (24)) || (CmdIdx == MMC_INDX (25))) {

    if (Instance->CardType == SD_CARD) {
      if (((CmdIdx == MMC_INDX (6)) && (LastCmd == MMC_INDX (55))) ||
          (CmdIdx == MMC_INDX (8))) {
        Status = SendCmd ((VOID *)Instance->DeviceBaseAddress, &Instance->Cmd, NULL);
      }
    } else if (Instance->CardType == EMMC_CARD) {
      ///
      /// not supporting DDR mode
      ///
      if ((CmdIdx == MMC_INDX (6))) {
        if ((((Argument >> 16) & 0xFF) == EXTCSD_BUS_WIDTH) &&
            ((((Argument >> 8) & 0xFF) == EMMC_BUS_WIDTH_DDR_4BIT) ||
             (((Argument >> 8) & 0xFF) == EMMC_BUS_WIDTH_DDR_8BIT))) {
          return EFI_UNSUPPORTED;
        } else {
          Status = SendCmd ((VOID *)Instance->DeviceBaseAddress, &Instance->Cmd, NULL);
        }
      }
    } else {
      Status = EFI_SUCCESS;
    }
   } else {
      Status = SendCmd ((VOID *)Instance->DeviceBaseAddress, &Instance->Cmd, NULL);
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
  MMC_DEVICE_INSTANCE          *Instance;
  UINT8                        CmdIdx;

  Instance = MMC_DEVICE_INSTANCE_FROM_HOST (This);

  CmdIdx = Instance->Cmd.CmdIdx;

  if (Buffer == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  if (Type == MMC_RESPONSE_TYPE_R2) {
    Type |= MMC_RSP_136;
  } else if (Type == MMC_RESPONSE_TYPE_R1b) {
    Type |= MMC_RSP_BUSY;
  }

  DEBUG_MSG ("MMC_RESPONSE_TYPE 0x%x for cmd %d \n", Type, CmdIdx);

  // if Last sent command is one among 6, 51, 17, 18, 24 and 25, then
  // set data to 1 else 0
  if ((CmdIdx == MMC_INDX (6)) || (CmdIdx == MMC_INDX (51)) ||
      (CmdIdx == MMC_INDX (17)) || (CmdIdx == MMC_INDX (18)) ||
      (CmdIdx == MMC_INDX (24)) || (CmdIdx == MMC_INDX (25))) {
    Status = RcvResp ((VOID *)Instance->DeviceBaseAddress, Type, Buffer, 1);
  } else {
    Status = RcvResp ((VOID *)Instance->DeviceBaseAddress, Type, Buffer, 0);
  }

  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to receive response for %d \n", CmdIdx));
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
  MMC_DEVICE_INSTANCE          *Instance;

  Instance = MMC_DEVICE_INSTANCE_FROM_HOST (This);

  // Raise the TPL at the highest level to disable Interrupts.
  Tpl = gBS->RaiseTPL (TPL_NOTIFY);

  //send Cmd structure here, library will send this command.
  RetVal = ReadBlock ((VOID *)Instance->DeviceBaseAddress, Lba, Length, Buffer, Instance->Cmd);

  if (Instance->Cmd.CmdIdx == MMC_INDX (6)) {
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
  MMC_DEVICE_INSTANCE          *Instance;

  Instance = MMC_DEVICE_INSTANCE_FROM_HOST (This);

  // Raise the TPL at the highest level to disable Interrupts.
  Tpl = gBS->RaiseTPL (TPL_NOTIFY);

  //send Cmd structure here, library will send this command.
  RetVal = WriteBlock ((VOID *)Instance->DeviceBaseAddress, Lba, Length, Buffer, Instance->Cmd);

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
  MMC_DEVICE_INSTANCE          *Instance;

  Instance = MMC_DEVICE_INSTANCE_FROM_HOST (This);

  switch (State) {
  case MmcInvalidState:
    ASSERT (0);
    break;
  case MmcHwInitializationState:
    DEBUG ((DEBUG_ERROR, "MmcNotifyState(MmcHwInitializationState)\n"));

    Status = MmcInitialize ((VOID *)Instance->DeviceBaseAddress);
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
  MMC_DEVICE_INSTANCE          *Instance;
  EFI_DEVICE_PATH_PROTOCOL     *NewDevicePathNode;
  UINT8                        NodeSubType;

  NodeSubType = 0;
  NewDevicePathNode = NULL;

  Instance = MMC_DEVICE_INSTANCE_FROM_HOST (This);
  DEBUG_MSG ("MmcBuildDevicePath : CardType %d \n", Instance->CardType);

  if (Instance->CardType == SD_CARD) {
    NodeSubType = MSG_SD_DP;
  } else if (Instance->CardType == EMMC_CARD) {
    NodeSubType = MSG_EMMC_DP;
  } else {
    return EFI_UNSUPPORTED;
  }

  NewDevicePathNode = CreateDeviceNode (MESSAGING_DEVICE_PATH, NodeSubType, sizeof (EMMC_DEVICE_PATH));
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
  MMC_DEVICE_INSTANCE          *Instance;

  Instance = MMC_DEVICE_INSTANCE_FROM_HOST (This);

  SetIos ((VOID *)Instance->DeviceBaseAddress, BusClockFreq, BusWidth, TimingMode);

  return EFI_SUCCESS;
}

BOOLEAN
MmcIsMultBlk (
  IN  EFI_MMC_HOST_PROTOCOL    *This
  )
{
  return TRUE;
}

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

EFI_STATUS
ConstructHostInstance (
  IN  UINTN                    BaseAddress,
  IN  CARD_TYPE                CardType
  )
{
  EFI_STATUS                   Status;
  EFI_HANDLE                   Handle;
  MMC_DEVICE_INSTANCE          *MmcHostInstance;

  Handle = NULL;

  MmcHostInstance = (MMC_DEVICE_INSTANCE *)AllocatePool (sizeof (MMC_DEVICE_INSTANCE));

  if (MmcHostInstance == NULL) {
    DEBUG ((DEBUG_ERROR, "Failed to allocate memory to MmcHostInstance\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  MmcHostInstance->Signature = MMC_DEVICE_SIGNATURE;
  MmcHostInstance->CardType = CardType;
  MmcHostInstance->DeviceBaseAddress = BaseAddress;
  MmcHostInstance->MmcHostProtocol.Revision = MMC_HOST_PROTOCOL_REVISION;
  MmcHostInstance->MmcHostProtocol.IsCardPresent = MmcIsCardPresent;
  MmcHostInstance->MmcHostProtocol.IsReadOnly = MmcIsReadOnly;
  MmcHostInstance->MmcHostProtocol.BuildDevicePath = MmcBuildDevicePath;
  MmcHostInstance->MmcHostProtocol.NotifyState = MmcNotifyState;
  MmcHostInstance->MmcHostProtocol.SendCommand = MmcSendCommand;
  MmcHostInstance->MmcHostProtocol.ReceiveResponse = MmcReceiveResponse;
  MmcHostInstance->MmcHostProtocol.ReadBlockData = MmcReadBlockData;
  MmcHostInstance->MmcHostProtocol.WriteBlockData = MmcWriteBlockData;
  MmcHostInstance->MmcHostProtocol.SetIos = MmcSetIos;
  MmcHostInstance->MmcHostProtocol.IsMultiBlock = MmcIsMultBlk;

  Status = gBS->InstallMultipleProtocolInterfaces (
             &Handle,
             &gEfiMmcHostProtocolGuid,
             &MmcHostInstance->MmcHostProtocol,
             NULL
             );

  if (EFI_ERROR(Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to install gEfiMmcHostProtocolGuid on %d \n",
      MmcHostInstance->CardType));
  }

  return Status;
}

EFI_STATUS
CreateMmcHostInstance (
  IN  VOID
  )
{
  EFI_STATUS                   Status;

  if ((VOID *)PcdGet64 (PcdSdxcBaseAddr)) {
    Status = ConstructHostInstance (PcdGet64 (PcdSdxcBaseAddr), SD_CARD);
  }

  if ((VOID *)PcdGet64 (PcdEMmcBaseAddr)) {
    Status =  ConstructHostInstance (PcdGet64 (PcdEMmcBaseAddr), EMMC_CARD);
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
  VOID                         *Dtb;

  Dtb = NULL;

  Status = CreateMmcHostInstance ();
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = EfiGetSystemConfigurationTable (&gFdtTableGuid, &Dtb);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Did not find the Dtb Blob.\n"));
    return EFI_SUCCESS;
  }

  Status = FdtFixupMmc (Dtb);

  return Status;
}
