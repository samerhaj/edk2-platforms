/** Dpaa2McIo.c
   DPAA2 Management Complex (MC) Environment-specific I/O for UEFI

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/
#include <Library/ArmLib.h>
#include <Library/Dpaa2McIo.h>
#include <Library/DpaaDebugLib.h>
#include <Library/TimerLib.h>
#include "ManagementComplex.h"

/**
   Writes a command to a Management Complex (MC) portal

   @param[in] Portal: pointer to an MC portal
   @param[in] Cmd: pointer to a filled command
 **/
STATIC
VOID
McWriteCommand (
  volatile DPAA2_MC_COMMAND *Portal,
  DPAA2_MC_COMMAND *Cmd
  )
{
  UINTN I;

  /* copy command parameters into the portal */
  for (I = 0; I < MC_CMD_NUM_OF_PARAMS; I++) {
    MmioWrite64 ((UINTN)&Portal->params[I], Cmd->params[I]);
  }

  /* submit the command by writing the header */
  ArmDataMemoryBarrier ();
  MmioWrite64 ((UINTN)&Portal->header, Cmd->header);
}


/**
   mc_read_response - reads the response for the last MC command from a
   Management Complex (MC) portal

   @param[in] portal pointer to an MC portal
   @param[out] resp pointer to command response buffer

   @retval MC_CMD_STATUS_OK on Success; Error code otherwise.
 **/
STATIC
MC_CMD_STATUS
McReadResponse (
  volatile DPAA2_MC_COMMAND *Portal,
  DPAA2_MC_COMMAND *Resp
  )
{
  UINTN I;
  MC_CMD_STATUS McStatus;

  /* Copy command response header from MC portal: */
  Resp->header = MmioRead64 ((UINTN)&Portal->header);
  McStatus = MC_CMD_HDR_READ_STATUS (Resp->header);
  if (McStatus != MC_CMD_STATUS_OK) {
    return McStatus;
  }

  /* Copy command response data from MC portal: */
  for (I = 0; I < MC_CMD_NUM_OF_PARAMS; I++) {
    Resp->params[I] = MmioRead64 ((UINTN)&Portal->params[I]);
  }

  return McStatus;
}


/**
   Send MC command and wait for response

   @param[in] McIo  Pointer to MC I/O object to be used
   @param[in] Cmd   MC command buffer. On input, it contains the command to send
                    to the MC. On output, it contains the response from the MC
                    if any.

   @retval 0, on success
   @retval error code, on failure
 **/
INT32
mc_send_command (
  DPAA2_MC_IO *McIo,
  DPAA2_MC_COMMAND *Cmd
  )
{
  MC_CMD_STATUS McCmdStatus;
  UINT32 Timeout;
  UINT16 CmdId;
  UINT16 Token;
  EFI_STATUS Status;

  CmdId = MC_CMD_HDR_READ_CMDID (Cmd->header);
  Token = MC_CMD_HDR_READ_TOKEN (Cmd->header);

  /*
   * Acquire lock to block timer interrupts:
   */
  EfiAcquireLock (&McIo->McSendCmdLock);

  if (gDpaaDebugFlags & DPAA_DEBUG_TRACE_MC_COMMANDS) {
    DPAA_DEBUG_MSG ("Sending MC command: 0x%lx (Cmd ID 0x%x, Token 0x%x)\n",
                    Cmd->header, CmdId, Token);
  }

  /*
   * Send the command to the MC:
   */
  McWriteCommand (McIo->McPortal, Cmd);

  /*
   * Wait for the MC to execute the command,
   * by polling for a response form the MC:
   */
  for (Timeout = 12000; Timeout != 0; Timeout --) {
    McCmdStatus = McReadResponse (McIo->McPortal, Cmd);
    if (McCmdStatus != MC_CMD_STATUS_READY) {
      break;
    }

    MicroSecondDelay (500);
  }

  if (Timeout == 0) {
    DPAA_ERROR_MSG ("Timeout waiting for MC response\n");
    Status = EFI_TIMEOUT;
    goto CommonExit;
  }

  if (gDpaaDebugFlags & DPAA_DEBUG_TRACE_MC_COMMANDS) {
    DPAA_DEBUG_MSG ("Received MC response: 0x%lx (MC status: 0x%x)\n",
                    Cmd->header, McCmdStatus);
  }

  if (McCmdStatus != MC_CMD_STATUS_OK) {
    DPAA_ERROR_MSG (
      "MC command failed (portal: 0x%p, obj handle: 0x%x, command: 0x%x, status: 0x%x)\n",
      McIo->McPortal, Token, CmdId, McCmdStatus);
    Status = EFI_DEVICE_ERROR;
    goto CommonExit;
  }

  Status = EFI_SUCCESS;

CommonExit:
  /*
   * Release lock:
   */
  EfiReleaseLock (&McIo->McSendCmdLock);

  if (EFI_ERROR (Status)) {
    DumpMcLogTail (&gManagementComplex, 10);
  }

  return (INT32)Status;
}
