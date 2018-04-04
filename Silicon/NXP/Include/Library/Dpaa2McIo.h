/** Dpaa2McIo.h
   DPAA2 Management Complex (MC) Environment-specific I/O

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef _DPAA2_MC_IO_H
#define _DPAA2_MC_IO_H

#include <Library/Dpaa2McCmd.h>
#include <Library/UefiLib.h>
#include <Uefi.h>

/*
 * MC command portal wrapper object
 */
typedef struct fsl_mc_io {
  /**
   * Pointer to the MC portal to be used
   * to send commands to the MC
   */
  volatile DPAA2_MC_COMMAND *McPortal;

  /**
   * TPL-based lock to serialize concurrent
   * commands sent on the same MC portal
   */
  EFI_LOCK McSendCmdLock;
} DPAA2_MC_IO;

INT32
mc_send_command (
  DPAA2_MC_IO *McIo,
  DPAA2_MC_COMMAND *Cmd
  );

#endif /* _DPAA2_MC_IO_H */
