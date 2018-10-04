/**************************************************************************

Copyright (c) 2001-2010, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 3. Neither the name of the Intel Corporation nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/

#ifndef _STARTSTOP_H_
#define _STARTSTOP_H_

#include "e1000.h"

#define EFI_DRIVER_STOP_PROTOCOL_GUID \
{ 0x34d59603, 0x1428, 0x4429, { 0xa4, 0x14, 0xe6, 0xb3, \
0xb5, 0xfd, 0x7d, 0xc1 } }

typedef struct _EFI_DRIVER_STOP_PROTOCOL  EFI_DRIVER_STOP_PROTOCOL;

typedef
EFI_STATUS
(EFIAPI *EFI_DRIVER_STOP_PROTOCOL_STOP_DRIVER) (
  IN EFI_DRIVER_STOP_PROTOCOL *This
  );

typedef
EFI_STATUS
(EFIAPI *EFI_DRIVER_STOP_PROTOCOL_START_DRIVER) (
  IN EFI_DRIVER_STOP_PROTOCOL *This
  );

typedef struct _EFI_DRIVER_STOP_PROTOCOL {
  EFI_DRIVER_STOP_PROTOCOL_STOP_DRIVER StopDriver;
  EFI_DRIVER_STOP_PROTOCOL_START_DRIVER StartDriver;
} EFI_DRIVER_STOP_PROTOCOL;

EFI_STATUS
StopDriver (
  IN EFI_DRIVER_STOP_PROTOCOL *This
  );

EFI_STATUS
StartDriver (
  IN EFI_DRIVER_STOP_PROTOCOL *This
  );


#endif /* _STARTSTOP_H_ */

