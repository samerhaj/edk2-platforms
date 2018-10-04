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

#ifndef _E1000_OSDEP_H_
#define _E1000_OSDEP_H_

#ifndef EFI_SPECIFICATION_VERSION
#define EFI_SPECIFICATION_VERSION 0x00020000
#endif

#ifndef TIANO_RELEASE_VERSION
#define TIANO_RELEASE_VERSION     0x00080005
#endif

struct e1000_hw;

UINT32
e1000_InDword (
  struct e1000_hw *hw,
  UINT32          Port
  );

VOID
e1000_OutDword (
  IN struct e1000_hw *hw,
  IN UINT32          Port,
  IN UINT32          Data
  );

UINT32
e1000_FlashRead (
  IN struct e1000_hw *hw,
  IN UINT32          Port
  );

UINT16
e1000_FlashRead16 (
  IN struct e1000_hw *hw,
  IN UINT32          Port
  );

VOID
e1000_FlashWrite (
  IN struct e1000_hw *hw,
  IN UINT32          Port,
  IN UINT32          Data
  );

VOID
e1000_FlashWrite16 (
  IN struct e1000_hw *hw,
  IN UINT32          Port,
  IN UINT16          Data
  );

VOID
e1000_PciFlush (
  struct e1000_hw *hw
  );

VOID
uSecDelay (
  struct e1000_hw *hw,
  UINTN  usecs
  );

VOID
e1000_write_reg_io(
  struct e1000_hw *hw,
  UINT32 offset,
  UINT32 value
  );


#define usec_delay(x)     uSecDelay(hw, x)
#define msec_delay(x)     uSecDelay(hw, x*1000)
#define msec_delay_irq(x) uSecDelay(hw, x*1000)
#define memset(Buffer, Value, BufferLength) SetMem (Buffer, BufferLength, Value)

#define CMD_MEM_WRT_INVALIDATE EFI_PCI_COMMAND_MEMORY_WRITE_AND_INVALIDATE

typedef BOOLEAN BOOLEANean_t;


#if (0)
#define DEBUGFUNC(F)
#define DEBUGOUT(s) Aprint(s);
#define DEBUGOUT1(s,a) Aprint(s,a);
#define DEBUGOUT2(s,a,b) Aprint(s,a,b);
#define DEBUGOUT3(s,a,b,c) Aprint(s,a,b,c);
#define DEBUGOUT7(s,a,b,c,d,e,f,g) Aprint(s,a,b,c,d,e,f,g);
#else
#define DEBUGFUNC(F)
#define DEBUGOUT(s)
#define DEBUGOUT1(s,a)
#define DEBUGOUT2(s,a,b)
#define DEBUGOUT3(s,a,b,c)
#define DEBUGOUT7(s,a,b,c,d,e,f,g)
#endif


#define E1000_WRITE_REG(a, reg, value) \
        e1000_OutDword(a, (UINT32)(reg), value)

#define E1000_READ_REG(a, reg)  \
        e1000_InDword(a, (UINT32)(reg)) \

#define E1000_WRITE_REG_ARRAY(a, reg, offset, value) \
        e1000_OutDword(a, (UINT32)(reg + ((offset) << 2)), value)

#define E1000_WRITE_REG_ARRAY_BYTE(a, reg, offset, value)  \
        e1000_OutDword(a, (UINT32)(reg + ((offset) << 2)), value) \

#define E1000_WRITE_REG_ARRAY_DWORD(a, reg, offset, value)  \
        e1000_OutDword(a, (UINT32)(reg + ((offset) << 2)), value)  \

#define E1000_READ_REG_ARRAY(a, reg, offset)  \
        e1000_InDword(a, (UINT32)(reg + ((offset) << 2)))

#define E1000_READ_REG_ARRAY_BYTE(a, reg, offset)  \
        e1000_InDword(a, (UINT32)(reg + ((offset) << 2)))

#define E1000_READ_REG_ARRAY_DWORD(a, reg, offset)  \
        e1000_InDword(a, (UINT32)(reg + ((offset) << 2)))

#define E1000_WRITE_FLUSH(a) e1000_PciFlush(a);

#define E1000_WRITE_REG_IO(a, reg, value) \
        e1000_write_reg_io(a, (UINT32) (reg), value)

#define E1000_READ_FLASH_REG(a, reg) \
        e1000_FlashRead(a, (UINT32)(reg))

#define E1000_WRITE_FLASH_REG(a, reg, data) \
        e1000_FlashWrite(a, (UINT32)(reg), data)

#define E1000_READ_FLASH_REG16(a, reg) \
        e1000_FlashRead16(a, (UINT32)(reg))

#define E1000_WRITE_FLASH_REG16(a, reg, data) \
        e1000_FlashWrite16(a, (UINT32)(reg), data)

#define E1000_MUTEX                       UINT8
#define E1000_MUTEX_INIT(mutex)           UNREFERENCED_1PARAMETER(mutex)
#define E1000_MUTEX_DESTROY(mutex)        UNREFERENCED_1PARAMETER(mutex)
#define E1000_MUTEX_LOCK(mutex)           UNREFERENCED_1PARAMETER(mutex)
#define E1000_MUTEX_TRYLOCK(mutex)        UNREFERENCED_1PARAMETER(mutex)
#define E1000_MUTEX_UNLOCK(mutex)         UNREFERENCED_1PARAMETER(mutex)

#endif
