#  @SocHelper.S
#
#  Copyright 2018 NXP
#
#  This program and the accompanying materials
#  are licensed and made available under the terms and conditions of the BSD License
#  which accompanies this distribution.  The full text of the license may be found at
#  http://opensource.org/licenses/bsd-license.php
#
#  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
#  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
#
#
#include <AutoGen.h>

.text
.align 2

GCC_ASM_EXPORT(CleanDcacheRange)

//
// VOID CleanDcacheRange(UINT64 StartAddr, UINT64 EndAddr)
//
// Cleans and invalidates a range of addresses from D-cache
//
// X0: start address
// X1: end address
//
ASM_PFX(CleanDcacheRange):
  mrs  x3, ctr_el0
  lsr  x3, x3, #16
  and  x3, x3, #0xf
  mov  x2, #4
  lsl  x2, x2, x3    /* cache line size */

  /* x2 <- minimal cache line size in cache system */
  sub  x3, x2, #1
  bic  x0, x0, x3
1:  dc  civac, x0  /* clean & invalidate data or unified cache */
  add  x0, x0, x2
  cmp  x0, x1
  b.lo  1b
  dsb  sy
  ret


ASM_FUNCTION_REMOVE_IF_UNREFERENCED
