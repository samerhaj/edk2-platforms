/** @Utils.h
  Header defining the General Purpose Utilities

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __GENERIC_FUNC_H__
#define __GENERIC_FUNC_H__


//
// Converts a 32-bit big endian value to the corresponding value in CPU
// byte-order, assuming the CPU byte order is little endian.
//
#define BE32_TO_CPU(_x) SwapBytes32(_x)

/*
 * Divide positive or negative dividend by positive divisor and round
 * to closest UINTNeger. Result is undefined for negative divisors and
 * for negative dividends if the divisor variable type is unsigned.
 */
#define DIV_ROUND_CLOSEST(x, divisor)(			\
{							\
	typeof(x) __x = x;				\
	typeof(divisor) __d = divisor;			\
	(((typeof(x))-1) > 0 ||				\
	 ((typeof(divisor))-1) > 0 || (__x) > 0) ?	\
		(((__x) + ((__d) / 2)) / (__d)) :	\
		(((__x) - ((__d) / 2)) / (__d));	\
}							\
)

/*
 * HammingWeight32: returns the hamming weight (i.e. the number
 * of bits set) of a 32-bit word
 */
static inline UINTN HammingWeight32(UINTN w)
{
  UINTN Res = (w & 0x55555555) + ((w >> 1) & 0x55555555);
  Res = (Res & 0x33333333) + ((Res >> 2) & 0x33333333);
  Res = (Res & 0x0F0F0F0F) + ((Res >> 4) & 0x0F0F0F0F);
  Res = (Res & 0x00FF00FF) + ((Res >> 8) & 0x00FF00FF);
  return (Res & 0x0000FFFF) + ((Res >> 16) & 0x0000FFFF);
}

static inline UINTN CpuMaskNext(UINTN Cpu, UINTN Mask)
{
  for (Cpu++; !((1 << Cpu) & Mask); Cpu++)
    ;

  return Cpu;
}

#define ForEachCpu(iter, cpu, num_cpus, mask) \
    for (iter = 0, cpu = CpuMaskNext(-1, mask); \
      iter < num_cpus; \
      iter++, cpu = CpuMaskNext(cpu, mask)) \

/**
  Find last (most-significant) bit set

  @param   X		the word to search

  Note Fls(0) = 0, Fls(1) = 1, Fls(0x80000000) = 32.

**/
static inline INT32 GenericFls(INT32 X)
{
	INT32 R = 32;

	if (!X)
		return 0;
	if (!(X & 0xffff0000u)) {
		X <<= 16;
		R -= 16;
	}
	if (!(X & 0xff000000u)) {
		X <<= 8;
		R -= 8;
	}
	if (!(X & 0xf0000000u)) {
		X <<= 4;
		R -= 4;
	}
	if (!(X & 0xc0000000u)) {
		X <<= 2;
		R -= 2;
	}
	if (!(X & 0x80000000u)) {
		X <<= 1;
		R -= 1;
	}
	return R;
}

VOID
PrintSize (
  IN  UINT64 Size,
  IN  CONST INT8 *S
  );

CHAR8 *StringToMHz (
  CHAR8   *Buf,
  UINT32  Size,
  UINT64  Hz
  );

#endif
