/** Bitops.h
  Header defining the general bitwise operations

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __BITOPS_H__
#define __BITOPS_H__

#include <Library/DebugLib.h>

#define MASK_LOWER_16              0xFFFF0000
#define MASK_UPPER_16              0x0000FFFF
#define MASK_LOWER_8               0xFF000000
#define MASK_UPPER_8               0x000000FF

/*
 * Returns the bit mask for a bit index from 0 to 31
 */
#define BIT(_BitIndex)         (0x1u << (_BitIndex))

/**
 * Upper32Bits - return bits 32-63 of a number
 * @N: the number we're accessing
 *
 * A basic shift-right of a 64- or 32-bit quantity.  Use this to suppress
 * the "right shift count >= width of type" warning when that quantity is
 * 32-bits.
 */
#define Upper32Bits(N) ((UINT32)(((N) >> 16) >> 16))

/**
 * Lower32Bits - return bits 0-31 of a number
 * @N: the number we're accessing
 */
#define Lower32Bits(N) ((UINT32)(N))


/*
 * Stores a value for a given bit field in 32-bit '_Container'
 */

#define SET_BIT_FIELD32(_Container, _BitShift, _BitWidth, _Value) \
  __SET_BIT_FIELD32(_Container,                                   \
      __GEN_BIT_FIELD_MASK32(_BitShift, _BitWidth),               \
      _BitShift,                                                  \
      _Value)

#define __SET_BIT_FIELD32(_Container, _BitMask, _BitShift, _Value)      \
  do {                                                                  \
    (_Container) &= ~(_BitMask);                                        \
    if ((_Value) != 0) {                                                \
      ASSERT(((UINT32)(_Value) << (_BitShift)) <= (_BitMask));          \
      (_Container) |=                                                   \
      ((UINT32)(_Value) << (_BitShift)) & (_BitMask);                   \
    }                                                                   \
  } while (0)

/*
 * Extracts the value for a given bit field in 32-bit _Container
 */

#define GET_BIT_FIELD32(_Container, _BitShift, _BitWidth) \
  __GET_BIT_FIELD32(_Container,                           \
      __GEN_BIT_FIELD_MASK32(_BitShift, _BitWidth),       \
      _BitShift)

#define __GET_BIT_FIELD32(_Container, _BitMask, _BitShift)  \
  (((UINT32)(_Container) & (_BitMask)) >> (_BitShift))

#define __GEN_BIT_FIELD_MASK32(_BitShift, _BitWidth)        \
  ((_BitWidth) < 32 ?                                       \
   (((UINT32)1 << (_BitWidth)) - 1) << (_BitShift) :        \
   ~(UINT32)0)

/*
 *Stores a value for a given bit field in 64-bit '_Container'
 */
#define SET_BIT_FIELD64(_Container, _BitShift, _BitWidth, _Value) \
  __SET_BIT_FIELD64(_Container,                                   \
      __GEN_BIT_FIELD_MASK64(_BitShift, _BitWidth),               \
      _BitShift,                                                  \
      _Value)

#define __SET_BIT_FIELD64(_Container, _BitMask, _BitShift, _Value)  \
  do {                                                              \
    (_Container) &= ~(_BitMask);                                    \
    if ((_Value) != 0) {                                            \
      ASSERT(((UINT64)(_Value) << (_BitShift)) <= (_BitMask));      \
      (_Container) |=                                               \
      ((UINT64)(_Value) << (_BitShift)) & (_BitMask);               \
    }                                                               \
  } while (0)

/*
 * Extracts the value for a given bit field in 64-bit _Container
 */
#define GET_BIT_FIELD64(_Container, _BitShift, _BitWidth) \
  __GET_BIT_FIELD64(_Container,                           \
      __GEN_BIT_FIELD_MASK64(_BitShift, _BitWidth),       \
      _BitShift)

#define __GET_BIT_FIELD64(_Container, _BitMask, _BitShift) \
  (((UINT64)(_Container) & (_BitMask)) >> (_BitShift))

#define __GEN_BIT_FIELD_MASK64(_BitShift, _BitWidth)       \
  ((_BitWidth) < 64 ?                                      \
   (((UINT64)1 << (_BitWidth)) - 1) << (_BitShift) :       \
   ~(UINT64)0)

/**

 Test If the Destination buffer sets (0->1) or clears (1->0) any bit in Source buffer ?

 @param[in]  Source       Source Buffer Pointer
 @param[in]  Destination  Destination Buffer Pointer
 @param[in]  NumBytes     Bytes to Compare
 @param[in]  Set          True : Test Weather Destination buffer sets any bit in Source buffer ?
                          False : Test Weather Destination buffer clears any bit in Source buffer ?

 @retval     TRUE         Destination buffer sets/clear a bit in source buffer.
 @retval     FALSE        Destination buffer doesn't sets/clear bit in source buffer.

**/
STATIC
inline
BOOLEAN
TestBitSetClear (
  IN  VOID    *Source,
  IN  VOID    *Destination,
  IN  UINTN   NumBytes,
  IN  BOOLEAN Set
  )
{
  UINTN Index = 0;
  VOID* Buffer;

  if (Set) {
    Buffer = Destination;
  } else {
    Buffer = Source;
  }

  while (Index < NumBytes) {
    if ((NumBytes - Index) >= 8) {
      if ((*((UINT64*)(Source+Index)) ^ *((UINT64*)(Destination+Index))) & *((UINT64*)(Buffer+Index))) {
        return TRUE;
      }
      Index += 8;
    } else if ((NumBytes - Index) >= 4) {
      if ((*((UINT32*)(Source+Index)) ^ *((UINT32*)(Destination+Index))) & *((UINT32*)(Buffer+Index))) {
        return TRUE;
      }
      Index += 4;
    } else if ((NumBytes - Index) >= 2) {
      if ((*((UINT16*)(Source+Index)) ^ *((UINT16*)(Destination+Index))) & *((UINT16*)(Buffer+Index))) {
        return TRUE;
      }
      Index += 2;
    } else if ((NumBytes - Index) >= 1) {
      if ((*((UINT8*)(Source+Index)) ^ *((UINT8*)(Destination+Index))) & *((UINT8*)(Buffer+Index))) {
        return TRUE;
      }
      Index += 1;
    }
  }
  return FALSE;
}

#endif
