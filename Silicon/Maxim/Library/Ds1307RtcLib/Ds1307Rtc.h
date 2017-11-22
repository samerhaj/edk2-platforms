/** Ds1307Rtc.h
*
*  Copyright 2017 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#ifndef __DS1307RTC_H__
#define __DS1307RTC_H__

/*
 * RTC time register
 */
#define DS1307_SEC_REG_ADDR        0x00
#define DS1307_MIN_REG_ADDR        0x01
#define DS1307_HR_REG_ADDR         0x02
#define DS1307_DAY_REG_ADDR        0x03
#define DS1307_DATE_REG_ADDR       0x04
#define DS1307_MON_REG_ADDR        0x05
#define DS1307_YR_REG_ADDR         0x06

#define DS1307_SEC_BIT_CH          0x80  /* Clock Halt (in Register 0)   */

/*
 * RTC control register
 */
#define DS1307_CTL_REG_ADDR        0x07

#define START_YEAR                 1970
#define END_YEAR                   2070

/*
 * TIME MASKS
 */
#define MASK_SEC                   0x7F
#define MASK_MIN                   0x7F
#define MASK_HOUR                  0x3F
#define MASK_DAY                   0x3F
#define MASK_MONTH                 0x1F

/*
 * I2C FLAGS
 */
#define I2C_REG_ADDRESS            0x2

typedef struct {
  UINTN                           OperationCount;
  EFI_I2C_OPERATION               SetAddressOp;
  EFI_I2C_OPERATION               GetSetDateTimeOp;
} RTC_I2C_REQUEST;

#endif // __DS1307RTC_H__
