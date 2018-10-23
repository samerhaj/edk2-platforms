/** @file

  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __PCF85263_H__
#define __PCF85263_H__

// Define EPOCH (1998-JANUARY-01) in the Julian Date representation
#define EPOCH_JULIAN_DATE         2450815

/*
 * RTC mode time register
 */
#define PCF85263_100SEC_REG_ADDR  0x00
#define PCF85263_SEC_REG_ADDR     0x01
#define PCF85263_MIN_REG_ADDR     0x02
#define PCF85263_HR_REG_ADDR      0x03
#define PCF85263_DATE_REG_ADDR    0x04
#define PCF85263_DAY_REG_ADDR     0x05
#define PCF85263_MON_REG_ADDR     0x06
#define PCF85263_YR_REG_ADDR      0x07

#define PCF85263_SEC_BIT_OSC      0x80

/*
 * RTC control register
 */
#define PCF85263_OSC_REG_ADDR     0x25
#define PCF85263_FUNC_REG_ADDR    0x28

#define ENABLE_100SEC             0x80
#define ENABLE_12HOUR_FORMAT      0x20

/*
 * RTC stop register
 */
#define PCF85263_STOPBIT_REG_ADDR 0x2E

#define STOP_RTC                  0x01
#define START_RTC                 0x00

/*
 * RTC reset register
 */
#define PCF85263_RESET_REG_ADDR   0x2F

#define CLEAR_PRESCALER           0xA4

/*
 * TIME MASKS
 */
#define MASK_SEC                   0x7F
#define MASK_MIN                   0x7F
#define MASK_HOUR                  0x3F
#define MASK_DAY                   0x3F
#define MASK_MONTH                 0x1F

typedef struct {
  UINTN                           OperationCount;
  EFI_I2C_OPERATION               SetAddressOp;
  EFI_I2C_OPERATION               GetSetDateTimeOp;
} RTC_I2C_REQUEST;

#endif //__PCF85263_H__
