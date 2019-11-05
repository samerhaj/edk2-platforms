/** Pcf2129Rtc.h
*
*  Copyright 2017, 2019 NXP
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

#ifndef __PCF2129RTC_H__
#define __PCF2129RTC_H__

#define BOOTTIME_DEBUG(x)       do { if (!EfiAtRuntime()) DEBUG (x); } while (0)

/*
 * RTC register addresses
 */
#define PCF2129_CTRL1_REG_ADDR      0x00  // Control Register 1
#define PCF2129_CTRL2_REG_ADDR      0x01  // Control Register 2
#define PCF2129_CTRL3_REG_ADDR      0x02  // Control Register 3
#define PCF2129_SEC_REG_ADDR        0x03
#define PCF2129_MIN_REG_ADDR        0x04
#define PCF2129_HR_REG_ADDR         0x05
#define PCF2129_DAY_REG_ADDR        0x06
#define PCF2129_WEEKDAY_REG_ADDR    0x07
#define PCF2129_MON_REG_ADDR        0x08
#define PCF2129_YR_REG_ADDR         0x09

/*
 *RTC Alarm register count
 */
#define PCF2129_GET_WAKEUP_TIME_REG_COUNT   4
#define PCF2129_SET_WAKEUP_TIME_REG_COUNT   4

#define PCF2129_ALARM_SEC_REG_ADDR  0x0A  //Address of Alarm Second register

/*
 *PCF2129 ALARM register offset
 */
#define PCF2129_ALARM_SEC_OFFSET    0
#define PCF2129_ALARM_MIN_OFFSET    1
#define PCF2129_ALARM_HR_OFFSET     2
#define PCF2129_ALARM_DAY_OFFSET    3
#define OPERATION_COUNT             2  //I2C Request num
#define PCF2129_REG_OFFSET_LEN      1  //8 bit address len

/*
 *Mask for PCF2129 ALARM Register
 */
#define PCF2129_ALARM_SEC_REG_MASK  0x7F
#define PCF2129_ALARM_MIN_REG_MASK  0x3F
#define PCF2129_ALARM_HR_REG_MASK   0x3F
#define PCF2129_ALARM_DAY_REG_MASK  0x1F

/*
 *Read/Write flag of RTC
 */
#define PCF2129_READ_FLAG           0x01
#define PCF2129_WRITE_FLAG          0x00
#define PCF2129_CTRL3_BIT_BLF       BIT2    /* Battery Low Flag*/

// Define EPOCH (1998-JANUARY-01) in the Julian Date representation
#define EPOCH_JULIAN_DATE           2450815

typedef struct {
  UINTN                           OperationCount; //RTC operation on I2C Bus
  EFI_I2C_OPERATION               SetAddressOp;
  EFI_I2C_OPERATION               GetSetDateTimeOp;
} RTC_I2C_REQUEST;

#endif // __PCF2129RTC_H__
