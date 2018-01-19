/** WatchDog.h
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

#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__

#define WDOG_SIZE           0x1000
#define WDOG_WCR_OFFSET     0
#define WDOG_WSR_OFFSET     2
#define WDOG_WRSR_OFFSET    4
#define WDOG_WICR_OFFSET    6
#define WDOG_WCR_WT         (0xFF << 8)
#define WDOG_WCR_WDE        (1 << 2)
#define WDOG_SERVICE_SEQ1   0x5555
#define WDOG_SERVICE_SEQ2   0xAAAA
#define WDOG_WCR_WDZST      0x1
#define WDOG_WCR_WRE        (1 << 3)  /* -> WDOG Reset Enable */

#define WT_MAX_TIME         128
#define WD_COUNT(Sec)       (((Sec) * 2 - 1) << 8)
#define WD_COUNT_MASK       0xff00
#define WD_SEC(Cnt)         (((Cnt) + 1) / 2)

#define NANO_SECOND_BASE    10000000

#define WT_FEED_INTERVAL    (WT_MAX_TIME * NANO_SECOND_BASE)

#endif //__WATCHDOG_H__
