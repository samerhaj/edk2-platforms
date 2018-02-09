/** @file
*
*  Copyright 2017-2018 NXP
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

#ifndef SOC_CLOCK_LIB_H_
#define SOC_CLOCK_LIB_H_

typedef enum {
  IP_SYSCLK = 0,
  IP_DDR,
  IP_CPU,
  IP_FMAN,
  IP_ESDHC,
  IP_QSPI,
  IP_IFC,
  IP_USB,
  IP_USB_PHY,
  IP_PCI,
  IP_GPIO,
  IP_DUART,
  IP_LPUART,
  IP_WDOG,
  IP_SPI,
  IP_I2C,
  IP_SATA,
  IP_MAX
} IP_MODULES;

/**
  Return the input clock frequency to an IP Module.
  If a module is disabled or doesn't exist on platform, then return zero.

  @param[in]  IpModule   The IP module whose input clock frequency is needed.
  @param[in]  Instance   The Instance of IP module whose input clock frequency is needed.
                         if there are multiple modules of same type then this value tells the
                         instance of module for which clock is to be retrieved.
                         (e.g. if there are four i2c controllers in SOC, then this value can be 1, 2, 3, 4)
                         for IP modules which have only single instance in SOC (e.g. one QSPI controller)
                         this value should be 0.

  @return      > 0       Return the input clock frequency to an IP Module
                0        either IP module doesn't exist in SOC
                         or IP module instance doesn't exist in SOC
                         or IP module instance is disabled. i.e. no input clock is provided to IP module instance.
**/
UINT64
SocGetClock (
  IN  IP_MODULES  IpModule,
  IN  UINT32      Instance
  );

#endif // NXP_SOC_CLOCK_LIB_H_
