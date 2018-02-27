/** @file
  Header file for Sata Controller initialization driver.

  Copyright 2017-2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

 **/

#ifndef _SATA_INIT_H_
#define _SATA_INIT_H_

#define AHCI_BAR_INDEX         0x05
//
// Offset for AHCI base address in PCI Header
//
#define PCI_AHCI_BASE_ADDRESS  0x24

#define SATA_PPCFG             0xA8
#define SATA_PTC               0xC8
#define SATA_PAXIC             0xC0

#define PORT_PHYSICAL          0xA003FFFE
#define PORT_TRANSPORT         0x08000025
#define PORT_RXWM              0x08000029
#define ENABLE_NONZERO_4MB_PRD 0x10000000

#define DCSR_SATA_ERRATA       0x20140520
#define SERDES2_SATA_ERRATA    0x01eb1300

#endif
