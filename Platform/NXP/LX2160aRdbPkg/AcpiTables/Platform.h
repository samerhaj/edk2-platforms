/** @file
*
*  Copyright (c) 2011-2015, ARM Limited. All rights reserved.
*  Copyright (c) 2015, Linaro Limited. All rights reserved.
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
*  Based on the files under ArmPlatformPkg/ArmJunoPkg/AcpiTables/
*
**/


#ifndef _LX2160ARDB_PLATFORM_H_
#define _LX2160ARDB_PLATFORM_H_

//
// ACPI table information used to initialize tables.
//
#define EFI_ACPI_ARM_OEM_ID             'N','X','P',' ',' ',' '   // OEMID
#define EFI_ACPI_ARM_OEM_TABLE_ID       SIGNATURE_64('L','X','2','1','6','0',' ',' ') // OEM table id, putting SOC name here
#define EFI_ACPI_ARM_OEM_REVISION       0x00000000
#define EFI_ACPI_ARM_CREATOR_ID         SIGNATURE_32('I','N','T','L')
#define EFI_ACPI_ARM_CREATOR_REVISION   0x20151124

// A macro to initialise the common header part of EFI ACPI tables as defined by
// EFI_ACPI_DESCRIPTION_HEADER structure.
#define ARM_ACPI_HEADER(Signature, Type, Revision) {              \
    Signature,                      /* UINT32  Signature */       \
    sizeof (Type),                  /* UINT32  Length */          \
    Revision,                       /* UINT8   Revision */        \
    0,                              /* UINT8   Checksum */        \
    { EFI_ACPI_ARM_OEM_ID },        /* UINT8   OemId[6] */        \
    EFI_ACPI_ARM_OEM_TABLE_ID,      /* UINT64  OemTableId */      \
    EFI_ACPI_ARM_OEM_REVISION,      /* UINT32  OemRevision */     \
    EFI_ACPI_ARM_CREATOR_ID,        /* UINT32  CreatorId */       \
    EFI_ACPI_ARM_CREATOR_REVISION   /* UINT32  CreatorRevision */ \
  }

// Gic
#define GIC_VERSION 3
#define GICD_BASE   0x6000000
#define GICI_BASE   0x6020000
#define GICR_BASE   0x06200000
#define GICR_LEN    0x200000
#define GICC_BASE   0x0c0c0000
#define GICH_BASE   0x0c0d0000
#define GICV_BASE   0x0c0e0000

// UART
#define UART0_BASE               0x21C0000
#define UART1_BASE               0x21D0000
#define UART2_BASE               0x21E0000
#define UART3_BASE               0x21F0000
#define UART0_IT                 64
#define UART1_IT                 65
#define UART2_IT                 104
#define UART3_IT                 105
#define UART_LEN                 0x10000
#define SPCR_FLOW_CONTROL_NONE   0

// I2C
#define I2C0_BASE 0x2000000
#define I2C1_BASE 0x2010000
#define I2C2_BASE 0x2020000
#define I2C3_BASE 0x2030000
#define I2C4_BASE 0x2040000
#define I2C5_BASE 0x2050000
#define I2C6_BASE 0x2060000
#define I2C7_BASE 0x2070000
#define I2C_LEN   0x10000
#define I2C0_IT   66
#define I2C1_IT   66
#define I2C2_IT   67
#define I2C3_IT   67
#define I2C4_IT   106
#define I2C5_IT   106
#define I2C6_IT   107
#define I2C7_IT   107

// Sata
#define SATA0_BASE 0x3200000
#define SATA1_BASE 0x3210000
#define SATA2_BASE 0x3220000
#define SATA3_BASE 0x3230000
#define SATA_LEN   0x10000
#define SATA0_IT_1 165
#define SATA0_IT_2 166
#define SATA0_IT_3 167
#define SATA1_IT_1 168
#define SATA1_IT_2 169
#define SATA1_IT_3 170
#define SATA2_IT_1 129
#define SATA2_IT_2 130
#define SATA2_IT_3 131
#define SATA3_IT_1 132
#define SATA3_IT_2 133
#define SATA3_IT_3 134

//USB
#define USB0_BASE 0x3100000
#define USB1_BASE 0x3110000
#define USB_LEN   0x10000
#define USB0_IT   112
#define USB1_IT   113

// DSPI
#define SPI0_BASE 0x2100000
#define SPI1_BASE 0x2110000
#define SPI2_BASE 0x2120000
#define SPI_LEN   0x10000
#define SPI_IT    58

//Flex SPI
#define FSPI0_BASE 0x20c0000
#define FSPI_LEN   0x10000
#define FSPI_IT    57

// SD
#define SDC0_BASE 0x2140000
#define SDC1_BASE 0x2150000
#define SDC_LEN   0x10000
#define SDC0_IT   60
#define SDC1_IT   95

// Timer
#define TIMER_BLOCK_COUNT            1
#define TIMER_FRAME_COUNT            4
#define TIMER_WATCHDOG_COUNT         2
#define TIMER_BASE_ADDRESS           0x23E0000 // a.k.a CNTControlBase
#define TIMER_READ_BASE_ADDRESS      0x23F0000 // a.k.a CNTReadBase
#define TIMER_GT_BLOCK_0_ADDRESS     0x2880000 // a.k.a CNTCTLBase (Secure)
#define TIMER_GT_BLOCK_1_ADDRESS     0x2890000 // a.k.a CNTCTLBase (Secure)
#define TIMER_GT_BASE_0_ADDRESS      0x28A0000  // a.k.a CNTBase0
#define TIMER_GT_BASE_1_ADDRESS      0x28B0000  // a.k.a CNTBase1
#define TIMER_GT_BASE_2_ADDRESS      0x28C0000  // a.k.a CNTBase2
#define TIMER_GT_BASE_3_ADDRESS      0x28D0000  // a.k.a CNTBase3
#define TIMER_GT_BASE_0_EL0_ADDRESS  0x28E0000  // a.k.a CNTBase0EL0
#define TIMER_GT_BASE_2_EL0_ADDRESS  0x28F0000  // a.k.a CNTBase2EL0
#define TIMER_WDT0_REFRESH_BASE      0x2390000
#define TIMER_WDT0_CONTROL_BASE      0x23A0000
#define TIMER_WDT1_REFRESH_BASE      0x23B0000
#define TIMER_WDT1_CONTROL_BASE      0x23C0000
#define TIMER_SEC_IT                 29
#define TIMER_NON_SEC_IT             30
#define TIMER_VIRT_IT                27
#define TIMER_HYP_IT                 26
#define TIMER_FRAME0_IT              78
#define TIMER_FRAME1_IT              79
#define TIMER_FRAME2_IT              92
#define TIMER_FRAME3_IT              93
#define TIMER_WDT0_IT                91
#define TIMER_WDT1_IT                108

#define DEFAULT_PLAT_FREQ            700000000
#endif
