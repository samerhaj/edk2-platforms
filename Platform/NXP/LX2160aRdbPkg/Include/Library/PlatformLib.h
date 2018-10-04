/** LX2160aRdb.h
*  Header defining the LX2160aRdb constants (Base addresses, sizes, flags)
*
*  Copyright 2018 NXP
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

#ifndef __LX2160ARDB_PLATFORM_H__
#define __LX2160ARDB_PLATFORM_H__

// Base address of i2c controllers
#define I2C0_BASE_ADDRESS                          0x02000000
#define I2C1_BASE_ADDRESS                          0x02010000
#define I2C2_BASE_ADDRESS                          0x02020000
#define I2C3_BASE_ADDRESS                          0x02030000
#define I2C4_BASE_ADDRESS                          0x02040000
#define I2C5_BASE_ADDRESS                          0x02050000
#define I2C6_BASE_ADDRESS                          0x02060000
#define I2C7_BASE_ADDRESS                          0x02070000
// Size of each controller
#define I2C_SIZE                                   0x10000
#define I2C_BUS_MAX                                8
#define I2C_DEBUG_REG                              0x6
#define I2C_GLITCH_EN                              0x8


#define SCFG_BASE_ADDR                             0x1FC0000
#define SCFG_SIZE                                  0x0010000

#define LX2160A_DDR_ADDR                           0x01080000
#define LX2160A_DDR2_ADDR                          0x01090000

// USB specific values and register
// To do : Verify with RM (when available)
#define SCFG_USB3PRM1CR                            0x000
#define SCFG_USB3PRM1CR_INIT                       0x27672b2a
#define SCFG_USB3PRM2CR                            0x004
#define USB_TXVREFTUNE                             0x9
#define USB_SQRXTUNE                               0xFC7FFFFF

#define DCSR_BASE                                  0x700000000ULL
#define DCSR_USB_PHY1                              0x4600000
#define DCSR_USB_PHY2                              0x4610000
#define DCSR_USB_PHY_RX_OVRD_IN_HI                 0x1006
#define USB_PHY_RX_EQ_VAL_1                        0x0000
#define USB_PHY_RX_EQ_VAL_2                        0x0080
#define USB_PHY_RX_EQ_VAL_3                        0x0380
#define USB_PHY_RX_EQ_VAL_4                        0x0b80

/* TZ Protection Controller Definitions */
#define NXP_TZPC_BASE                              0x02200000
#define NXP_TZPCR0SIZE_BASE                        (NXP_TZPC_BASE)
#define NXP_TZPCDECPROT_0_STAT_BASE                (NXP_TZPC_BASE + 0x800)
#define NXP_TZPCDECPROT_0_SET_BASE                 (NXP_TZPC_BASE + 0x804)
#define NXP_TZPCDECPROT_0_CLR_BASE                 (NXP_TZPC_BASE + 0x808)

/* NXP_TZ Address Space Controller Definitions */
#define NXP_TZASC1_BASE                            0x01100000  /* as per CCSR map. */
#define NXP_TZASC_GATE_KEEPER(x)                   ((NXP_TZASC1_BASE + (x * 0x10000)) + 0x008)
#define NXP_TZASC_REGION_ATTRIBUTES_0(x)           ((NXP_TZASC1_BASE + (x * 0x10000)) + 0x110)
#define NXP_TZASC_REGION_ID_ACCESS_0(x)            ((NXP_TZASC1_BASE + (x * 0x10000)) + 0x114)
#define SVR_DEV_LX2160                             0x8736

/* TrustZone Protection Controller Definitions */
#define TZPC_BASE_ADDR                            0x02200000
#define TZPC_DCFG_RESET_BIT_MASK                  0x08

// to do check these base addresses
#define CONFIG_SYS_FSL_DCSR_DDR_ADDR              0x70012c000ULL
#define CONFIG_SYS_FSL_DCSR_DDR2_ADDR             0x70012d000ULL
#define CONFIG_SYS_FSL_DCSR_DDR3_ADDR             0x700132000ULL

/* SMMU Defintions */
#define SMMU_BASE_ADDR                            0x05000000
#define SMMU_REG_SCR0                             (SMMU_BASE_ADDR + 0x0)
#define SMMU_REG_SACR                             (SMMU_BASE_ADDR + 0x10)
#define SMMU_REG_IDR1                             (SMMU_BASE_ADDR + 0x24)
#define SMMU_REG_NSCR0                            (SMMU_BASE_ADDR + 0x400)
#define SMMU_REG_NSACR                            (SMMU_BASE_ADDR + 0x410)

#define SACR_PAGESIZE_MASK                        0x00010000
#define SCR0_CLIENTPD_MASK                        0x00000001
#define SCR0_USFCFG_MASK                          0x00000400
#define CONFIG_SYS_FSL_DDR_LE
#endif
