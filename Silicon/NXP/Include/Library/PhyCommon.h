/** PhyCommon.h
  Ethernet PHY private common declarations

  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __PHY_COMMON_H__
#define __PHY_COMMON_H__

/* Generic MII registers. */
#define MII_BMCR                      0x00 /* Basic mode control register */
#define MII_BMSR                      0x01 /* Basic mode status register  */
#define MII_PHYSID1                   0x02 /* PHYS ID 1                */
#define MII_PHYSID2                   0x03 /* PHYS ID 2                */
#define MII_ADVERTISE                 0x04 /* Advertisement control reg   */
#define MII_LPA                       0x05 /* Link partner ability reg    */
#define MII_EXPANSION                 0x06 /* Expansion register              */
#define MII_CTRL1000                  0x09 /* 1000BASE-T control              */
#define MII_STAT1000                  0x0a /* 1000BASE-T status        */
#define MII_ESTATUS                   0x0f /* Extended Status */
#define MII_DCOUNTER                  0x12 /* Disconnect counter              */
#define MII_FCSCOUNTER                0x13 /* False carrier counter       */
#define MII_NWAYTEST                  0x14 /* N-way auto-neg test reg     */
#define MII_RERRCOUNTER               0x15 /* Receive error counter       */
#define MII_SREVISION                 0x16 /* Silicon revision         */
#define MII_LBRERROR                  0x18 /* Lpback, rx, bypass error    */
#define MII_PHYADDR                   0x19 /* PHY address                     */
#define MII_TPISTATUS                 0x1b /* TPI status for 10mbps       */
#define MII_NCONFIG                   0x1c /* Network interface config    */

/* BMCR */
#define PHY_BMCR_ANRESTART            0x0200 /* 1 = Restart autonegotiation */
#define PHY_BMCR_ISOLATE              0x0400 /* 0 = Isolate PHY */
#define PHY_BMCR_ANENABLE             0x1000 /* 1 = Enable autonegotiation */
#define PHY_BMCR_RESET                0x8000 /* 1 = Reset the PHY */

/* BSMR */
#define PHY_BMSR_LSTATUS              0x0004 /* 1 = Link up */
#define PHY_BMSR_ANEGCAPABLE          0x0008 /* 1 = Able to perform auto-neg */
#define PHY_BMSR_ANEGCOMPLETE         0x0020 /* 1 = Auto-neg complete */

/* Advertisement control register. */
#define PHY_ADVERTISE_SLCT            0x001f  /* Selector bits               */
#define PHY_ADVERTISE_CSMA            0x0001  /* Only selector supported     */
#define PHY_ADVERTISE_10HALF          0x0020  /* Try for 10mbps half-duplex  */
#define PHY_ADVERTISE_1000XFULL       0x0020  /* Try for 1000BASE-X full-duplex */
#define PHY_ADVERTISE_10FULL          0x0040  /* Try for 10mbps full-duplex  */
#define PHY_ADVERTISE_1000XHALF       0x0040  /* Try for 1000BASE-X half-duplex */
#define PHY_ADVERTISE_100HALF         0x0080  /* Try for 100mbps half-duplex */
#define PHY_ADVERTISE_1000XPAUSE      0x0080  /* Try for 1000BASE-X pause    */
#define PHY_ADVERTISE_100FULL         0x0100  /* Try for 100mbps full-duplex */
#define PHY_ADVERTISE_1000XPSE_ASYM   0x0100  /* Try for 1000BASE-X asym pause */
#define PHY_ADVERTISE_100BASE4        0x0200  /* Try for 100mbps 4k packets  */
#define PHY_ADVERTISE_PAUSE_CAP       0x0400  /* Try for pause               */
#define PHY_ADVERTISE_PAUSE_ASYM      0x0800  /* Try for asymetric pause     */
#define PHY_ADVERTISE_RESV            0x1000  /* Unused...                   */
#define PHY_ADVERTISE_RFAULT          0x2000  /* Say we can detect faults    */
#define PHY_ADVERTISE_LPACK           0x4000  /* Ack link partners response  */
#define PHY_ADVERTISE_NPAGE           0x8000  /* Next page bit               */

#define PHY_ADVERTISE_FULL            (PHY_ADVERTISE_100FULL | PHY_ADVERTISE_10FULL | \
                                       PHY_ADVERTISE_CSMA)
#define PHY_ADVERTISE_ALL             (PHY_ADVERTISE_10HALF | PHY_ADVERTISE_10FULL | \
                                       PHY_ADVERTISE_100HALF | PHY_ADVERTISE_100FULL)
/* 1000BASE-T Control register */
#define PHY_ADVERTISE_1000FULL        0x0200  /* Advertise 1000BASE-T full duplex */
#define PHY_ADVERTISE_1000HALF        0x0100  /* Advertise 1000BASE-T half duplex */

/* MII_STAT1000 masks */
#define PHY_1000BTSR_1000HD           0x0400
#define PHY_1000BTSR_1000FD           0x0800
#define PHY_1000BTSR_RRS              0x1000
#define PHY_1000BTSR_LRS              0x2000
#define PHY_1000BTSR_MSCR             0x4000
#define PHY_1000BTSR_MSCF             0x8000

/* Link partner ability register */
#define PHY_LPA_SLCT                  0x001f /* Same as advertise selector  */
#define PHY_LPA_10HALF                0x0020 /* Can do 10mbps half-duplex   */
#define PHY_LPA_1000XFULL             0x0020 /* Can do 1000BASE-X full-duplex */
#define PHY_LPA_10FULL                0x0040 /* Can do 10mbps full-duplex   */
#define PHY_LPA_1000XHALF             0x0040 /* Can do 1000BASE-X half-duplex */
#define PHY_LPA_100HALF               0x0080 /* Can do 100mbps half-duplex  */
#define PHY_LPA_1000XPAUSE            0x0080 /* Can do 1000BASE-X pause     */
#define PHY_LPA_100FULL               0x0100 /* Can do 100mbps full-duplex  */
#define PHY_LPA_1000XPAUSE_ASYM       0x0100 /* Can do 1000BASE-X pause asym*/
#define PHY_LPA_100BASE4              0x0200 /* Can do 100mbps 4k packets   */
#define PHY_LPA_PAUSE_CAP             0x0400 /* Can pause                */
#define PHY_LPA_PAUSE_ASYM            0x0800 /* Can pause asymetrically     */
#define PHY_LPA_RFAULT                0x2000 /* Link partner faulted        */
#define PHY_LPA_PHY_LPACK             0x4000 /* Link partner acked us       */
#define PHY_LPA_NPAGE                 0x8000 /* Next page bit            */

#define PHY_LPA_DUPLEX                (PHY_LPA_10FULL | PHY_LPA_100FULL)
#define PHY_LPA_100                   (PHY_LPA_100FULL | PHY_LPA_100HALF | PHY_LPA_100BASE4)

/* Basic mode control register */
#define PHY_BMCR_RESV                 0x003f /* Unused...                */
#define PHY_BMCR_SPEED1000            0x0040 /* MSB of Speed (1000)             */
#define PHY_BMCR_CTST                 0x0080 /* Collision test           */
#define PHY_BMCR_FULLDPLX             0x0100 /* Full duplex                     */
#define PHY_BMCR_ANRESTART            0x0200 /* Auto negotiation restart    */
#define PHY_BMCR_ISOLATE              0x0400 /* Disconnect DP83840 from MII */
#define PHY_BMCR_PDOWN                0x0800 /* Powerdown the DP83840       */
#define PHY_BMCR_ANENABLE             0x1000 /* Enable auto negotiation     */
#define PHY_BMCR_SPEED100             0x2000 /* Select 100Mbps           */
#define PHY_BMCR_LOOPBACK             0x4000 /* TXD loopback bits        */
#define PHY_BMCR_RESET                0x8000 /* Reset the DP83840        */

/* Basic mode status register */
#define PHY_BMSR_ERCAP                0x0001 /* Ext-reg capability              */
#define PHY_BMSR_JCD                  0x0002 /* Jabber detected          */
#define PHY_BMSR_LSTATUS              0x0004 /* Link status                     */
#define PHY_BMSR_ANEGCAPABLE          0x0008 /* Able to do auto-negotiation */
#define PHY_BMSR_RFAULT               0x0010 /* Remote fault detected       */
#define PHY_BMSR_ANEGCOMPLETE         0x0020 /* Auto-negotiation complete   */
#define PHY_BMSR_RESV                 0x00c0 /* Unused...                */
#define PHY_BMSR_ESTATEN              0x0100 /* Extended Status in R15 */
#define PHY_BMSR_100HALF2             0x0200 /* Can do 100BASE-T2 HDX */
#define PHY_BMSR_100FULL2             0x0400 /* Can do 100BASE-T2 FDX */
#define PHY_BMSR_10HALF               0x0800 /* Can do 10mbps, half-duplex  */
#define PHY_BMSR_10FULL               0x1000 /* Can do 10mbps, full-duplex  */
#define PHY_BMSR_100HALF              0x2000 /* Can do 100mbps, half-duplex */
#define PHY_BMSR_100FULL              0x4000 /* Can do 100mbps, full-duplex */
#define PHY_BMSR_100BASE4             0x8000 /* Can do 100mbps, 4k packets  */

/* Extended Status */
#define PHY_ESTATUS_1000_XFULL        0x8000 /* Can do 1000BX Full */
#define PHY_ESTATUS_1000_XHALF        0x4000 /* Can do 1000BX Half */
#define PHY_ESTATUS_1000_TFULL        0x2000 /* Can do 1000BT Full */
#define PHY_ESTATUS_1000_THALF        0x1000 /* Can do 1000BT Half */

#define MII_MMD_CTRL                  0x0d   /* MMD Access Control Register */
#define MII_MMD_DATA                  0x0e   /* MMD Access Data Register */
#define MII_MMD_CTRL_NOINCR           0x4000 /* no post increment */

/* INPHI PHY ID */
#define PHY_UID_IN112525_S03       0x02107440

EFI_STATUS
UpdatePhyLink (
  DPAA2_PHY *Dpaa2Phy
  );

VOID
ParsePhyLink (
  DPAA2_PHY *Dpaa2Phy
  );

VOID
ConfigAneg (
  DPAA2_PHY *Dpaa2Phy
  );

VOID
RestartAneg (
  DPAA2_PHY *Dpaa2Phy
  );

#endif /* __PHY_COMMON_H__ */
