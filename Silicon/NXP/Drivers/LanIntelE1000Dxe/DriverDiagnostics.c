/**************************************************************************

Copyright (c) 2001-2010, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 3. Neither the name of the Intel Corporation nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/

#include "e1000.h"

extern EFI_DRIVER_BINDING_PROTOCOL  mGigUndiDriverBinding;

extern EFI_GUID gEfiPro1000ComGuid;


#define MAX_ETHERNET_SIZE       1518
#define TEST_PACKET_SIZE        1024

#if (DBG_LVL & DIAG)
#define PHY_LOOPBACK_ITERATIONS 10
#else
#define PHY_LOOPBACK_ITERATIONS 10000
#endif

#define PHY_PHLBKC              19
#define PHY_PHCTRL1             23
#define PHY_PHSTAT              26

EFI_STATUS
GigUndiDriverDiagnosticsRunDiagnostics (
  IN EFI_DRIVER_DIAGNOSTICS_PROTOCOL             *This,
  IN EFI_HANDLE                                  ControllerHandle,
  IN EFI_HANDLE                                  ChildHandle OPTIONAL,
  IN EFI_DRIVER_DIAGNOSTIC_TYPE                  DiagnosticType,
  IN CHAR8                                       *Language,
  OUT EFI_GUID                                   **ErrorType,
  OUT UINTN                                      *BufferSize,
  OUT CHAR16                                     **Buffer
  );

EFI_STATUS
GigUndiRunPhyLoopback (
  GIG_DRIVER_DATA   *GigAdapterInfo,
  PXE_CPB_TRANSMIT  PxeCpbTransmit
  );

BOOLEAN
_SetIntegratedM88PhyLoopback(
    struct e1000_hw                     *hw,
    IN  UINT16                          Speed
    );

BOOLEAN
_SetNinevehPhyLoopback(
    struct e1000_hw  *hw,
    IN  UINT16        Speed
    );

BOOLEAN
_SetIgpPhyLoopback(
    IN  struct e1000_hw                 *hw,
    IN  UINT16                          Speed
    );

EFI_DRIVER_DIAGNOSTICS_PROTOCOL     gGigUndiDriverDiagnostics = {
  GigUndiDriverDiagnosticsRunDiagnostics,
  "eng"
};

EFI_DRIVER_DIAGNOSTICS2_PROTOCOL     gGigUndiDriverDiagnostics2 = {
  (EFI_DRIVER_DIAGNOSTICS2_RUN_DIAGNOSTICS) GigUndiDriverDiagnosticsRunDiagnostics,
  "en-US"
};


#define E1000_RCTL_LBM_MASK (0x000000C0)  /* bitmask to retrieve LBM bits */

typedef enum {
  e1000_lbm_none        = 0,
  e1000_lbm_mac,
  e1000_lbm_phy_1000,
  e1000_lbm_phy_100,
  e1000_lbm_phy_10,
  e1000_lbm_transceiver,
  e1000_lbm_count,
  e1000_lbm_invalid     = 0xFF
} e1000_lbm_type;

UINT8 Packet[MAX_ETHERNET_SIZE];

#pragma pack(1)
typedef struct {
  UINT8 DestAddr[6];
  UINT8 SourceAddr[6];
  UINT8 Length[2];
} ETHERNET_HDR;
#pragma pack()

/***************************************************************************
**
** Name:            _NalGenericReadPhyRegister16Ex()
**
** Description:     Writes a 16bit value to the Phy. - allows user to
**                  set PHY Page
**
** Author:          GVR
**
** Born on Date:    06/01/2005
**
** Arguments:       Handle   = Handle to this adapter.
**                  Page     = PHY Page # or device #
**                  Offset = The register to read from the PHY. This is a
**                             numeric offset value.
**                  Value    = The value read to return.
**
** Returns:         NAL_STATUS
**
****************************************************************************/
INT32
_ReadPhyRegister16Ex(
    IN  struct e1000_hw           *hw,
    IN  UINT32             Page,
    IN  UINT32             Offset,
    OUT UINT16*            Value
    )
{
    //
    // See e1000_hw.c and e1000_hw.h for bit field specifications for PHY page register
    //
    Page = (Page & 0x07FF) << 5;
    Offset = ((Offset & 0x1F) | Page);

    return e1000_read_phy_reg(hw, Offset, Value);
}

/***************************************************************************
**
** Name:            _NalGenericWritePhyRegister16Ex()
**
** Description:     Writes a 16bit value to the Phy. - allows user to
**                  set PHY Page
**
** Author:          GVR
**
** Born on Date:    06/01/2005
**
** Arguments:       Handle   = Handle to this adapter.
**                  Page     = PHY Page #
**                  Register = The register to write to the PHY. This is a
**                             numeric offset value.
**                  Data     = The value read to write.
**
** Returns:         NAL_STATUS
**
****************************************************************************/
INT32
_WritePhyRegister16Ex(
    IN  struct e1000_hw           *hw,
    IN  UINT32             Page,
    IN  UINT32             Offset,
    IN  UINT16             Data
    )
{
    /* see sdk/adapters/module0/e1000_hw.c and e1000_hw.h for bit field
     * specifications for PHY page register */
    Page = (Page & 0x07FF) << 5;
    Offset = ((Offset & 0x1F) | Page);

    return e1000_write_phy_reg(hw, Offset, Data);
}


VOID
_BuildPacket (
  GIG_DRIVER_DATA *GigAdapterInfo
  )
/*++

Routine Description:
   Build a packet to transmit in the phy loopback test.

Arguments:
  GigAdapterInfo                   - Pointer to the NIC data structure information
                                    which the UNDI driver is layering on so that we can
                                    get the MAC address
Returns:
  Sets the global array Packet[] with the packet to send out during PHY loopback.

--*/
{
  ETHERNET_HDR  *EthernetHdr;
  UINT16        Length;
  UINT16        i;

  EthernetHdr = NULL;
  Length      = 0;
  i           = 0;

  ZeroMem ((char *) Packet, MAX_ETHERNET_SIZE);

  //
  // First copy the source and destination addresses
  //
  EthernetHdr = (ETHERNET_HDR *) Packet;
  CopyMem ((char *) &EthernetHdr->SourceAddr, (char *) GigAdapterInfo->hw.mac.addr, ETH_ADDR_LEN);
  CopyMem ((char *) &EthernetHdr->DestAddr, (char *) GigAdapterInfo->BroadcastNodeAddress, ETH_ADDR_LEN);

  //
  // Calculate the data segment size and store it in the header big Endian style
  //
  Length                  = TEST_PACKET_SIZE - sizeof (ETHERNET_HDR);
  EthernetHdr->Length[0]  = (UINT8) (Length >> 8);
  EthernetHdr->Length[1]  = (UINT8) Length;

  //
  // Generate Packet data
  //
  for (i = 0; i < Length; i++) {
    Packet[i + sizeof (ETHERNET_HDR)] = (UINT8) i;
  }
}

VOID
_DisplayBuffersAndDescriptors (
  GIG_DRIVER_DATA *GigAdapterInfo
  )
/*++

Routine Description:
   Display the buffer and descriptors for debuging the PHY loopback test.

Arguments:
  GigAdapterInfo                   - Pointer to the NIC data structure information
                                    which the UNDI driver is layering on so that we can
                                    get the MAC address
Returns:
  Sets the global array Packet[] with the packet to send out during PHY loopback.

--*/
{
  E1000_RECEIVE_DESCRIPTOR  *ReceiveDesc;
  E1000_TRANSMIT_DESCRIPTOR *TransmitDesc;
  UINT32                    j;

  DEBUGPRINT (DIAG, ("Receive Descriptor\n"));
  DEBUGPRINT(DIAG, ("RCTL=%X ", E1000_READ_REG(&GigAdapterInfo->hw, E1000_RCTL)));
  DEBUGPRINT(DIAG, ("RDH0=%x ", (UINT16) E1000_READ_REG (&GigAdapterInfo->hw, E1000_RDH(0))));
  DEBUGPRINT(DIAG, ("RDT0=%x ", (UINT16) E1000_READ_REG (&GigAdapterInfo->hw, E1000_RDT(0))));
  DEBUGPRINT(DIAG, ("cur_rx_ind=%X\n", GigAdapterInfo->cur_rx_ind));

  ReceiveDesc = GigAdapterInfo->rx_ring;
  for (j = 0; j < DEFAULT_RX_DESCRIPTORS; j++) {
    DEBUGPRINT (DIAG, ("Buff=%x,", ReceiveDesc->buffer_addr));
    DEBUGPRINT (DIAG, ("Len=%x,", ReceiveDesc->length));
    DEBUGPRINT (DIAG, ("Stat=%x,", ReceiveDesc->status));
    DEBUGPRINT (DIAG, ("Csum=%x,", ReceiveDesc->csum));
    DEBUGPRINT (DIAG, ("Special=%x\n", ReceiveDesc->special));
    ReceiveDesc++;
  }

  DEBUGWAIT (DIAG);
  DEBUGPRINT (DIAG, ("Transmit Descriptor\n"));
  DEBUGPRINT(DIAG, ("TCTL=%X ", E1000_READ_REG(&GigAdapterInfo->hw, E1000_TCTL)));
  DEBUGPRINT(DIAG, ("TDH0=%x ", (UINT16) E1000_READ_REG (&GigAdapterInfo->hw, E1000_TDH(0))));
  DEBUGPRINT(DIAG, ("TDT0=%x ", (UINT16) E1000_READ_REG (&GigAdapterInfo->hw, E1000_TDT(0))));
  DEBUGPRINT(DIAG, ("cur_tx_ind=%X\n", GigAdapterInfo->cur_tx_ind));

  TransmitDesc = GigAdapterInfo->tx_ring;
  for (j = 0; j < DEFAULT_TX_DESCRIPTORS; j++) {
    DEBUGPRINT (DIAG, ("Buff=%x,", TransmitDesc->buffer_addr));
    DEBUGPRINT (DIAG, ("Cmd=%x,", TransmitDesc->lower.flags.cmd));
    DEBUGPRINT (DIAG, ("Cso=%x,", TransmitDesc->lower.flags.cso));
    DEBUGPRINT (DIAG, ("Length=%x,", TransmitDesc->lower.flags.length));
    DEBUGPRINT (DIAG, ("Status= %x,", TransmitDesc->upper.fields.status));
    DEBUGPRINT (DIAG, ("Special=%x,", TransmitDesc->upper.fields.special));
    DEBUGPRINT (DIAG, ("Css=%x\n", TransmitDesc->upper.fields.css));
    TransmitDesc++;
  }

  DEBUGWAIT (DIAG);
}

BOOLEAN
_SetIntegratedM88PhyLoopback(
    struct e1000_hw                     *hw,
    IN  UINT16                          Speed
    )
/*++
  Description: This routine is used by diagnostic software to put
               the 82544, 82540, 82545, and 82546 MAC based network
               cards and the M88E1000 PHY into loopback mode.

               Loopback speed is determined by the Speed value
               passed into this routine.

               Valid values are 1000, 100, and 10 Mbps

               Current procedure is to:
               1) Disable auto-MDI/MDIX
               2) Perform SW phy reset (bit 15 of PHY_CONTROL)
               3) Disable autoneg and reset
               4) For the specified speed, set the loopback
                  mode for that speed.  Also force the MAC
                  to the correct speed and duplex for the
                  specified operation.
               5) If this is an 82543, setup the TX_CLK and
                  TX_CRS again.
               6) Disable the receiver so a cable disconnect
                  and reconnect will not cause autoneg to
                  begin.

  Arguments:    Adapter - Ptr to this card's adapter data structure
                Speed   - desired loopback speed

  Returns:      TRUE - Success, FALSE - Failure
--*/
{
    UINT32              CtrlReg         = 0;
    UINT32              StatusReg       = 0;
    UINT16              PhyReg          = 0;
    BOOLEAN             LoopbackModeSet = FALSE;

    hw->mac.autoneg = FALSE;

    /*******************************************************************
    ** Set up desired loopback speed and duplex depending on input
    ** into this function.
    *******************************************************************/
    switch(Speed)
    {
    case SPEED_1000:
        DEBUGPRINT(DIAG, ("Setting M88E1000 PHY into loopback at 1000 Mbps\n"));
        /****************************************************************
        ** Set up the MII control reg to the desired loopback speed.
        ****************************************************************/
        if (hw->phy.type == e1000_phy_igp)
        {
            e1000_write_phy_reg(hw, PHY_CONTROL, 0x4140); /* force 1000, set loopback */
        }
        else if (hw->phy.type == e1000_phy_m88)
        {
            e1000_write_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, 0x0808); /* Auto-MDI/MDIX Off */
            e1000_write_phy_reg(hw, PHY_CONTROL, 0x9140); /* reset to update Auto-MDI/MDIX */
            e1000_write_phy_reg(hw, PHY_CONTROL, 0x8140); /* autoneg off */
            e1000_write_phy_reg(hw, PHY_CONTROL, 0x4140); /* force 1000, set loopback */
        }
        else if (hw->phy.type == e1000_phy_gg82563)
        {
            e1000_write_phy_reg(hw, GG82563_PHY_KMRN_MODE_CTRL, 0x1CE); /* Force Link Up */
            e1000_write_phy_reg(hw, GG82563_REG(0, 0), 0x4140); /* bit 14 = IEEE loopback, force 1000, full duplex */
        }
        /****************************************************************
        ** Now set up the MAC to the same speed/duplex as the PHY.
        ****************************************************************/
        CtrlReg = E1000_READ_REG(hw, E1000_CTRL);
        CtrlReg &= ~E1000_CTRL_SPD_SEL;   /* Clear the Speed selection bits */
        CtrlReg |= (E1000_CTRL_FRCSPD   | /* Set the Force Speed Bit */
                    E1000_CTRL_FRCDPX   | /* Set the Force Duplex Bit */
                    E1000_CTRL_SPD_1000 | /* Force Speed to 1000 */
                    E1000_CTRL_FD);       /* Force Duplex to FULL */

        /* For some SerDes we'll need to commit the writes now so that the
         * status register is updated on link. */
        if (hw->phy.media_type == e1000_media_type_internal_serdes)
        {
            E1000_WRITE_REG (hw, E1000_CTRL, CtrlReg);
            msec_delay(100);
            CtrlReg = E1000_READ_REG(hw, E1000_CTRL);
        }

        if (hw->phy.media_type == e1000_media_type_copper)
        {
            /* For Marvel Phy, inverts Loss-Of-Signal */
            if (hw->phy.type == e1000_phy_m88)
            {
                CtrlReg |= (E1000_CTRL_ILOS); /* Invert Loss-Of-Signal */
            }
        }
        else
        {
            /* Set the ILOS bits on the fiber nic if half duplex link is detected. */
            StatusReg = E1000_READ_REG(hw, E1000_STATUS);
            if((StatusReg & E1000_STATUS_FD) == 0)
            {
                DEBUGPRINT(DIAG, ("Link seems unstable in PHY Loopback setup\n"));
                CtrlReg |= (E1000_CTRL_ILOS | E1000_CTRL_SLU);  /* Invert Loss-Of-Signal */
            }
        }

        E1000_WRITE_REG (hw, E1000_CTRL, CtrlReg);
        LoopbackModeSet = TRUE;
        break;

    case SPEED_100:
        DEBUGPRINT(DIAG, ("Setting M88E1000 PHY into loopback at 100 Mbps\n"));
        /****************************************************************
        ** Set up the MII control reg to the desired loopback speed.
        ****************************************************************/
        e1000_write_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, 0x0808); /* Auto-MDI/MDIX Off */
        e1000_write_phy_reg(hw, PHY_CONTROL, 0x9140); /* reset to update Auto-MDI/MDIX */
        e1000_write_phy_reg(hw, PHY_CONTROL, 0x8140); /* autoneg off */
        e1000_write_phy_reg(hw, PHY_CONTROL, 0x8100); /* reset to update autoneg */
        e1000_write_phy_reg(hw, M88E1000_EXT_PHY_SPEC_CTRL, 0x0c14); /* MAC interface speed to 100Mbps */
        e1000_write_phy_reg(hw, PHY_CONTROL, 0xe100); /* reset to update MAC interface speed */
        e1000_write_phy_reg(hw, PHY_CONTROL, 0x6100); /* force 100, set loopback */

        /****************************************************************
        ** Now set up the MAC to the same speed/duplex as the PHY.
        ****************************************************************/
        CtrlReg =E1000_READ_REG(hw, E1000_CTRL);
        CtrlReg &= ~E1000_CTRL_SPD_SEL;   /* Clear the Speed selection bits */
        CtrlReg |= (E1000_CTRL_ILOS   |  /* Invert Loss-Of-Signal */
                    E1000_CTRL_SLU     |  /* Set the Force Link Bit */
                    E1000_CTRL_FRCSPD  |  /* Set the Force Speed Bit */
                    E1000_CTRL_FRCDPX  |  /* Set the Force Duplex Bit */
                    E1000_CTRL_SPD_100 |  /* Force Speed to 100 */
                    E1000_CTRL_FD);       /* Force Duplex to FULL */

        E1000_WRITE_REG (hw, E1000_CTRL, CtrlReg);
        LoopbackModeSet = TRUE;
        break;

    case SPEED_10:
        DEBUGPRINT(DIAG, ("Setting M88E1000 PHY into loopback at 10 Mbps\n"));
        /****************************************************************
        ** Set up the MII control reg to the desired loopback speed.
        ****************************************************************/
        e1000_write_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, 0x0808); /* Auto-MDI/MDIX Off */
        e1000_write_phy_reg(hw, PHY_CONTROL, 0x9140); /* reset to update Auto-MDI/MDIX */
        e1000_write_phy_reg(hw, PHY_CONTROL, 0x8140); /* autoneg off */
        e1000_write_phy_reg(hw, PHY_CONTROL, 0x8100); /* reset to update autoneg */
        e1000_write_phy_reg(hw, M88E1000_EXT_PHY_SPEC_CTRL, 0x0c04); /* MAC interface speed to 10Mbps */
        e1000_write_phy_reg(hw, PHY_CONTROL, 0x8100); /* reset to update MAC interface speed */
        e1000_write_phy_reg(hw, PHY_CONTROL, 0x4100); /* force 10, set loopback */

        /****************************************************************
        ** Now set up the MAC to the same speed/duplex as the PHY.
        ****************************************************************/
        CtrlReg = E1000_READ_REG(hw, E1000_CTRL);
        CtrlReg &= ~E1000_CTRL_SPD_SEL;   /* Clear the Speed selection bits */
        CtrlReg |= (E1000_CTRL_SLU     |   /* Set the Force Link Bit */
                    E1000_CTRL_FRCSPD |   /* Set the Force Speed Bit */
                    E1000_CTRL_FRCDPX |   /* Set the Force Duplex Bit */
                    E1000_CTRL_SPD_10 |   /* Force Speed to 10 */
                    E1000_CTRL_FD);       /* Force Duplex to FULL */

        E1000_WRITE_REG (hw, E1000_CTRL, CtrlReg);
        LoopbackModeSet = TRUE;
        break;

    default:
        DEBUGPRINT(DIAG, ("Invalid speed value loopback mode \"%d\"\n", Speed));
        LoopbackModeSet = FALSE;
        break;
    }

    e1000_read_phy_reg(hw, PHY_CONTROL, &PhyReg);
    if (hw->phy.type == e1000_phy_m88)
    {

      /****************************************************************
       * Disable the receiver on the PHY so when a cable is plugged
       * in, the PHY does not begin to autoneg when a cable is
       * reconnected to the NIC.
       ***************************************************************/
      e1000_write_phy_reg(hw, 29, 0x001F);
      e1000_write_phy_reg(hw, 30, 0x8FFC);
      e1000_write_phy_reg(hw, 29, 0x001A);
      e1000_write_phy_reg(hw, 30, 0x8FF0);

      /****************************************************************
       * This delay is necessary with some nics on some machines after
       * the PHY receiver is disabled.
       ***************************************************************/
      usec_delay(500);

      e1000_read_phy_reg(hw, M88E1000_PHY_SPEC_CTRL,     &PhyReg);
      e1000_read_phy_reg(hw, M88E1000_EXT_PHY_SPEC_CTRL, &PhyReg);
    }
    //
    // The following delay is necessary for the PHY loopback mode to take on ESB2 based LOMs
    //
    msec_delay(100);

    return LoopbackModeSet;
}

#ifndef  NO_82571_SUPPORT
VOID
_SetI82571SerdesLoopback(
    struct e1000_hw  *hw
    )
/*++
  Description: This routine is used to set fiber and serdes based 82571
               and 82575 adapters into loopback mode.

  Arguments:    hw - Ptr to this card's adapter data structure

  Returns:      TRUE - Success, FALSE - Failure
--*/

{
  UINT32    CtrlReg         = 0;
  UINT32    TxctlReg        = 0;
  UINT32    StatusReg       = 0;
  BOOLEAN   LinkUp          = FALSE;

  DEBUGPRINT(DIAG, ("Setting PHY loopback on I82571 fiber/serdes.\n"));
  /* I82571 transceiver loopback */
  CtrlReg = E1000_READ_REG (hw, E1000_CTRL);
  CtrlReg |= E1000_CTRL_SLU;
  E1000_WRITE_REG (hw, E1000_CTRL, CtrlReg);

  /* Disable autoneg by setting bit 31 of TXCW to zero */
  TxctlReg = E1000_READ_REG (hw, E1000_TXCW);
  TxctlReg &= ~(1 << 31);
  E1000_WRITE_REG (hw, E1000_TXCW, TxctlReg);

  /* Read status register link up */
  StatusReg = E1000_READ_REG (hw, E1000_STATUS);
  LinkUp = ((StatusReg & E1000_STATUS_LU) == 0) ? FALSE : TRUE;

  /* Set ILOS if link is not up */
  if(LinkUp == FALSE)
  {
    /* Set bit 7 (Invert Loss) and set link up in bit 6. */
    CtrlReg = E1000_READ_REG (hw, E1000_CTRL);
    CtrlReg |= (E1000_CTRL_ILOS);
    E1000_WRITE_REG (hw, E1000_CTRL, CtrlReg);
  }

  /* Write 0x410 to Serdes control register to enable SerDes analog loopback */
  E1000_WRITE_REG (hw, E1000_SCTL, 0x0410);
  msec_delay(10);
}
#endif

#ifndef NO_82575_SUPPORT
VOID
_SetI82575SerdesLoopback(
    struct e1000_hw  *hw
    )
/*++
  Description: This routine is used to set fiber and serdes based 82571
               and 82575 adapters into loopback mode.

  Arguments:    hw - Ptr to this card's adapter data structure

  Returns:      TRUE - Success, FALSE - Failure
--*/

{
  UINT32    CtrlReg         = 0;
  UINT32    CtrlExtReg      = 0;
  UINT32    PcsLctl         = 0;
  UINT32    ConnSwReg       = 0;

  DEBUGPRINT(DIAG, ("Setting PHY loopback on I82575 fiber/serdes.\n"));

  /* I82575 transceiver loopback per EAS */
  /* Write 0x410 to Serdes control register to enable SerDes analog loopback */
  E1000_WRITE_REG (hw, E1000_SCTL, 0x0410);
  msec_delay(10);

  CtrlExtReg = E1000_READ_REG (hw, E1000_CTRL_EXT);
  CtrlExtReg |= E1000_CTRL_EXT_LINK_MODE_PCIE_SERDES;
  E1000_WRITE_REG (hw, E1000_CTRL_EXT, CtrlExtReg);
  msec_delay(10);

  CtrlReg = E1000_READ_REG (hw, E1000_CTRL);
  CtrlReg |= E1000_CTRL_SLU | E1000_CTRL_FD;
  CtrlReg &= ~(E1000_CTRL_RFCE | E1000_CTRL_TFCE | E1000_CTRL_LRST);
  E1000_WRITE_REG (hw, E1000_CTRL, CtrlReg);
  msec_delay(10);

  PcsLctl = E1000_READ_REG(hw, E1000_PCS_LCTL);
  PcsLctl |= E1000_PCS_LCTL_FORCE_LINK | E1000_PCS_LCTL_FSD
    | E1000_PCS_LCTL_FDV_FULL| E1000_PCS_LCTL_FLV_LINK_UP;
  PcsLctl &= ~E1000_PCS_LCTL_AN_ENABLE;
  E1000_WRITE_REG (hw, E1000_PCS_LCTL, PcsLctl);
  msec_delay(10);

  /* Read status register link up */
  ConnSwReg = E1000_READ_REG (hw, E1000_CONNSW);
  ConnSwReg &= ~E1000_CONNSW_ENRGSRC;
  E1000_WRITE_REG (hw, E1000_CONNSW, ConnSwReg);
  msec_delay(10);

}
#endif

VOID
_SetI82580SerdesLoopback(
    struct e1000_hw  *hw
    )
/*++
  Description: This routine is used to set fiber and serdes based 82571
               and 82575 adapters into loopback mode.

  Arguments:    hw - Ptr to this card's adapter data structure

  Returns:      TRUE - Success, FALSE - Failure
--*/

{
  UINT32    RctlReg         = 0;
  UINT32    CtrlReg         = 0;
  UINT32    PcsLctl         = 0;
  UINT32    ConnSwReg       = 0;

  DEBUGPRINT(DIAG, ("Setting PHY loopback on 82580 fiber/serdes.\n"));

  /* Write 0x410 to Serdes control register to enable SerDes analog loopback */
  E1000_WRITE_REG (hw, E1000_SCTL, 0x0410);

  /* Configure SerDes to loopback */
  RctlReg = E1000_READ_REG(hw, E1000_RCTL);
  RctlReg |= E1000_RCTL_LBM_TCVR;
  E1000_WRITE_REG(hw, E1000_RCTL, RctlReg);

  /* Move to Force mode */
  CtrlReg = E1000_READ_REG(hw, E1000_CTRL);
  CtrlReg |= E1000_CTRL_SLU | E1000_CTRL_FD;
  CtrlReg &= ~(E1000_CTRL_RFCE | E1000_CTRL_TFCE);
  E1000_WRITE_REG(hw, E1000_CTRL, CtrlReg);

  PcsLctl = E1000_READ_REG(hw, E1000_PCS_LCTL);
  PcsLctl |= E1000_PCS_LCTL_FORCE_LINK | E1000_PCS_LCTL_FSD
    | E1000_PCS_LCTL_FDV_FULL| E1000_PCS_LCTL_FLV_LINK_UP;
  PcsLctl &= ~E1000_PCS_LCTL_AN_ENABLE;
  E1000_WRITE_REG(hw, E1000_PCS_LCTL, PcsLctl);

  ConnSwReg = E1000_READ_REG (hw, E1000_CONNSW);
  ConnSwReg &= ~E1000_CONNSW_ENRGSRC;
  E1000_WRITE_REG (hw, E1000_CONNSW, ConnSwReg);

  msec_delay(10);   // Need this delay or SerDes loopback will fail.
}

/**********************************************************************
** Procedure:    _NalI8254xSetBoazmanPhyLoopback
**
** Description:  This routine is used by diagnostic software to put
**               the Intel Gigabit PHY into loopback mode.
**
**               Loopback speed is determined by the Speed value
**               passed into this routine.
**
**               Valid values are 1000, 100, and 10 Mbps
**
** Author:       MVM
**
** Born on Date: 04/02/2007
**
** Arguments:    Adapter - Ptr to this card's adapter data structure
**               Speed   - desired loopback speed
**
** Returns:      TRUE - Success, FALSE - Failure
**********************************************************************/
BOOLEAN
_SetBoazmanPhyLoopback(
    IN  struct e1000_hw*          hw,
    IN  UINT16             Speed
    )
{
    UINT16              PhyValue        = 0;
    BOOLEAN             LoopbackModeSet = FALSE;

#ifndef NO_82571_SUPPORT
#ifndef NO_82574_SUPPORT
    UINT32              Reg             = 0;
    /* 82574 requires ILOS set */
    if(hw->mac.type == e1000_82574
        || hw->mac.type == e1000_82583
        )
    {
        DEBUGPRINT(DIAG, ( "Setting ILOS on 82574.\n"));
        Reg = E1000_READ_REG(hw, E1000_CTRL);
        E1000_WRITE_REG(hw,  E1000_CTRL, Reg | E1000_CTRL_ILOS);
    }
#endif
#endif

    if(Speed == SPEED_1000)
    {
        DEBUGPRINT(DIAG, ( "Setting Boazman PHY into loopback at 1000 Mbps\n"));
        /* set 21_2.2:0 to the relevant speed (1G ?3b110, 100Mb ?3b101, 10Mb ? 3b100) */
        _ReadPhyRegister16Ex(hw, 2, 21, &PhyValue);
        PhyValue = (PhyValue & (~(7))) | 6;
    }
    else if(Speed == SPEED_100)
    {
        DEBUGPRINT(DIAG, ( "Setting Boazman PHY into loopback at 100 Mbps\n"));
        /* set 21_2.2:0 to the relevant speed (1G ?3b110, 100Mb ?3b101, 10Mb ? 3b100) */
        _ReadPhyRegister16Ex(hw, 2, 21, &PhyValue);
        PhyValue = (PhyValue & (~(7))) | 5;
    }
    else
    {
        DEBUGPRINT(DIAG, ( "Setting Boazman PHY into loopback at 10 Mbps\n"));
        /* set 21_2.2:0 to the relevant speed (1G ?3b110, 100Mb ?3b101, 10Mb ? 3b100) */
        _ReadPhyRegister16Ex(hw, 2, 21, &PhyValue);
        PhyValue = (PhyValue & (~(7))) | 4;
    }

    _WritePhyRegister16Ex(hw, 2, 21, PhyValue);

    /* assert sw reset (so settings will take effect). */
    e1000_read_phy_reg(hw, PHY_CONTROL, &PhyValue);
    e1000_write_phy_reg(hw, PHY_CONTROL, PhyValue | (1 << 15));
    msec_delay(1);

#ifndef NO_82571_SUPPORT
#ifndef NO_82574_SUPPORT
    /* ICH9 and ICH10 version requires all these undocumented writes */
    if (hw->mac.type != e1000_82574
        || hw->mac.type != e1000_82583
        )
    {
        /* force duplex to FD: 16_769.3:2=3. */
        _ReadPhyRegister16Ex(hw, 769, 16, &PhyValue);
        PhyValue |= (3 << 2);
        _WritePhyRegister16Ex(hw, 769, 16, PhyValue);

        /* set 16_776.6= state (link up when in force link) */
        _ReadPhyRegister16Ex(hw, 776, 16, &PhyValue);
        PhyValue |= (1 << 6);
        _WritePhyRegister16Ex(hw, 776, 16, PhyValue);

        /* set 16_769.6= state (force link) */
        _ReadPhyRegister16Ex(hw, 769, 16, &PhyValue);
        PhyValue |= (1 << 6);
        _WritePhyRegister16Ex(hw, 769, 16, PhyValue);

        /* Set Early Link Enable - 20_769.10 = 1 */
        _ReadPhyRegister16Ex(hw, 769, 20, &PhyValue);
        PhyValue |= (1 << 10);
        _WritePhyRegister16Ex(hw, 769, 20, PhyValue);
    }
#endif
#endif

    LoopbackModeSet = _SetIgpPhyLoopback(hw, Speed);

    return LoopbackModeSet;
}


BOOLEAN
_SetNinevehPhyLoopback(
    struct e1000_hw  *hw,
    IN  UINT16        Speed
    )
/*++
  Description: This routine is used by diagnostic software to put
               the Intel Gigabit PHY into loopback mode.

               Loopback speed is determined by the Speed value
               passed into this routine.

               Valid values are 1000, 100, and 10 Mbps

  Arguments:    Adapter - Ptr to this card's adapter data structure
                Speed   - desired loopback speed

  Returns:      TRUE - Success, FALSE - Failure
--*/
{
    UINT32              StatusReg       = 0;
    UINT32              DelayValue      = 10;
    UINT32              DelayMax        = 5000;
    UINT32              i               = 0;
    BOOLEAN             LoopbackModeSet = FALSE;

    if(Speed == SPEED_1000)
    {
        DEBUGPRINT(DIAG, ("Setting Nineveh PHY into loopback at 1000 Mbps\n"));
        e1000_write_phy_reg(hw, PHY_CONTROL, 0x4140); /* force 1000, set loopback */
        LoopbackModeSet = TRUE;
    }
    else if(Speed == SPEED_100)
    {
        DEBUGPRINT(DIAG, ( "Setting Nineveh PHY into loopback at 100 Mbps\n"));
        e1000_write_phy_reg(hw, PHY_CONTROL, 0x6100); /* force 100, set loopback */
    }
    else
    {
        LoopbackModeSet = _SetIgpPhyLoopback(hw, Speed);
    }

    /* Poll for link to be stable */
    for(i = 0; i < DelayMax; i += DelayValue)
    {
        msec_delay(DelayValue);
        StatusReg = E1000_READ_REG(hw, E1000_STATUS);
        if(StatusReg & (E1000_STATUS_LU | E1000_STATUS_FD))
        {
            DEBUGPRINT(DIAG, ("Nineveh link up indication after %d iterations\n", i));
            if(Speed == SPEED_1000)
            {
                if(StatusReg & E1000_STATUS_SPEED_1000)
                {
                    DEBUGPRINT(DIAG, ("Nineveh 1gb loopback link detected after %d iterations\n", i));
                    break;
                }
            }
            else if(Speed == SPEED_100)
            {
                if(StatusReg & E1000_STATUS_SPEED_100)
                {
                    DEBUGPRINT(DIAG, ("Nineveh 100mbit loopback link detected after %d iterations\n", i));
                    break;
                }
            }

            /* Don't bother reading the status register data for 10mbit. We force this up in
             * _SetIgpPhyLoopback */
            else
            {
                DEBUGPRINT(DIAG, ("Nineveh 10mbit loopback link detected after %d iterations\n", i));
                break;
            }
        }
    }

    return LoopbackModeSet;
}


BOOLEAN
_SetIgpPhyLoopback(
    IN  struct e1000_hw                 *hw,
    IN  UINT16                          Speed
    )
/*++
  Description:  This routine is used by diagnostic software to put
                the Intel Gigabit PHY into loopback mode.

                Loopback speed is determined by the Speed value
                passed into this routine.

                Valid values are 1000, 100, and 10 Mbps

  Arguments:    Adapter - Ptr to this card's adapter data structure
                Speed   - desired loopback speed

  Returns:      TRUE - Success, FALSE - Failure
--*/
{
    UINT32              CtrlReg         = 0;
    UINT32              StatusReg       = 0;
    UINT16              PhyReg          = 0;
    BOOLEAN             LoopbackModeSet = FALSE;

    hw->mac.autoneg = FALSE;

    /*******************************************************************
    ** Set up desired loopback speed and duplex depending on input
    ** into this function.
    *******************************************************************/
    switch(Speed)
    {
    case SPEED_1000:
        DEBUGPRINT(DIAG, ("Setting IGP01E1000 PHY into loopback at 1000 Mbps\n"));

        /****************************************************************
        ** Set up the MII control reg to the desired loopback speed.
        ****************************************************************/
        e1000_write_phy_reg(hw, PHY_CONTROL, 0x4140); /* force 1000, set loopback */
        msec_delay(250);

        /****************************************************************
        ** Now set up the MAC to the same speed/duplex as the PHY.
        ****************************************************************/
        CtrlReg = E1000_READ_REG(hw, E1000_CTRL);
        CtrlReg &= ~E1000_CTRL_SPD_SEL;   /* Clear the Speed selection bits */
        CtrlReg |= (E1000_CTRL_FRCSPD   | /* Set the Force Speed Bit */
                    E1000_CTRL_FRCDPX   | /* Set the Force Duplex Bit */
                    E1000_CTRL_SPD_1000 | /* Force Speed to 1000 */
                    E1000_CTRL_FD);       /* Force Duplex to FULL */

        if (hw->phy.media_type != e1000_media_type_copper)
        {
            /* Set the ILOS bits on the fiber nic if half duplex link is detected. */
            StatusReg = E1000_READ_REG(hw, E1000_STATUS);
            if((StatusReg & E1000_STATUS_FD) == 0)
            {
                CtrlReg |= (E1000_CTRL_ILOS | E1000_CTRL_SLU);  /* Invert Loss-Of-Signal */
            }
        }

        E1000_WRITE_REG (hw, E1000_CTRL, CtrlReg);
        LoopbackModeSet = TRUE;
        break;

    case SPEED_100:
        DEBUGPRINT(DIAG, ("Setting IGP01E1000 PHY into loopback at 100 Mbps\n"));
        /****************************************************************
        ** Set up the MII control reg to the desired loopback speed.
        ****************************************************************/
        e1000_write_phy_reg(hw, PHY_CONTROL, 0x6100); /* force 100, set loopback */

        /****************************************************************
        ** Now set up the MAC to the same speed/duplex as the PHY.
        ****************************************************************/
        CtrlReg = E1000_READ_REG(hw, E1000_CTRL);
        CtrlReg &= ~E1000_CTRL_SPD_SEL;   /* Clear the Speed selection bits */
        CtrlReg &= ~E1000_CTRL_SPD_SEL;   /* Clear the Speed selection bits */
        CtrlReg |= (E1000_CTRL_FRCSPD  |  /* Set the Force Speed Bit */
                    E1000_CTRL_FRCDPX  |  /* Set the Force Duplex Bit */
                    E1000_CTRL_SPD_100 |  /* Force Speed to 100 */
                    E1000_CTRL_FD);       /* Force Duplex to FULL */

        if (hw->phy.media_type != e1000_media_type_copper)
        {
            /* Set the ILOS bits on the fiber nic if half duplex link is */
            /* detected.                                                 */
            StatusReg = E1000_READ_REG(hw, E1000_STATUS);
            if((StatusReg & E1000_STATUS_FD) == 0)
            {
                CtrlReg |= (E1000_CTRL_ILOS | E1000_CTRL_SLU);  /* Invert Loss-Of-Signal */
            }
        }

        E1000_WRITE_REG (hw, E1000_CTRL, CtrlReg);
        LoopbackModeSet = TRUE;
        break;

    case SPEED_10:
        DEBUGPRINT(DIAG, ("Setting IGP01E1000 PHY into loopback at 10 Mbps\n"));
        /****************************************************************
        ** Set up the MII control reg to the desired loopback speed.
        ****************************************************************/
        e1000_write_phy_reg(hw, PHY_CONTROL, 0x4100); /* force 10, set loopback */
        /* For 10mbps loopback we need to assert the "Force link pass" bit in
           the Port Configuration register */
        e1000_read_phy_reg(hw, IGP01E1000_PHY_PORT_CONFIG, &PhyReg);
        PhyReg |= 0x4000;
        e1000_write_phy_reg(hw, IGP01E1000_PHY_PORT_CONFIG, PhyReg);

        /****************************************************************
        ** Now set up the MAC to the same speed/duplex as the PHY.
        ****************************************************************/
        CtrlReg = E1000_READ_REG(hw, E1000_CTRL);
        CtrlReg &= ~E1000_CTRL_SPD_SEL;   /* Clear the Speed selection bits */
        CtrlReg |= (E1000_CTRL_FRCSPD |   /* Set the Force Speed Bit */
                    E1000_CTRL_FRCDPX |   /* Set the Force Duplex Bit */
                    E1000_CTRL_SPD_10 |   /* Force Speed to 10 */
                    E1000_CTRL_FD);       /* Force Duplex to FULL */

        if (hw->phy.media_type != e1000_media_type_copper)
        {
            /* Set the ILOS bits on the fiber nic if half duplex link is */
            /* detected.                                                 */
            StatusReg = E1000_READ_REG(hw, E1000_STATUS);
            if((StatusReg & E1000_STATUS_FD) == 0)
            {
                CtrlReg |= (E1000_CTRL_ILOS | E1000_CTRL_SLU);  /* Invert Loss-Of-Signal */
            }
        }

        E1000_WRITE_REG (hw, E1000_CTRL, CtrlReg);
        LoopbackModeSet = TRUE;
        break;

    default:
        DEBUGPRINT(DIAG, ("Invalid speed value loopback mode \"%d\"\n", Speed));
        LoopbackModeSet = FALSE;
        break;
    }

    usec_delay(500);
    return LoopbackModeSet;
}

BOOLEAN
_SetPhyLoopback82580 (
  struct e1000_hw  *hw
  )
{
  UINT32  Reg;
  UINT16  PhyReg = 0;
  BOOLEAN LoopbackModeSet = FALSE;

  DEBUGPRINT(DIAG, ("_SetPhyLoopback82580\n"));

  /* Set Link Mode to Internal */
  Reg = E1000_READ_REG(hw, E1000_CTRL_EXT);
  DEBUGPRINT(DIAG, ("_SetPhyLoopback82580: E1000_CTRL_EXT = 0x%x\n", Reg));
  Reg = (~E1000_CTRL_EXT_LINK_MODE_MASK) & Reg;
  E1000_WRITE_REG(hw, E1000_CTRL_EXT, Reg);

  /* Disable PHY power management in case the cable is unplugged and the PHY is asleep */
  DEBUGPRINT(DIAG, ("PHPM = %08x\n", E1000_READ_REG(hw, 0x0E14)));
  Reg = E1000_READ_REG(hw, 0x0E14);
  Reg &= ~0x0005;     /* SPD_EN and LPLU */
  E1000_WRITE_REG(hw, 0x0E14, Reg);

  /* Set 1000 Mbps loopback mode in PHY */
  e1000_write_phy_reg(hw, PHY_CONTROL, 0x4140);

  /* Set 1000 Mbps mode in MAC */
  Reg = E1000_READ_REG(hw, E1000_CTRL);
  Reg &= ~E1000_CTRL_SPD_SEL;   		// Clear the Speed selection bits
  Reg |= (E1000_CTRL_FRCSPD  |  		// Set the Force Speed Bit
               E1000_CTRL_FRCDPX  |  	// Set the Force Duplex Bit
               E1000_CTRL_SPD_1000 |  	// Force Speed to 1000
               E1000_CTRL_FD);       	// Force Duplex to FULL
  E1000_WRITE_REG (hw, E1000_CTRL, Reg);


  /* Enable PHY loopback mode */
  e1000_read_phy_reg(hw, PHY_PHLBKC, &PhyReg);
  DEBUGPRINT(DIAG, ("PHY_PHLBKC = %04x\n", PhyReg));
  PhyReg = 0x8001;    /* MII and Force Link Status */
  e1000_write_phy_reg(hw, PHY_PHLBKC, PhyReg);

  e1000_read_phy_reg(hw, PHY_PHCTRL1, &PhyReg);
  DEBUGPRINT(DIAG, ("PHY_PHCTRL1 = %04x\n", PhyReg));
  PhyReg |= 0x2000;   /* LNK_EN */
  e1000_write_phy_reg(hw, PHY_PHCTRL1, PhyReg);

  msec_delay(500);

  LoopbackModeSet = TRUE;
  return LoopbackModeSet;
}

BOOLEAN
e1000_set_phy_loopback (
  struct e1000_hw  *hw,
  UINT16         speed
  )
/*++

Routine Description:

  Set the PHY into loopback mode.  This routine integrates any errata workarounds that might exist.

Arguments:

  hw    - Pointer to the shared code adapter structure
  speed - Select speed to perform loopback test

Returns:
   True if PHY has been configured for loopback mode
   False otherwise

--*/
{
  BOOLEANean_t status;

  DEBUGPRINT(DIAG, ("e1000_set_phy_loopback\n"));

  switch (hw->mac.type) {
#ifndef NO_82571_SUPPORT
  case e1000_82573:
    DEBUGPRINT(DIAG, ("Enabling M88E1000 loopback mode.\n"));
    status = _SetIntegratedM88PhyLoopback (hw, speed);
    break;
#endif
#ifndef NO_80003ES2LAN_SUPPORT
  case e1000_80003es2lan:
    DEBUGPRINT(DIAG, ("Enabling M88E1000 loopback mode.\n"));
    status = _SetIntegratedM88PhyLoopback (hw, speed);
    break;
#endif
#ifndef NO_82571_SUPPORT
    case e1000_82571:
    case e1000_82572:
     /* I82571 sets a special loopback mode through the SERDES register. This is only for Fiber
       * adapters and is used because MAC and PHY loopback are broken on these adapters */
      if(hw->phy.media_type != e1000_media_type_copper)
      {
        _SetI82571SerdesLoopback(hw);
        status = TRUE;
      }
      else
      {
        DEBUGPRINT(DIAG, ("I82571: Enabling IGP01E100 loopback mode.\n"));
        status = _SetIgpPhyLoopback (hw, speed);
      }
      break;
#endif

#ifndef NO_82571_SUPPORT
#ifndef NO_82574_SUPPORT
    case e1000_82574:
    case e1000_82583:
      DEBUGPRINT(DIAG, ("Enabling Boazman for 82574, 82583 loopback mode.\n"));
      status = _SetBoazmanPhyLoopback(hw, speed);
      break;
#endif
#endif

#ifndef NO_ICH8LAN_SUPPORT
    case e1000_ich9lan:
      DEBUGPRINT(DIAG, ("Enabling Boazman for ICH9 loopback mode.\n"));
      status = _SetBoazmanPhyLoopback(hw, speed);
      break;
    case e1000_pchlan:
    case e1000_pch2lan:
      DEBUGPRINT(DIAG, ("Enabling Hanksville for PCH loopback mode.\n"));
      status = _SetBoazmanPhyLoopback(hw, speed);
      break;
#endif

#ifndef NO_82575_SUPPORT
    case e1000_82575:
      if(hw->phy.media_type != e1000_media_type_copper)
      {
        _SetI82575SerdesLoopback(hw);
        status = TRUE;
      }
      else
      {
        DEBUGPRINT(DIAG, ("Enabling 82575, 82576 loopback\n"));
        status = _SetNinevehPhyLoopback (hw, speed);
      }
      break;
#endif
#ifndef NO_82576_SUPPORT
    case e1000_82576:
#endif
    case e1000_82580:
      if(hw->phy.media_type == e1000_media_type_copper && hw->dev_spec._82575.sgmii_active == FALSE)
      {
        DEBUGPRINT(DIAG, ("Enabling 82580 loopback for copper\n"));
        status = _SetPhyLoopback82580(hw);
      }
      else
      {
        DEBUGPRINT(DIAG, ("Enabling 82580 loopback for SerDes/SGMII/1000BASE-KX\n"));
        _SetI82580SerdesLoopback(hw);
        status = TRUE;
      }
      break;

    default:
      DEBUGPRINT(DIAG, ("Unknown MAC type.\n"));
      DEBUGWAIT(DIAG);
      status = FALSE;
      break;
  }


  return status;
}

EFI_STATUS
GigUndiRunPhyLoopback (
  GIG_DRIVER_DATA   *GigAdapterInfo,
  PXE_CPB_TRANSMIT  PxeCpbTransmit
  )
/*++

Routine Description:
  Run the PHY loopback test for N iterations.  This routine transmits a packet, waits a bit, and then
  checks to see if it was received.  If any of the packets are not received then it will be interpreted as
  a failure.

Arguments:
  GigAdapterInfo    - Pointer to the NIC data structure the PHY loopback test will be run on.
  PxeCpbTransmit    - Pointer to the packet to transmit.

Returns:
  EFI_SUCCESS       - All packets were received successfully
  other             - An error occured.

--*/
{
  PXE_CPB_RECEIVE CpbReceive;
  PXE_DB_RECEIVE  DbReceive;
  EFI_STATUS      Status;
  UINT64          FreeTxBuffer[DEFAULT_TX_DESCRIPTORS];
  UINT32          j;
  UINT32          i;

  Status  = EFI_SUCCESS;
  j       = 0;

  while (j < PHY_LOOPBACK_ITERATIONS) {
    Status = e1000_Transmit (
              GigAdapterInfo,
              (UINT64) &PxeCpbTransmit,
              PXE_OPFLAGS_TRANSMIT_WHOLE
              );
    _DisplayBuffersAndDescriptors (GigAdapterInfo);

    if (EFI_ERROR (Status)) {
      DEBUGPRINT (CRITICAL, ("e1000_Transmit error Status %X. Iteration=%d\n", Status, j));
      DEBUGWAIT (CRITICAL);
      break;
    }

    //
    // Wait a little, then check to see if the packet has arrived
    //
    Status = gBS->AllocatePool (
                    EfiBootServicesData,
                    RX_BUFFER_SIZE,
                    (VOID **) &CpbReceive.BufferAddr
                    );

    if (EFI_ERROR (Status)) {
      DEBUGPRINT (CRITICAL, ("AllocatePool error Status %X. Iteration=%d\n", Status, j));
      DEBUGWAIT (CRITICAL);
      break;
    }

    CpbReceive.BufferLen = RX_BUFFER_SIZE;
    i = 0;
    do {
      Status = e1000_Receive (
                 GigAdapterInfo,
                 (UINT64) &CpbReceive,
                 (UINT64) &DbReceive
                 );
      gBS->Stall (10);
      i++;
      if (i > 100000) {
        DEBUGPRINT(CRITICAL, ("ERROR: No receive data timeout! Iteration=%d\n", i));
        break;
      }
    } while (Status == PXE_STATCODE_NO_DATA);

    if (EFI_ERROR(Status)) {
      DEBUGPRINT(CRITICAL, ("e1000_Receive Status %X\n", Status));
      DEBUGWAIT(CRITICAL);
    }

    if (CompareMem ((VOID *) (UINTN) CpbReceive.BufferAddr, (VOID *) (UINTN) Packet, TEST_PACKET_SIZE) == 0) {
      Status = EFI_SUCCESS;
    } else {
      DEBUGPRINT (CRITICAL, ("PHY LOOPBACK FAILED, Corrupt Packet Data!\n"));
      Status = EFI_DEVICE_ERROR;
      break;
    }

    e1000_FreeTxBuffers (
      GigAdapterInfo,
      DEFAULT_TX_DESCRIPTORS,
      FreeTxBuffer
      );

    j++;
    gBS->FreePool ((VOID *) ((UINTN) CpbReceive.BufferAddr));
  }

  return Status;
}

EFI_STATUS
GigUndiPhyLoopback (
  GIG_UNDI_PRIVATE_DATA *GigUndiPrivateData
  )
/*++

  Routine Description:
                 Sets up the adapter to run the Phy loopback test and then calls
                 the loop which will iterate through the test.

 Arguments:      GigUndiPrivateData - Pointer to adapter data.

  Returns:
                 EFI_SUCCESS                - The Phy loopback test passed.
                 EFI_DEVICE_ERROR           - Phy loopback test failed
                 EFI_INVALID_PARAMETER      - Some other error occured.
--*/
{
  PXE_CPB_TRANSMIT      PxeCpbTransmit;
  UINT8                 ReceiveStarted;
  EFI_STATUS            Status;

  ReceiveStarted = GigUndiPrivateData->NicInfo.ReceiveStarted;
  GigUndiPrivateData->NicInfo.DriverBusy = TRUE;

  DEBUGPRINT (DIAG, ("GigUndiPrivateData->NicInfo.Block %X\n", (UINTN) GigUndiPrivateData->NicInfo.Block));
  DEBUGPRINT (DIAG, ("GigUndiPrivateData->NicInfo.MapMem %X\n", (UINTN) GigUndiPrivateData->NicInfo.MapMem));
  DEBUGPRINT (DIAG, ("GigUndiPrivateData->NicInfo.Delay %X\n", (UINTN) GigUndiPrivateData->NicInfo.Delay));
  DEBUGPRINT (DIAG, ("GigUndiPrivateData->NicInfo.MemIo %X\n", (UINTN) GigUndiPrivateData->NicInfo.MemIo));
  DEBUGWAIT (DIAG);

  //
  // Initialize and start the UNDI driver if it has not already been done
  //
  e1000_reset_hw(&GigUndiPrivateData->NicInfo.hw);
  GigUndiPrivateData->NicInfo.HwInitialized = FALSE;
  if (e1000_Inititialize (&GigUndiPrivateData->NicInfo) != PXE_STATCODE_SUCCESS) {
      DEBUGPRINT (CRITICAL, ("Error initializing adapter!\n"));
      Status = EFI_DEVICE_ERROR;
      goto error;
  }

  //
  // Put the PHY into loopback mode,
  //
  if (e1000_set_phy_loopback (&GigUndiPrivateData->NicInfo.hw, SPEED_1000)) {
    DEBUGPRINT (DIAG, ("PHY loopback mode set successful\n"));
  } else {
    DEBUGPRINT (CRITICAL, ("ERROR: PHY loopback not set!\n"));
    DEBUGWAIT (CRITICAL);
    Status =  EFI_UNSUPPORTED;
    goto error;
  }

  DEBUGWAIT (DIAG);

  //
  // Enable the receive unit
  //
  e1000_ReceiveEnable (&GigUndiPrivateData->NicInfo);

  //
  // Build our packet, and send it out the door.
  //
  DEBUGPRINT (DIAG, ("Building Packet\n"));
  _BuildPacket (&GigUndiPrivateData->NicInfo);

  PxeCpbTransmit.MediaheaderLen = sizeof (ETHERNET_HDR);
  PxeCpbTransmit.DataLen        = TEST_PACKET_SIZE - sizeof (ETHERNET_HDR);
  PxeCpbTransmit.FrameAddr      = (UINTN) Packet;
  PxeCpbTransmit.reserved       = 0;
  DEBUGPRINT (DIAG, ("Packet length = %d\n", PxeCpbTransmit.DataLen));
  DEBUGPRINT (DIAG, ("Packet = %X FrameAddr = %X\n", (UINTN) Packet, PxeCpbTransmit.FrameAddr));
  DEBUGPRINT (DIAG, ("Packet data:\n"));
//  for (i = 0; i < PxeCpbTransmit.DataLen; i++) {
//    DEBUGPRINT (DIAG, ("%d: %x ", i, ((UINT8 *) ((UINTN) PxeCpbTransmit.FrameAddr))[i]));
//  }

  DEBUGWAIT (DIAG);

  Status = GigUndiRunPhyLoopback (&GigUndiPrivateData->NicInfo, PxeCpbTransmit);
  DEBUGPRINT (DIAG, ("PHY Loopback test returns %r\n", Status));

  e1000_ReceiveDisable(&GigUndiPrivateData->NicInfo);

  DEBUGWAIT (DIAG);

error:
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("Error Status %X\n", Status));
    DEBUGWAIT (CRITICAL);
  }

  //
  // After PHY loopback test completes we need to perform a full reset of the adapter.
  // If the adapter was initialized on entry then force a full reset of the adapter.
  // Also reenable the receive unit if it was enabled before we started the PHY loopback test.
  //
  e1000_reset_hw(&GigUndiPrivateData->NicInfo.hw);
  GigUndiPrivateData->NicInfo.HwInitialized = FALSE;
  if (GigUndiPrivateData->NicInfo.State == PXE_STATFLAGS_GET_STATE_INITIALIZED) {
    e1000_Inititialize (&GigUndiPrivateData->NicInfo);
    DEBUGPRINT(DIAG, ("e1000_Inititialize complete\n"));
    //
    //  Restart the receive unit if it was running on entry
    //
    if (ReceiveStarted) {
      DEBUGPRINT(DIAG, ("RESTARTING RU\n"));
      e1000_ReceiveEnable(&GigUndiPrivateData->NicInfo);
    }
  }
  DEBUGPRINT(DIAG, ("ADAPTER RESET COMPLETE\n"));

  GigUndiPrivateData->NicInfo.DriverBusy = FALSE;

  return Status;
}

EFI_STATUS
GigUndiDriverDiagnosticsRunDiagnostics (
  IN EFI_DRIVER_DIAGNOSTICS_PROTOCOL             *This,
  IN EFI_HANDLE                                  ControllerHandle,
  IN EFI_HANDLE                                  ChildHandle OPTIONAL,
  IN EFI_DRIVER_DIAGNOSTIC_TYPE                  DiagnosticType,
  IN CHAR8                                       *Language,
  OUT EFI_GUID                                   **ErrorType,
  OUT UINTN                                      *BufferSize,
  OUT CHAR16                                     **Buffer
  )
/*++

  Routine Description:
    Runs diagnostics on a controller.

  Arguments:
    This             - A pointer to the EFI_DRIVER_DIAGNOSTICS_PROTOCOL instance.
    ControllerHandle - The handle of the controller to run diagnostics on.
    ChildHandle      - The handle of the child controller to run diagnostics on
                       This is an optional parameter that may be NULL.  It will
                       be NULL for device drivers.  It will also be NULL for a
                       bus drivers that wish to run diagnostics on the bus
                       controller.  It will not be NULL for a bus driver that
                       wishes to run diagnostics on one of its child controllers.
    DiagnosticType   - Indicates type of diagnostics to perform on the controller
                       specified by ControllerHandle and ChildHandle.   See
                       "Related Definitions" for the list of supported types.
    Language         - A pointer to a three character ISO 639-2 language
                       identifier.  This is the language in which the optional
                       error message should be returned in Buffer, and it must
                       match one of the languages specified in SupportedLanguages.
                       The number of languages supported by a driver is up to
                       the driver writer.
    ErrorType        - A GUID that defines the format of the data returned in
                       Buffer.
    BufferSize       - The size, in bytes, of the data returned in Buffer.
    Buffer           - A buffer that contains a Null-terminated Unicode string
                       plus some additional data whose format is defined by
                       ErrorType.  Buffer is allocated by this function with
                       AllocatePool(), and it is the caller's responsibility
                       to free it with a call to FreePool().

  Returns:
    EFI_SUCCESS           - The controller specified by ControllerHandle and
                            ChildHandle passed the diagnostic.
    EFI_INVALID_PARAMETER - ControllerHandle is not a valid EFI_HANDLE.
    EFI_INVALID_PARAMETER - ChildHandle is not NULL and it is not a valid
                            EFI_HANDLE.
    EFI_INVALID_PARAMETER - Language is NULL.
    EFI_INVALID_PARAMETER - ErrorType is NULL.
    EFI_INVALID_PARAMETER - BufferType is NULL.
    EFI_INVALID_PARAMETER - Buffer is NULL.
    EFI_UNSUPPORTED       - The driver specified by This does not support
                            running diagnostics for the controller specified
                            by ControllerHandle and ChildHandle.
    EFI_UNSUPPORTED       - The driver specified by This does not support the
                            type of diagnostic specified by DiagnosticType.
    EFI_UNSUPPORTED       - The driver specified by This does not support the
                            language specified by Language.
    EFI_OUT_OF_RESOURCES  - There are not enough resources available to complete
                            the diagnostics.
    EFI_OUT_OF_RESOURCES  - There are not enough resources available to return
                            the status information in ErrorType, BufferSize,
                            and Buffer.
    EFI_DEVICE_ERROR      - The controller specified by ControllerHandle and
                            ChildHandle did not pass the diagnostic.

--*/
{
  EFI_DEVICE_PATH_PROTOCOL                  *UndiDevicePath;
  GIG_UNDI_PRIVATE_DATA                     *GigUndiPrivateData;
  EFI_PRO1000_COM_PROTOCOL                  *EfiPro1000Com;
  EFI_STATUS                                Status;

  Status              = EFI_SUCCESS;
  GigUndiPrivateData  = NULL;

  if (DiagnosticType == EfiDriverDiagnosticTypeManufacturing) {
    DEBUGPRINT (CRITICAL, ("DiagnosticType == EfiDriverDiagnosticTypeManufacturing\n"));
    DEBUGWAIT (CRITICAL);
    return EFI_UNSUPPORTED;
  }

  //
  // Make sure this driver is currently managing ControllerHandle
  //
  Status = gBS->OpenProtocol (
                  ControllerHandle,
                  &gEfiDevicePathProtocolGuid,
                  (VOID **) &UndiDevicePath,
                  mGigUndiDriverBinding.DriverBindingHandle,
                  ControllerHandle,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );

  DEBUGPRINT (DIAG, (" OpenProtocol Status = %8X\n", Status));

  if (Status != EFI_ALREADY_STARTED) {
    DEBUGPRINT (CRITICAL, ("EFI_ALREADY_STARTED\n"));
    DEBUGWAIT (CRITICAL);
    gBS->CloseProtocol (
          ControllerHandle,
          &gEfiDevicePathProtocolGuid,
          mGigUndiDriverBinding.DriverBindingHandle,
          ControllerHandle
          );
    return EFI_UNSUPPORTED;
  }

  //
  //  Open an instance for the Network Interface Identifier Protocol so we can check
  // if the child handle interface is actually valid.
  //
  DEBUGPRINT (DIAG, ("Open an instance for the Network Interface Identifier Protocol\n"));
  Status = gBS->OpenProtocol (
                  ControllerHandle,
                  &gEfiPro1000ComGuid,
                  (VOID **) &EfiPro1000Com,
                  mGigUndiDriverBinding.DriverBindingHandle,
                  ControllerHandle,
                  EFI_OPEN_PROTOCOL_GET_PROTOCOL
                  );

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("OpenProtocol error Status %X\n", Status));
    DEBUGWAIT (CRITICAL);
    return Status;
  }

  GigUndiPrivateData = GIG_UNDI_PRIVATE_DATA_FROM_THIS (EfiPro1000Com->NIIProtocol_31);


  switch (DiagnosticType) {
    case EfiDriverDiagnosticTypeStandard:
      if  (e1000_validate_nvm_checksum(&GigUndiPrivateData->NicInfo.hw) == 0) {
        Status = EFI_SUCCESS;
      } else {
        DEBUGPRINT (CRITICAL, ("e1000_validate_nvm_checksum error!\n"));
        DEBUGWAIT (CRITICAL);
        Status = EFI_DEVICE_ERROR;
      }
      break;
    case EfiDriverDiagnosticTypeExtended:
      Status = GigUndiPhyLoopback(GigUndiPrivateData);
      if (EFI_ERROR (Status)) {
        DEBUGPRINT (CRITICAL, ("GigUndiPhyLoopback error Status %X\n", Status));
        DEBUGWAIT (CRITICAL);
      }
      break;
    default:
      DEBUGPRINT (CRITICAL, ("DiagnosticType unsupported!\n"));
      DEBUGWAIT (CRITICAL);
      Status = EFI_UNSUPPORTED;
  }

  return Status;
}
