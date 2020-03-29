 /** @file
  Implement the interface to the AX88179 Ethernet controller.

  This module implements the interface to the ASIX AX88179
  USB to Ethernet MAC with integrated 10/100 PHY.  Note that this implementation
  only supports the integrated PHY since no other test cases were available.

  Copyright (c) 2011, Intel Corporation
  All rights reserved. This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include "Ax88179.h"


/**
  Compute the CRC

  @param [in] pMacAddress      Address of a six byte buffer to containing the MAC address.

  @returns The CRC-32 value associated with this MAC address

**/
UINT32
Ax88179Crc (
  IN UINT8 * pMacAddress
  )
{
  UINT32 BitNumber;
  INT32 Carry;
  INT32 Crc;
  UINT32 Data;
  UINT8 * pEnd;

  //
  //  Walk the MAC address
  //
  Crc = -1;
  pEnd = &pMacAddress[ PXE_HWADDR_LEN_ETHER ];
  while ( pEnd > pMacAddress ) {
    Data = *pMacAddress++;
    
    
    //
    //  CRC32: x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1
    //
    //          1 0000 0100 1100 0001 0001 1101 1011 0111
    //
    for ( BitNumber = 0; 8 > BitNumber; BitNumber++ ) {
      Carry = (( Crc >> 31 ) & 1 ) ^ ( Data & 1 );
      Crc <<= 1;
      if ( 0 != Carry ) {
        Crc ^= 0x04c11db7;
      }
      Data >>= 1;
    }
  }

  //
  //  Return the CRC value
  //
  return (UINT32) Crc;
}

/**
  Get the MAC address

  This routine calls ::Ax88179UsbCommand to request the MAC
  address from the network adapter.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [out] pMacAddress      Address of a six byte buffer to receive the MAC address.

  @retval EFI_SUCCESS          The MAC address is available.
  @retval other                The MAC address is not valid.

**/
EFI_STATUS
Ax88179MacAddressGet (
  IN NIC_DEVICE * pNicDevice,
  OUT UINT8 * pMacAddress
  )
{
  EFI_STATUS Status;
  
  Status  = Ax88179MacRead (NODE_ID, 
                            PXE_HWADDR_LEN_ETHER,
                            pNicDevice,
                            pMacAddress);

  return Status;
}

/**
  Set the MAC address

  This routine calls ::Ax88179UsbCommand to set the MAC address
  in the network adapter.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in] pMacAddress      Address of a six byte buffer to containing the new MAC address.

  @retval EFI_SUCCESS          The MAC address was set.
  @retval other                The MAC address was not set.

**/
EFI_STATUS
Ax88179MacAddressSet (
  IN NIC_DEVICE * pNicDevice,
  OUT UINT8 * pMacAddress
  )
{
  EFI_STATUS Status;
  
  Status  = Ax88179MacWrite (NODE_ID, 
                             PXE_HWADDR_LEN_ETHER,
                             pNicDevice,
                             pMacAddress);

  return Status;
}

/**
  Clear the multicast hash table

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure

**/
VOID
Ax88179MulticastClear (
  IN NIC_DEVICE * pNicDevice
  )
{
  int i = 0;
  //
  // Clear the multicast hash table
  //

  for ( i = 0 ; i < 8 ; i++ )
      pNicDevice->MulticastHash[i] = 0;
}
/**
  Enable a multicast address in the multicast hash table

  This routine calls ::Ax88179Crc to compute the hash bit for
  this MAC address.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in] pMacAddress      Address of a six byte buffer to containing the MAC address.

**/
VOID
Ax88179MulticastSet (
  IN NIC_DEVICE * pNicDevice,
  IN UINT8 * pMacAddress
  )
{
  UINT32 Crc;
  //
  // Compute the CRC on the destination address
  //
  Crc = Ax88179Crc (pMacAddress) >> 26;
  //
  //  Set the bit corresponding to the destination address
  //
  pNicDevice->MulticastHash [ Crc >> 3 ] |= ( 1<< (Crc& 7));
}

/**
  Start the link negotiation

  This routine calls ::Ax88179PhyWrite to start the PHY's link
  negotiation.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure

  @retval EFI_SUCCESS          The link negotiation was started.
  @retval other                Failed to start the link negotiation.

**/
EFI_STATUS
Ax88179NegotiateLinkStart (
  IN NIC_DEVICE * pNicDevice
  )
{
  UINT16 Control = 0;
  EFI_STATUS Status;
  
#if FORCE_100Mbps
  Ax88179PhyRead ( pNicDevice,
                   0x09,
                   &Control );
  Control &= 0xFCFF;
  Ax88179PhyWrite ( pNicDevice, 0x09, Control );
#endif

  //
  // Set the link speed and duplex
  //
  Control = BMCR_AUTONEGOTIATION_ENABLE
          | BMCR_RESTART_AUTONEGOTIATION;
  if ( pNicDevice->b1000Mbps ) {
    Control |= BMCR_1000MBPS;
  } else if ( pNicDevice->b100Mbps ) {
    Control |= BMCR_100MBPS;
  }
  
  if ( pNicDevice->bFullDuplex ) {
    Control |= BMCR_FULL_DUPLEX;
  }
  Status = Ax88179PhyWrite ( pNicDevice, PHY_BMCR, Control );
  if (!EFI_ERROR(Status))
    gBS->Stall(3000000);
  //
  // Return the operation status
  //
  return Status;
}

/**
  Complete the negotiation of the PHY link

  This routine calls ::Ax88179PhyRead to determine if the
  link negotiation is complete.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in, out] pPollCount  Address of number of times this routine was polled
  @param [out] pbComplete      Address of boolean to receive complate status.
  @param [out] pbLinkUp        Address of boolean to receive link status, TRUE=up.
  @param [out] pbHiSpeed       Address of boolean to receive link speed, TRUE=100Mbps.
  @param [out] pbFullDuplex    Address of boolean to receive link duplex, TRUE=full.

  @retval EFI_SUCCESS          The MAC address is available.
  @retval other                The MAC address is not valid.

**/
EFI_STATUS
Ax88179NegotiateLinkComplete (
  IN NIC_DEVICE * pNicDevice,
  IN OUT UINTN * pPollCount,
  OUT BOOLEAN * pbComplete,
  OUT BOOLEAN * pbLinkUp,
  OUT BOOLEAN * pbHiSpeed,
  OUT BOOLEAN * pbGigaSpeed,
  OUT BOOLEAN * pbFullDuplex
  )
{
  UINT16 PhyData;
  EFI_STATUS  Status;

  //
  //  Determine if the link is up.
  //
  *pbComplete = FALSE;

  //
  //  Get the link status
  // 
  Status = Ax88179PhyRead ( pNicDevice,
                            PHY_PHYSR,
                            &PhyData );

  if ( !EFI_ERROR ( Status )) {
    *pbLinkUp =  (BOOLEAN)( 0 != ( PhyData & PHYSR_LINK ));
    if (*pbLinkUp) {
      //
      //  Determine if the autonegotiation is complete.
      //
      Status = Ax88179PhyRead ( pNicDevice,
                            PHY_BMSR,
                            &PhyData );
      if ( !EFI_ERROR (Status)) {
          *pbComplete =  (BOOLEAN)( 0 != ( PhyData & BMSR_AUTONEG_CMPLT ));
          
          if ( 0 != *pbComplete ) {
            //
            //  Get the partners capabilities.
            //
            Status = Ax88179PhyRead ( pNicDevice,
                                      PHY_PHYSR,
                                      &PhyData );
            if ( !EFI_ERROR ( Status )) {
              //
              //  Autonegotiation is complete
              //  Determine the link speed.
              //
              *pbGigaSpeed = (BOOLEAN) (( PhyData & PHYSR_SPEED_MASK ) == PHYSR_1000 );
              *pbHiSpeed = (BOOLEAN)(( PhyData & PHYSR_SPEED_MASK ) == PHYSR_100 );

              //
              //  Determine the link duplex.
              //
              *pbFullDuplex = (BOOLEAN)( (PhyData & PHYSR_FULLDUP) == PHYSR_FULLDUP);
          }
        }
      }          
    }    
  }    
        
     
  //
  // Return the operation status
  //
  return Status;
}


/**
  Read a register from the PHY

  This routine calls ::Ax88179UsbCommand to read a PHY register.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in] RegisterAddress  Number of the register to read.
  @param [in, out] pPhyData    Address of a buffer to receive the PHY register value

  @retval EFI_SUCCESS          The PHY data is available.
  @retval other                The PHY data is not valid.

**/
EFI_STATUS
Ax88179PhyRead (
  IN NIC_DEVICE * pNicDevice,
  IN UINT8 RegisterAddress,
  IN OUT UINT16 * pPhyData
  )
{
  USB_DEVICE_REQUEST SetupMsg;
  EFI_STATUS Status;

  //
  //  Read the PHY register address.
  //
  SetupMsg.RequestType = USB_ENDPOINT_DIR_IN
                       | USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
  SetupMsg.Request = ACCESS_PHY;
  SetupMsg.Value = pNicDevice->PhyId;
  SetupMsg.Index = RegisterAddress;
  SetupMsg.Length = sizeof ( *pPhyData );
  Status = Ax88179UsbCommand ( pNicDevice,
                               &SetupMsg,
                               pPhyData );

  //
  //  Return the operation status.
  //
  return Status;
}


/**
  Write to a PHY register

  This routine calls ::Ax88179UsbCommand to write a PHY register.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in] RegisterAddress  Number of the register to read.
  @param [in] PhyData          Address of a buffer to receive the PHY register value

  @retval EFI_SUCCESS          The PHY data was written.
  @retval other                Failed to wwrite the PHY register.

**/
EFI_STATUS
Ax88179PhyWrite (
  IN NIC_DEVICE * pNicDevice,
  IN UINT8 RegisterAddress,
  IN UINT16 PhyData
  )
{
  USB_DEVICE_REQUEST SetupMsg;
  EFI_STATUS Status;

  //
  //  Write the PHY register
  //
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
  SetupMsg.Request = ACCESS_PHY;
  SetupMsg.Value = pNicDevice->PhyId;
  SetupMsg.Index = RegisterAddress;
  SetupMsg.Length = sizeof ( PhyData );
  Status = Ax88179UsbCommand ( pNicDevice,
                               &SetupMsg,
                               &PhyData );
  //
  //  Return the operation status.
  //
  return Status;
}


/**
  Reset the AX88179

  This routine uses ::Ax88179UsbCommand to reset the network
  adapter.  This routine also uses ::Ax88179PhyWrite to reset
  the PHY.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure

  @retval EFI_SUCCESS          The MAC address is available.
  @retval other                The MAC address is not valid.

**/
EFI_STATUS
Ax88179Reset (
  IN NIC_DEVICE * pNicDevice
  )
{
  EFI_STATUS Status;
  UINT16  val;
  UINT8 val8;

  Status = Ax88179SetIInInterval(pNicDevice, 0xff);
  
  if (EFI_ERROR(Status)) goto err;
  
  val8 = 0;
  Status  = Ax88179MacRead (PLSR, 
                            sizeof(val8),
                            pNicDevice,
                            &val8);
							
  if (EFI_ERROR(Status)) goto err;
  
  if (val8 & PLSR_USB_SS)
    pNicDevice->usbMaxPktSize = 1024;
  else
    pNicDevice->usbMaxPktSize = 512;
 
  val = 0;                                
  Status = Ax88179MacWrite ( PHYPWRRSTCTL,
                             sizeof (val),
                             pNicDevice,
                             &val);
                              
  if (EFI_ERROR(Status)) goto err; 
                              
  gBS->Stall ( 10000 );
  
  val =  PHYPWRRSTCTL_IPRL;                
  Status = Ax88179MacWrite ( PHYPWRRSTCTL,
                             sizeof (val),
                             pNicDevice,
                             &val);
  
  if (EFI_ERROR(Status)) goto err;  
                                
  gBS->Stall ( 200000 );
                                  
  val = CLKSELECT_BCS | CLKSELECT_ACS;
  Status = Ax88179MacWrite ( CLKSELECT,
                             1,
                             pNicDevice,
                             &val);
  
  if (EFI_ERROR(Status)) goto err; 
     
  gBS->Stall ( 100000 );
  
  val = 0x52;
  Status = Ax88179MacWrite ( PAUSE_WATERLVL_HIGH,
                             1,
                             pNicDevice,
                             &val);
  
  if (EFI_ERROR(Status)) goto err;
  
  val = 0x34;
  Status = Ax88179MacWrite ( PAUSE_WATERLVL_LOW,
                             1,
                             pNicDevice,
                             &val);
  
  if (EFI_ERROR(Status)) goto err;
                        
  val = RXBINQCTRL_TIMEN | RXBINQCTRL_IFGEN | RXBINQCTRL_SIZEN;            
  
  Status =  Ax88179MacWrite ( RXBINQCTRL,
                              0x01,
                              pNicDevice,
                              &val);
  
  if (EFI_ERROR(Status)) goto err;
  
  val = 0;
  Status =  Ax88179MacWrite ( RXBINQTIMERL,
                              0x01,
                              pNicDevice,
                              &val);
                       
  if (EFI_ERROR(Status)) goto err;  
  
  val = 0;
  Status =  Ax88179MacWrite ( RXBINQTIMERH,
                              0x01,
                              pNicDevice,
                              &val);
                                    
  if (EFI_ERROR(Status)) goto err;  
  
  val = 12; //AX88179_BULKIN_SIZE_INK - 1;           
  Status =  Ax88179MacWrite ( RXBINQSIZE,
                              0x01,
                              pNicDevice,
                              &val);
                                    
  if (EFI_ERROR(Status)) goto err;  
  
  val = 0x0F;           
  Status =  Ax88179MacWrite ( RXBINQIFG,
                              0x01,
                              pNicDevice,
                              &val);

err:                     
  return Status;
}

EFI_STATUS
Ax88179RxControl (
  IN NIC_DEVICE * pNicDevice,
  IN UINT32 RxFilter
  )
{
  UINT16 MediumStatus;
  UINT16 RxControl = 0;
  EFI_STATUS Status = EFI_SUCCESS;
  //
  // Enable the receiver if something is to be received
  //
  if ( 0 != RxFilter ) {
    //
    //  Enable the receiver
    //
    Status  = Ax88179MacRead (MEDIUMSTSMOD, 
                              sizeof ( MediumStatus ),
                              pNicDevice,
                              &MediumStatus);
    
    if ( !EFI_ERROR ( Status ) && pNicDevice->CurMediumStatus != MediumStatus) {
        MediumStatus = MEDIUMSTSMOD_RE | MEDIUMSTSMOD_ONE;
        if ( pNicDevice->bFullDuplex ) {
          MediumStatus |= MEDIUMSTSMOD_TFC | MEDIUMSTSMOD_RFC | MEDIUMSTSMOD_FD;
        } else {
          MediumStatus &= ~(MEDIUMSTSMOD_TFC | MEDIUMSTSMOD_RFC | MEDIUMSTSMOD_FD);
        }
        if ( pNicDevice->b1000Mbps ) {
          MediumStatus |= MEDIUMSTSMOD_GM;
          MediumStatus |= MEDIUMSTSMOD_ENCK;
          MediumStatus &= ~MEDIUMSTSMOD_PS;
        } else {
          MediumStatus &= ~MEDIUMSTSMOD_GM;
          MediumStatus &= ~MEDIUMSTSMOD_ENCK;
          if ( pNicDevice->b100Mbps ) {
            MediumStatus |= MEDIUMSTSMOD_PS;         
          }  else {
            MediumStatus &= ~MEDIUMSTSMOD_PS;
          }
        }                
        Status  = Ax88179MacWrite (MEDIUMSTSMOD, 
                                   sizeof ( MediumStatus ),
                                   pNicDevice,
                                   &MediumStatus);
        if ( !EFI_ERROR ( Status ))
          pNicDevice->CurMediumStatus = MediumStatus;
    }
  }
  
  RxControl = RXCTL_SO | RXCTL_IPE;
    
  //
  //  Enable multicast if requested
  //
  if ( 0 != ( RxFilter & EFI_SIMPLE_NETWORK_RECEIVE_MULTICAST )) {
    RxControl |= RXCTL_AM;
    //
    //  Update the multicast hash table
    //
    Status  = Ax88179MacWrite (MULCATFLTARRY, 
                               8,
                               pNicDevice,
                               &pNicDevice->MulticastHash );
  }

  //
  //  Enable all multicast if requested
  //
  if ( 0 != ( RxFilter & EFI_SIMPLE_NETWORK_RECEIVE_PROMISCUOUS_MULTICAST )) {
    RxControl |= RXCTL_AMALL;
  }
  //
  //  Enable broadcast if requested
  //
  if ( 0 != ( RxFilter & EFI_SIMPLE_NETWORK_RECEIVE_BROADCAST )) {
    RxControl |= RXCTL_AB;
  }

  //
  //  Enable promiscuous mode if requested
  //
  if ( 0 != ( RxFilter & EFI_SIMPLE_NETWORK_RECEIVE_PROMISCUOUS )) {
    RxControl |= RXCTL_PRO;
  }

  //
  //  Update the receiver control
  //
  if (pNicDevice->CurRxControl != RxControl) {
    Status  = Ax88179MacWrite (RXCTL, 
                               0x02,
                               pNicDevice,
                               &RxControl);
    if ( !EFI_ERROR ( Status ))
      pNicDevice->CurRxControl = RxControl;
  }

  //
  // Return the operation status
  //
  return Status;
}

EFI_STATUS
Ax88179ReloadSrom  (
  IN NIC_DEVICE * pNicDevice
  )
{
  EFI_STATUS Status;

  Status = EFI_UNSUPPORTED;
  return Status;

}

/**
  Read an SROM location

  This routine calls ::Ax88179UsbCommand to read data from the
  SROM.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in] Address          SROM address
  @param [out] pData           Buffer to receive the data

  @retval EFI_SUCCESS          The read was successful
  @retval other                The read failed

**/
EFI_STATUS
Ax88179SromRead (
  IN NIC_DEVICE * pNicDevice,
  IN UINT32 Address,
  OUT UINT16 * pData
  )
{
  EFI_STATUS Status;

  Status = EFI_UNSUPPORTED;
  return Status;
}

EFI_STATUS
Ax88179EnableSromWrite  (
  IN NIC_DEVICE * pNicDevice
  )
{
  EFI_STATUS Status;

  Status = EFI_UNSUPPORTED;
  return Status;
}


EFI_STATUS
Ax88179DisableSromWrite  (
  IN NIC_DEVICE * pNicDevice
  )
{
  EFI_STATUS Status;

  Status = EFI_UNSUPPORTED;  
  return Status;
}

/**
  Write an SROM location

  This routine calls ::Ax88179UsbCommand to write data from the
  SROM.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in] Address          SROM address
  @param [out] pData           Buffer of data to write

  @retval EFI_SUCCESS          The write was successful
  @retval other                The write failed

**/
EFI_STATUS
Ax88179SromWrite (
  IN NIC_DEVICE * pNicDevice,
  IN UINT32 Address,
  IN UINT16 * pData
  )  
{
  EFI_STATUS Status;
  
  Status = EFI_UNSUPPORTED;   
  return Status;
}



/**
  Send a command to the USB device.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in] pRequest         Pointer to the request structure
  @param [in, out] pBuffer     Data buffer address

  @retval EFI_SUCCESS          The USB transfer was successful
  @retval other                The USB transfer failed

**/

EFI_STATUS
Ax88179UsbCommand (
  IN NIC_DEVICE * pNicDevice,
  IN USB_DEVICE_REQUEST * pRequest,
  IN OUT VOID * pBuffer
  )
{
  
  EFI_USB_DATA_DIRECTION Direction;
  EFI_USB_IO_PROTOCOL * pUsbIo;
  EFI_STATUS Status = EFI_TIMEOUT;
  UINT32 CmdStatus = EFI_USB_NOERROR;
  int i;
  //
  // Determine the transfer direction
  //
  Direction = EfiUsbNoData;
  if ( 0 != pRequest->Length ) {
    Direction = ( 0 != ( pRequest->RequestType & USB_ENDPOINT_DIR_IN ))
                        ? EfiUsbDataIn : EfiUsbDataOut;
  }

  //
  // Issue the command
  //
  pUsbIo = pNicDevice->pUsbIo;
  
  for ( i = 0 ; i < 3 && EFI_TIMEOUT == Status; i++) {
    Status = pUsbIo->UsbControlTransfer ( pUsbIo,
                                        pRequest,
                                        Direction,
                                        USB_BUS_TIMEOUT,
                                        pBuffer,
                                        pRequest->Length,
                                        &CmdStatus );
  }
  //
  // Determine the operation status
  //
  if (EFI_ERROR(Status) || EFI_ERROR(CmdStatus))
    Status = EFI_DEVICE_ERROR;
  //
  // Return the operation status
  //
  return Status;
}

EFI_STATUS
Ax88179MacRead (
  IN  UINT8 Offset,
  IN  UINT8 Length,
  IN  NIC_DEVICE * pNicDevice,
  IN  OUT  VOID * pData
  )
{

  EFI_STATUS Status;
  USB_DEVICE_REQUEST SetupMsg;
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE
                       | USB_ENDPOINT_DIR_IN;
                       
  SetupMsg.Request = ACCESS_MAC;
  SetupMsg.Value  = Offset;
  SetupMsg.Index  = Length;
  SetupMsg.Length = SetupMsg.Index;
  
  Status = Ax88179UsbCommand ( pNicDevice,
                               &SetupMsg,
                               pData );
                               
  return Status;
  
}

EFI_STATUS
Ax88179MacWrite (
  IN  UINT8 Offset,
  IN  UINT8 Length,
  IN  NIC_DEVICE * pNicDevice,
  IN  OUT  VOID * pData
  )
{

  EFI_STATUS Status;
  USB_DEVICE_REQUEST SetupMsg;
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
                       
  SetupMsg.Request = ACCESS_MAC;
  SetupMsg.Value  = Offset;
  SetupMsg.Index  = Length;
  SetupMsg.Length = SetupMsg.Index;
  
  Status = Ax88179UsbCommand ( pNicDevice,
                               &SetupMsg,
                               pData );
                               
  return Status;
  
}

EFI_STATUS
Ax88179SetIInInterval (
  IN  NIC_DEVICE * pNicDevice,
  IN  UINT8 Interval
  )
{

  EFI_STATUS Status;
  USB_DEVICE_REQUEST SetupMsg;
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
                       
  SetupMsg.Request = 0x91;
  SetupMsg.Value  = Interval;
  SetupMsg.Index  = 0;
  SetupMsg.Length = 0;
  
  Status = Ax88179UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL );
                               
  return Status;
  
}

EFI_STATUS
Ax88179SetMedium (
  IN NIC_DEVICE * pNicDevice
  )
{
  UINT16 MediumStatus;
  EFI_STATUS Status;
  
  MediumStatus = MEDIUMSTSMOD_RE | MEDIUMSTSMOD_ONE;
  if ( pNicDevice->bFullDuplex ) {
    MediumStatus |= MEDIUMSTSMOD_TFC | MEDIUMSTSMOD_RFC | MEDIUMSTSMOD_FD;
  } else {
    MediumStatus &= ~(MEDIUMSTSMOD_TFC | MEDIUMSTSMOD_RFC | MEDIUMSTSMOD_FD);
  }
  if ( pNicDevice->b1000Mbps ) {
    MediumStatus |= MEDIUMSTSMOD_GM;
    MediumStatus &= ~MEDIUMSTSMOD_PS;
  } else {
    MediumStatus &= ~MEDIUMSTSMOD_GM;
    if ( pNicDevice->b100Mbps ) {
      MediumStatus |= MEDIUMSTSMOD_PS;         
    } else {
      MediumStatus &= ~MEDIUMSTSMOD_PS;
    }
  }                
  Status  = Ax88179MacWrite (MEDIUMSTSMOD, 
                             sizeof ( MediumStatus ),
                             pNicDevice,
                             &MediumStatus);
  if ( !EFI_ERROR ( Status ))
  pNicDevice->CurMediumStatus = MediumStatus;

  return Status;
}


BOOLEAN
Ax88179GetLinkStatus (
  IN NIC_DEVICE * pNicDevice
)
{
  UINT32 CmdStatus;
  EFI_USB_IO_PROTOCOL * pUsbIo;
  UINT64    IntData = 0;
  UINTN     IntDataLeng = 8;
  EFI_STATUS Status;

  //
  // Issue the command
  //
  pUsbIo = pNicDevice->pUsbIo;
  Status = pUsbIo->UsbSyncInterruptTransfer( pUsbIo,
                                        USB_ENDPOINT_DIR_IN | INTERRUPT_ENDPOINT,
                                        &IntData,
                                        &IntDataLeng,
                                        USB_BUS_TIMEOUT,
                                        &CmdStatus );

  if ( EFI_ERROR(Status) || EFI_ERROR(CmdStatus) || 0 == IntDataLeng)
    return FALSE;

  return (IntData & 0x10000)? TRUE : FALSE;

}

EFI_STATUS  
Ax88179BulkIn(
  IN NIC_DEVICE * pNicDevice
) 
{
  int i;
  UINT16  val;
  UINTN LengthInBytes = 0;
  UINTN TMP_LENG = AX88179_MAX_BULKIN_SIZE;
  UINTN CURBufSize = AX88179_MAX_BULKIN_SIZE;
  UINTN PREBufSize = 0;
  EFI_STATUS Status = EFI_NOT_READY;
  EFI_USB_IO_PROTOCOL *pUsbIo;
  UINT32 TransferStatus;

  pNicDevice->SkipRXCnt = 0;
    
  pUsbIo = pNicDevice->pUsbIo; 
  for ( i = 0 ; i < (AX88179_MAX_BULKIN_SIZE / 512) && pUsbIo != NULL; i++) {
    VOID* tmpAddr = 0;

    if (pNicDevice->bSetZeroLen) {
      val =  PHYPWRRSTCTL_IPRL | PHYPWRRSTCTL_BZ;                
      Status = Ax88179MacWrite ( PHYPWRRSTCTL,
                                 sizeof (val),
                                 pNicDevice,
                                 &val);
      if (EFI_ERROR(Status)) {
        LengthInBytes = 0;
        Status = EFI_NOT_READY;
        goto no_pkt;
      }
      pNicDevice->bSetZeroLen = FALSE;
    }
    tmpAddr = (VOID*) &pNicDevice->pBulkInbuf[LengthInBytes];
        
    Status =  EFI_NOT_READY;
    Status = pUsbIo->UsbBulkTransfer ( pUsbIo,
                          USB_ENDPOINT_DIR_IN | BULK_IN_ENDPOINT,
                          tmpAddr,
                          &TMP_LENG,
                          BULKIN_TIMEOUT,        
                          &TransferStatus ); 

    if (( !EFI_ERROR ( Status )) && ( !EFI_ERROR ( TransferStatus )) && TMP_LENG != 0) {
      LengthInBytes += TMP_LENG;
      if ((TMP_LENG % pNicDevice->usbMaxPktSize) != 0) {	  
        goto done;
      } 
      CURBufSize = CURBufSize - TMP_LENG;
      TMP_LENG = CURBufSize;
      pNicDevice->bSetZeroLen = TRUE;
    } else if (( !EFI_ERROR ( Status )) &&
               ( !EFI_ERROR ( TransferStatus )) &&
               ( TMP_LENG == 0) &&
               LengthInBytes) {
      if (PREBufSize == CURBufSize) {
        goto done;
      }
      TMP_LENG = CURBufSize;
      PREBufSize = CURBufSize;
      pNicDevice->bSetZeroLen = TRUE;
    } else if (( !EFI_ERROR ( Status )) &&
               ( !EFI_ERROR ( TransferStatus )) &&
               ( TMP_LENG == 0)) {
      pNicDevice->bSetZeroLen = TRUE;
      LengthInBytes = 0;
      Status = EFI_NOT_READY;
      goto done;
    } else if (EFI_TIMEOUT == Status && EFI_USB_ERR_TIMEOUT == TransferStatus) { 
      pNicDevice->bSetZeroLen = TRUE;
      LengthInBytes = 0;
      Status = EFI_NOT_READY;
      goto done;
    } else {
      pNicDevice->bSetZeroLen = TRUE;
      LengthInBytes = 0;
      Status = EFI_NOT_READY;
      goto done;
    }
  }

done:
  if (LengthInBytes != 0) {
    UINT16 tmplen = 0;
    UINT16 tmpPktCnt = 0;

    tmpPktCnt = *((UINT16 *) (pNicDevice->pBulkInbuf + LengthInBytes - 4));
    tmplen =  *((UINT16*) (pNicDevice->pBulkInbuf + LengthInBytes - 2));

    if (((UINTN)(((tmpPktCnt * 4 + 4 + 7) & 0xfff8) + tmplen)) == LengthInBytes) {
      pNicDevice->PktCnt = tmpPktCnt;
      pNicDevice->pCurPktHdrOff = pNicDevice->pBulkInbuf + tmplen;
      pNicDevice->pCurPktOff = pNicDevice->pBulkInbuf; 
      *((UINT16 *) (pNicDevice->pBulkInbuf + LengthInBytes - 4)) = 0;
      *((UINT16*) (pNicDevice->pBulkInbuf + LengthInBytes - 2)) = 0;
      Status = EFI_SUCCESS;
    } else {
      Status = EFI_NOT_READY;
    }
  } else {
    Status = EFI_NOT_READY;
  }
no_pkt:
   return Status;
}
