/** @file
  Implement the interface to the AX88772 Ethernet controller.

  This module implements the interface to the ASIX AX88772
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

#include "Ax88772.h"


/**
  Compute the CRC 

  @param [in] pMacAddress      Address of a six byte buffer to containing the MAC address.

  @returns The CRC-32 value associated with this MAC address

**/
UINT32
Ax88772Crc (
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

  This routine calls ::Ax88772UsbCommand to request the MAC
  address from the network adapter.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [out] pMacAddress      Address of a six byte buffer to receive the MAC address.

  @retval EFI_SUCCESS          The MAC address is available.
  @retval other                The MAC address is not valid.

**/
EFI_STATUS
Ax88772MacAddressGet (
  IN NIC_DEVICE * pNicDevice,
  OUT UINT8 * pMacAddress
  )
{
  USB_DEVICE_REQUEST SetupMsg;
  EFI_STATUS Status;

  //
  //  Set the register address.
  //
  SetupMsg.RequestType = USB_ENDPOINT_DIR_IN
                       | USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
  SetupMsg.Request = CMD_MAC_ADDRESS_READ;
  SetupMsg.Value = 0;
  SetupMsg.Index = 0;
  SetupMsg.Length = PXE_HWADDR_LEN_ETHER;

  //
  //  Read the PHY register
  //
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               pMacAddress );

  //
  // Return the operation status
  //
  return Status;
}


/**
  Set the MAC address

  This routine calls ::Ax88772UsbCommand to set the MAC address
  in the network adapter.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in] pMacAddress      Address of a six byte buffer to containing the new MAC address.

  @retval EFI_SUCCESS          The MAC address was set.
  @retval other                The MAC address was not set.

**/
EFI_STATUS
Ax88772MacAddressSet (
  IN NIC_DEVICE * pNicDevice,
  IN UINT8 * pMacAddress
  )
{
  USB_DEVICE_REQUEST SetupMsg;
  EFI_STATUS Status;

  //
  //  Set the register address.
  //
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
  SetupMsg.Request = CMD_MAC_ADDRESS_WRITE;
  SetupMsg.Value = 0;
  SetupMsg.Index = 0;
  SetupMsg.Length = PXE_HWADDR_LEN_ETHER;
  
  //
  //  Read the PHY register
  //
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               pMacAddress );
  
  //
  // Return the operation status
  //
  return Status;
}

/**
  Clear the multicast hash table

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure

**/
VOID
Ax88772MulticastClear (
  IN NIC_DEVICE * pNicDevice
  )
{
  int i = 0;
  //
  // Clear the multicast hash table
  //
  for ( i = 0 ; i < 8 ; i ++ )
     pNicDevice->MulticastHash[i] = 0;
}


/**
  Enable a multicast address in the multicast hash table

  This routine calls ::Ax88772Crc to compute the hash bit for
  this MAC address.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in] pMacAddress      Address of a six byte buffer to containing the MAC address.

**/
VOID
Ax88772MulticastSet (
  IN NIC_DEVICE * pNicDevice,
  IN UINT8 * pMacAddress
  )
{
  UINT32 Crc;

  //
  //  Compute the CRC on the destination address
  //
  Crc = Ax88772Crc ( pMacAddress ) >> 26;

  //
  //  Set the bit corresponding to the destination address
  //
  pNicDevice->MulticastHash [ Crc >> 3 ] |= ( 1<< (Crc& 7));

}

/**
  Start the link negotiation

  This routine calls ::Ax88772PhyWrite to start the PHY's link
  negotiation.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure

  @retval EFI_SUCCESS          The link negotiation was started.
  @retval other                Failed to start the link negotiation.

**/
EFI_STATUS
Ax88772NegotiateLinkStart (
  IN NIC_DEVICE * pNicDevice
  )
{
  UINT16 Control;
  EFI_STATUS Status;

  //
  // Set the supported capabilities.
  //
  Status = Ax88772PhyWrite ( pNicDevice,
                             PHY_ANAR,
                             AN_CSMA_CD
                             | AN_TX_FDX | AN_TX_HDX
                             | AN_10_FDX | AN_10_HDX | AN_FCS );
  if ( !EFI_ERROR ( Status )) {
    //
    // Set the link speed and duplex
    //
    Control = BMCR_AUTONEGOTIATION_ENABLE
            | BMCR_RESTART_AUTONEGOTIATION;
    if ( pNicDevice->b100Mbps ) {
      Control |= BMCR_100MBPS;
    }
    if ( pNicDevice->bFullDuplex ) {
      Control |= BMCR_FULL_DUPLEX;
    }
    Status = Ax88772PhyWrite ( pNicDevice, PHY_BMCR, Control );

    if (!EFI_ERROR(Status))
        gBS->Stall(3000000);
  }
  return Status;
}



/**
  Complete the negotiation of the PHY link

  This routine calls ::Ax88772PhyRead to determine if the
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
Ax88772NegotiateLinkComplete (
  IN NIC_DEVICE * pNicDevice,
  IN OUT UINTN * pPollCount,
  OUT BOOLEAN * pbComplete,
  OUT BOOLEAN * pbLinkUp,
  OUT BOOLEAN * pbHiSpeed,
  OUT BOOLEAN * pbFullDuplex
  )
{
  UINT16 Mask;
  UINT16 PhyData;
  EFI_STATUS  Status;
 
  //
  //  Determine if the link is up.
  //
  *pbComplete = FALSE;  

  //
  //  Get the link status
  //
  Status = Ax88772PhyRead ( pNicDevice,
                            PHY_BMSR,
                            &PhyData );

  if ( !EFI_ERROR ( Status )) {
    *pbLinkUp = (BOOLEAN)( 0 != ( PhyData & BMSR_LINKST ));
    if ( 0 == *pbLinkUp ) {
    } else {
      *pbComplete = (BOOLEAN)( 0 != ( PhyData & 0x20 ));  
      if ( 0 == *pbComplete ) {
      } else {
        Status = Ax88772PhyRead ( pNicDevice,
                                  PHY_ANLPAR,
                                  &PhyData );
        if ( !EFI_ERROR ( Status )) {
          //
          //  Autonegotiation is complete
          //  Determine the link speed.
          //
          *pbHiSpeed = (BOOLEAN)( 0 != ( PhyData & ( AN_TX_FDX | AN_TX_HDX )));

          //
          //  Determine the link duplex.
          //
          Mask = ( *pbHiSpeed ) ? AN_TX_FDX : AN_10_FDX;
          *pbFullDuplex = (BOOLEAN)( 0 != ( PhyData & Mask ));
        }
      } 
    }
  } else {
  }
  return Status;
}


/**
  Read a register from the PHY

  This routine calls ::Ax88772UsbCommand to read a PHY register.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in] RegisterAddress  Number of the register to read.
  @param [in, out] pPhyData    Address of a buffer to receive the PHY register value

  @retval EFI_SUCCESS          The PHY data is available.
  @retval other                The PHY data is not valid.

**/
EFI_STATUS
Ax88772PhyRead (
  IN NIC_DEVICE * pNicDevice,
  IN UINT8 RegisterAddress,
  IN OUT UINT16 * pPhyData
  )
{
  USB_DEVICE_REQUEST SetupMsg;
  EFI_STATUS Status;

  //
  //  Request access to the PHY
  //
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
  SetupMsg.Request = CMD_PHY_ACCESS_SOFTWARE;   
  SetupMsg.Value = 0;
  SetupMsg.Index = 0;
  SetupMsg.Length = 0;
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL );
  if ( !EFI_ERROR ( Status )) {
    //
    //  Read the PHY register address.
    //
    SetupMsg.RequestType = USB_ENDPOINT_DIR_IN
                         | USB_REQ_TYPE_VENDOR
                         | USB_TARGET_DEVICE;
    SetupMsg.Request = CMD_PHY_REG_READ;
    SetupMsg.Value = pNicDevice->PhyId;
    SetupMsg.Index = RegisterAddress;
    SetupMsg.Length = sizeof ( *pPhyData );
    Status = Ax88772UsbCommand ( pNicDevice,
                                 &SetupMsg,
                                 pPhyData );
    if ( !EFI_ERROR ( Status )) {

      //
      //  Release the PHY to the hardware
      //
      SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                           | USB_TARGET_DEVICE;
      SetupMsg.Request = CMD_PHY_ACCESS_HARDWARE;
      SetupMsg.Value = 0;
      SetupMsg.Index = 0;
      SetupMsg.Length = 0;
      Status = Ax88772UsbCommand ( pNicDevice,
                                   &SetupMsg,
                                   NULL );
    }
  }

  //
  //  Return the operation status.
  //

  return Status;
}


/**
  Write to a PHY register

  This routine calls ::Ax88772UsbCommand to write a PHY register.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in] RegisterAddress  Number of the register to read.
  @param [in] PhyData          Address of a buffer to receive the PHY register value

  @retval EFI_SUCCESS          The PHY data was written.
  @retval other                Failed to wwrite the PHY register.

**/
EFI_STATUS
Ax88772PhyWrite (
  IN NIC_DEVICE * pNicDevice,
  IN UINT8 RegisterAddress,
  IN UINT16 PhyData
  )
{
  USB_DEVICE_REQUEST SetupMsg;
  EFI_STATUS Status;

  //
  //  Request access to the PHY
  //
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
  SetupMsg.Request = CMD_PHY_ACCESS_SOFTWARE;
  SetupMsg.Value = 0;
  SetupMsg.Index = 0;
  SetupMsg.Length = 0;
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL );
  if ( !EFI_ERROR ( Status )) {
    //
    //  Write the PHY register
    //
    SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                         | USB_TARGET_DEVICE;
    SetupMsg.Request = CMD_PHY_REG_WRITE;
    SetupMsg.Value = pNicDevice->PhyId;
    SetupMsg.Index = RegisterAddress;
    SetupMsg.Length = sizeof ( PhyData );
    Status = Ax88772UsbCommand ( pNicDevice,
                                 &SetupMsg,
                                 &PhyData );
    if ( !EFI_ERROR ( Status )) {

      //
      //  Release the PHY to the hardware
      //
      SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                           | USB_TARGET_DEVICE;
      SetupMsg.Request = CMD_PHY_ACCESS_HARDWARE;
      SetupMsg.Value = 0;
      SetupMsg.Index = 0;
      SetupMsg.Length = 0;
      Status = Ax88772UsbCommand ( pNicDevice,
                                   &SetupMsg,
                                   NULL );
    }
  }

  //
  //  Return the operation status.
  //

  return Status;
}


/**
  Reset the AX88772

  This routine uses ::Ax88772UsbCommand to reset the network
  adapter.  This routine also uses ::Ax88772PhyWrite to reset
  the PHY.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure

  @retval EFI_SUCCESS          The MAC address is available.
  @retval other                The MAC address is not valid.

**/
EFI_STATUS
Ax88772Reset (
  IN NIC_DEVICE * pNicDevice
  )
{
  USB_DEVICE_REQUEST SetupMsg;
  EFI_STATUS Status;
  EFI_USB_IO_PROTOCOL *pUsbIo;
  
  pUsbIo = pNicDevice->pUsbIo;

  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                           | USB_TARGET_DEVICE;
  SetupMsg.Request = CMD_PHY_ACCESS_HARDWARE;
  SetupMsg.Value = 0;
  SetupMsg.Index = 0;
  SetupMsg.Length = 0;
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL );
                                   
  if (EFI_ERROR(Status)) goto err;                                 
                                   
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                          | USB_TARGET_DEVICE;
  SetupMsg.Request = CMD_PHY_SELECT;
  SetupMsg.Value = SPHY_PSEL;
  SetupMsg.Index = 0;
  SetupMsg.Length = 0;
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL );
                                    
  if (EFI_ERROR(Status)) goto err;  
                                     
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
  SetupMsg.Request = CMD_RESET;
  SetupMsg.Value = SRR_IPRL ;
  SetupMsg.Index = 0;
  SetupMsg.Length = 0;
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL );  
                                   
  if (EFI_ERROR(Status)) goto err;  
                                   
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
  SetupMsg.Request = CMD_RESET;
  SetupMsg.Value = SRR_IPPD | SRR_IPRL ;
  SetupMsg.Index = 0;
  SetupMsg.Length = 0;
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL );
                                   
  gBS->Stall ( 200000 );
    
  if (EFI_ERROR(Status)) goto err;  
    
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
  SetupMsg.Request = CMD_RESET;
  SetupMsg.Value =  SRR_IPRL  ;
  SetupMsg.Index = 0;
  SetupMsg.Length = 0;
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL );   
                                    
  gBS->Stall ( 200000 ); 
     
  if (EFI_ERROR(Status)) goto err;  
     
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
  SetupMsg.Request = CMD_RESET;
  SetupMsg.Value = 0;
  SetupMsg.Index = 0;
  SetupMsg.Length = 0;
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL );
                                    
  if (EFI_ERROR(Status)) goto err;
                                    
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
  SetupMsg.Request = CMD_PHY_SELECT;
  SetupMsg.Value = SPHY_PSEL;
  SetupMsg.Index = 0;
  SetupMsg.Length = 0;
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL ); 
                                    
  if (EFI_ERROR(Status)) goto err;
                                 
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
  SetupMsg.Request = CMD_RESET;
  SetupMsg.Value =  SRR_IPRL | SRR_BZ | SRR_BZTYPE;
  SetupMsg.Index = 0;
  SetupMsg.Length = 0;
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL );
                                    
  if (EFI_ERROR(Status)) goto err;
                                
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
  SetupMsg.Request = CMD_RX_CONTROL_WRITE;
  SetupMsg.Value = 0;
  SetupMsg.Index = 0;
  SetupMsg.Length = 0;
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL );  
                                  
  if (EFI_ERROR(Status)) goto err;  
    
  if (!pNicDevice->Flag772A) {
    SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                         | USB_TARGET_DEVICE; 
    SetupMsg.Request = CMD_RXQTC;
#if RXTHOU
    /*size cannot exceed 3K*/
    //SetupMsg.Value = 0x0;
    //SetupMsg.Index = 0x8001;
    /*size cannot exceed 16K*/
    SetupMsg.Value = 0x8300;
    SetupMsg.Index = 0x8500;
    /*size cannot exceed 32K*/
    //SetupMsg.Value = 0x8784;
    //SetupMsg.Index = 0x8A00;
    SetupMsg.Length = 0;
#else
    SetupMsg.Value = 0x8000;
    SetupMsg.Index = 0x8001;
#endif
    Status = Ax88772UsbCommand ( pNicDevice,
                                 &SetupMsg,
                                 NULL ); 
  }
err:
  return Status;
}

/**
  Enable or disable the receiver

  This routine calls ::Ax88772UsbCommand to update the
  receiver state.  This routine also calls ::Ax88772MacAddressSet
  to establish the MAC address for the network adapter.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in] RxFilter         Simple network RX filter mask value

  @retval EFI_SUCCESS          The MAC address was set.
  @retval other                The MAC address was not set.

**/
EFI_STATUS
Ax88772RxControl (
  IN NIC_DEVICE * pNicDevice,
  IN UINT32 RxFilter
  )
{
  UINT16 MediumStatus;
  UINT16 RxControl;
  USB_DEVICE_REQUEST SetupMsg;
  EFI_STATUS Status = EFI_SUCCESS;

  //
  // Enable the receiver if something is to be received
  //
  if ( 0 != RxFilter ) {
    //
    //  Enable the receiver
    //
    SetupMsg.RequestType = USB_ENDPOINT_DIR_IN
                         | USB_REQ_TYPE_VENDOR
                         | USB_TARGET_DEVICE;
    SetupMsg.Request = CMD_MEDIUM_STATUS_READ;    
    SetupMsg.Value = 0;
    SetupMsg.Index = 0;
    SetupMsg.Length = sizeof ( MediumStatus );
    Status = Ax88772UsbCommand ( pNicDevice,
                                 &SetupMsg,
                                 &MediumStatus );
    if ( !EFI_ERROR ( Status )) {
      if ( 0 == ( MediumStatus & MS_RE )) {
        MediumStatus |= MS_RE | MS_ONE;
        
        if ( pNicDevice->bFullDuplex )
          MediumStatus |= MS_TFC | MS_RFC | MS_FD;
        else
          MediumStatus &= ~(MS_TFC | MS_RFC | MS_FD);
        
        if ( pNicDevice->b100Mbps )
          MediumStatus |= MS_PS;
        else
          MediumStatus &= ~MS_PS;
        
        SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                             | USB_TARGET_DEVICE;
        SetupMsg.Request = CMD_MEDIUM_STATUS_WRITE;
        SetupMsg.Value = MediumStatus;
        SetupMsg.Index = 0;
        SetupMsg.Length = 0;
        Status = Ax88772UsbCommand ( pNicDevice,
                                     &SetupMsg,
                                     NULL );

        if (EFI_ERROR(Status))
              goto EXIT;
      }
    } else {
        goto EXIT;
    }
  }
  RxControl = RXC_SO;
  if (!pNicDevice->Flag772A)
    RxControl |= RXC_RH1M;
   
  //
  //  Enable multicast if requested
  //
  if ( 0 != ( RxFilter & EFI_SIMPLE_NETWORK_RECEIVE_MULTICAST )) {
      RxControl |= RXC_AM;
      //
      //  Update the multicast hash table
      //
      SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                           | USB_TARGET_DEVICE;
      SetupMsg.Request = CMD_MULTICAST_HASH_WRITE;
      SetupMsg.Value = 0;
      SetupMsg.Index = 0;
      SetupMsg.Length = sizeof ( pNicDevice ->MulticastHash );
      Status = Ax88772UsbCommand ( pNicDevice,
                                   &SetupMsg,
                                   &pNicDevice->MulticastHash );

      if (EFI_ERROR(Status))
              goto EXIT;
  }

  //
  //  Enable all multicast if requested
  //
  if ( 0 != ( RxFilter & EFI_SIMPLE_NETWORK_RECEIVE_PROMISCUOUS_MULTICAST )) {
      RxControl |= RXC_AMALL;
  }

  //
  //  Enable broadcast if requested
  //
  if ( 0 != ( RxFilter & EFI_SIMPLE_NETWORK_RECEIVE_BROADCAST )) {
      RxControl |= RXC_AB;
  }

  //
  //  Enable promiscuous mode if requested
  //
  if ( 0 != ( RxFilter & EFI_SIMPLE_NETWORK_RECEIVE_PROMISCUOUS )) {
      RxControl |= RXC_PRO;
  }
    
  //
  //  Update the receiver control
  //
  if (pNicDevice->CurRxControl != RxControl) {
    SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                         | USB_TARGET_DEVICE;
    SetupMsg.Request = CMD_RX_CONTROL_WRITE;
#if RXTHOU
    if (pNicDevice->Flag772A)
      RxControl |= 0x0300;
#endif
    if (pNicDevice->Flag772A)
      RxControl &= ~(RXC_MFB);
    SetupMsg.Value = RxControl;
    SetupMsg.Index = 0;
    SetupMsg.Length = 0;
    Status = Ax88772UsbCommand ( pNicDevice,
                                 &SetupMsg,
                                 NULL );
    if ( !EFI_ERROR ( Status ))
      pNicDevice->CurRxControl = RxControl;
  }

  //
  // Return the operation status
  //
EXIT:
  return Status;
}



EFI_STATUS
Ax88772ReloadSrom  (
  IN NIC_DEVICE * pNicDevice
  )
{
  USB_DEVICE_REQUEST SetupMsg;
  EFI_STATUS Status;

  //
  //  Read a value from the SROM
  //
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
                       
  SetupMsg.Request = CMD_WRITE_GPIOS;
  SetupMsg.Value = 0x80;
  SetupMsg.Index = 0;
  SetupMsg.Length = 0 ;
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL );
                               
  if (EFI_SUCCESS == Status)
          gBS->Stall(500000);  
   
  return Status;

}


/**
  Read an SROM location

  This routine calls ::Ax88772UsbCommand to read data from the
  SROM.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in] Address          SROM address
  @param [out] pData           Buffer to receive the data

  @retval EFI_SUCCESS          The read was successful
  @retval other                The read failed

**/
EFI_STATUS
Ax88772SromRead (
  IN NIC_DEVICE * pNicDevice,
  IN UINT32 Address,
  OUT UINT16 * pData
  )
{

  USB_DEVICE_REQUEST SetupMsg;
  EFI_STATUS Status;

  //
  //  Read a value from the SROM
  //
  SetupMsg.RequestType = USB_ENDPOINT_DIR_IN
                       | USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
  SetupMsg.Request = CMD_SROM_READ;
  SetupMsg.Value = (UINT16) Address;
  SetupMsg.Index = 0;
  SetupMsg.Length = sizeof ( *pData );
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               pData );

  //
  // Return the operation status
  //
  return Status;
}

EFI_STATUS
Ax88772EnableSromWrite  (
  IN NIC_DEVICE * pNicDevice
  )
{
  USB_DEVICE_REQUEST SetupMsg;
  EFI_STATUS Status;

  //
  //  Read a value from the SROM
  //
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
                       
  SetupMsg.Request = CMD_SROM_WRITE_EN;
  SetupMsg.Value = 0;
  SetupMsg.Index = 0;
  SetupMsg.Length = 0 ;
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL );  
                               
  if (EFI_SUCCESS == Status)
          gBS->Stall(500000);
   
  return Status;

}


EFI_STATUS
Ax88772DisableSromWrite  (
  IN NIC_DEVICE * pNicDevice
  )
{
  USB_DEVICE_REQUEST SetupMsg;
  EFI_STATUS Status;

  //
  //  Read a value from the SROM
  //
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
                       
  SetupMsg.Request = CMD_SROM_WRITE_DIS;
  SetupMsg.Value = 0;
  SetupMsg.Index = 0;
  SetupMsg.Length = 0;
  Status = Ax88772UsbCommand ( pNicDevice,
                      &SetupMsg,
                      NULL );  
                               
  if (EFI_SUCCESS == Status)
          gBS->Stall(500000);
   
  return Status;

}

/**
  Write an SROM location

  This routine calls ::Ax88772UsbCommand to write data from the
  SROM.

  @param [in] pNicDevice       Pointer to the NIC_DEVICE structure
  @param [in] Address          SROM address
  @param [out] pData           Buffer of data to write

  @retval EFI_SUCCESS          The write was successful
  @retval other                The write failed

**/
EFI_STATUS
Ax88772SromWrite (
  IN NIC_DEVICE * pNicDevice,
  IN UINT32 Address,
  IN UINT16 * pData
  )  
{

  USB_DEVICE_REQUEST SetupMsg;
  EFI_STATUS Status;    
  
  SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                       | USB_TARGET_DEVICE;
                       
  SetupMsg.Request = CMD_SROM_WRITE;
  SetupMsg.Value = (UINT16) Address;
  SetupMsg.Index = (UINT16) (*pData);
  SetupMsg.Length = 0;
  
  Status = Ax88772UsbCommand ( pNicDevice,
                               &SetupMsg,
                               NULL );

  //
  // Return the operation status
  //
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
Ax88772UsbCommand (
  IN NIC_DEVICE * pNicDevice,
  IN USB_DEVICE_REQUEST * pRequest,
  IN OUT VOID * pBuffer
  )
{
  UINT32 CmdStatus;
  EFI_USB_DATA_DIRECTION Direction;
  EFI_USB_IO_PROTOCOL * pUsbIo;
  EFI_STATUS Status;

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
  Status = pUsbIo->UsbControlTransfer ( pUsbIo,
                                        pRequest,
                                        Direction,
                                        USB_BUS_TIMEOUT,
                                        pBuffer,
                                        pRequest->Length,
                                        &CmdStatus );
 
  //
  // Determine the operation status
  //
  if ( !EFI_ERROR ( Status )) {
    Status = CmdStatus;
  } else {
    //
    // Only use status values associated with the Simple Network protocol
    //
    if ( EFI_TIMEOUT == Status ) {
      Status = EFI_DEVICE_ERROR;
    }
  }

  //
  // Return the operation status
  //

  return Status;
}

BOOLEAN
Ax88772GetLinkStatus (
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

  if ( EFI_ERROR(Status) || EFI_ERROR(CmdStatus) || 0 == IntDataLeng) {
      return FALSE;
  }
  return (IntData & 0x800000)? FALSE : TRUE;

}

#if RXTHOU
EFI_STATUS  
Ax88772BulkIn(
  IN NIC_DEVICE * pNicDevice
) 
{
  int i;
  UINTN LengthInBytes = 0;
  UINTN TMP_LENG = AX88772_MAX_BULKIN_SIZE;
  UINTN ORI_TMP_LENG = 0;
  EFI_STATUS Status = EFI_DEVICE_ERROR;
  EFI_USB_IO_PROTOCOL *pUsbIo;
  UINT32 TransferStatus = 0;
  UINT16 tmpPktCnt = 0; 
  UINT16 * tmpHdr = (UINT16 *)pNicDevice->pBulkInbuf;
  USB_DEVICE_REQUEST SetupMsg;

  pUsbIo = pNicDevice->pUsbIo; 
  for ( i = 0 ; i < (AX88772_MAX_BULKIN_SIZE / 512) && pUsbIo != NULL; i++) {
    VOID* tmpAddr = 0;

    tmpPktCnt = 0;
    tmpAddr = (VOID*) &pNicDevice->pBulkInbuf[LengthInBytes];
    ORI_TMP_LENG = TMP_LENG;
    Status = pUsbIo->UsbBulkTransfer ( pUsbIo,
                          USB_ENDPOINT_DIR_IN | BULK_IN_ENDPOINT,
                          tmpAddr,
                          &TMP_LENG,
                          BULKIN_TIMEOUT,
                          &TransferStatus );

    if (ORI_TMP_LENG == TMP_LENG) {
      Status = EFI_NOT_READY;
      goto no_pkt;
    }

    if (( !EFI_ERROR ( Status )) &&
        ( !EFI_ERROR ( TransferStatus )) &&
        TMP_LENG != 0) {              
      LengthInBytes += TMP_LENG;
      if ((TMP_LENG % 512) != 0) {	  
        goto done;
      }
    } else if (( !EFI_ERROR ( Status )) &&
               ( !EFI_ERROR ( TransferStatus )) &&
               (TMP_LENG == 0)) {
      Status = EFI_NOT_READY;
      goto done;
    } else if (EFI_TIMEOUT == Status && EFI_USB_ERR_TIMEOUT == TransferStatus) { 
      SetupMsg.RequestType = USB_REQ_TYPE_STANDARD | 0x02;
      SetupMsg.Request = 0x01;
      SetupMsg.Value =  0;
      SetupMsg.Index = 0x82;
      SetupMsg.Length = 0;
      Status = Ax88772UsbCommand ( pNicDevice,
                                   &SetupMsg,
                                   NULL );
      Status = EFI_NOT_READY;
      goto done;
    } else {
      Status = EFI_DEVICE_ERROR;
      goto done;
    }
  }
done:
  if (LengthInBytes != 0) {
    UINT16 tmpLen;
    UINT16 tmpLenBar;
    UINT16 tmpTotalLen = 0;
    UINTN TotalLen = LengthInBytes;

    do {
      tmpLen = (*tmpHdr) & 0x7FF;
      tmpLenBar = *(tmpHdr + 1);    
      tmpTotalLen = ((tmpLen + 4 + 1) & 0xfffe);
			
      if ((tmpLen & 0x7FF) + (tmpLenBar & 0x7FF) == 0x7FF) {
        tmpPktCnt++;
      } else {
        if ( tmpPktCnt != 0) {
          break;
        }
        Status = EFI_NOT_READY;
        goto no_pkt;
      }
      tmpHdr += (tmpTotalLen / 2);
      TotalLen -= tmpTotalLen;
    } while (TotalLen > 0);

  if (LengthInBytes >= 1000 && tmpPktCnt != 0) {
    if ((pNicDevice->rxburst) == 1) {
      SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                           | USB_TARGET_DEVICE;
      SetupMsg.Request = CMD_RESET;
      SetupMsg.Value =  SRR_IPRL;
      SetupMsg.Index = 0;
      SetupMsg.Length = 0;
      Ax88772UsbCommand ( pNicDevice,
                          &SetupMsg,
                          NULL );
    }
			
    if (pNicDevice->rxburst < 2)
    pNicDevice->rxburst++;

  } else {
    if (pNicDevice->rxburst >= 2) {
      SetupMsg.RequestType = USB_REQ_TYPE_VENDOR
                           | USB_TARGET_DEVICE;
      SetupMsg.Request = CMD_RESET;
      SetupMsg.Value =  SRR_IPRL| SRR_BZ | SRR_BZTYPE;
      SetupMsg.Index = 0;
      SetupMsg.Length = 0;
      Ax88772UsbCommand ( pNicDevice,
                          &SetupMsg,
                          NULL );
			}						
      pNicDevice->rxburst = 0;
    }
  }
	
  if (tmpPktCnt != 0) {
    pNicDevice->PktCnt = tmpPktCnt;
    pNicDevice->pCurPktHdrOff = pNicDevice->pBulkInbuf;
    pNicDevice->pCurPktOff = pNicDevice->pBulkInbuf + 4;
    Status = EFI_SUCCESS;
  }			 
   
no_pkt:
  return Status;
}
#else
EFI_STATUS  
Ax88772BulkIn(
  IN NIC_DEVICE * pNicDevice
) 
{
  int i;
  UINTN LengthInBytes = 0;
  UINTN TMP_LENG = AX88772_MAX_BULKIN_SIZE;
  UINTN ORI_TMP_LENG = 0;
  UINTN CURBufSize = AX88772_MAX_BULKIN_SIZE;
  EFI_STATUS Status = EFI_DEVICE_ERROR;
  EFI_USB_IO_PROTOCOL *pUsbIo;
  UINT32 TransferStatus = 0;
  UINT16 tmpPktCnt = 0; 
  UINT16 * tmpHdr = (UINT16 *)pNicDevice->pBulkInbuf;
  
  pUsbIo = pNicDevice->pUsbIo; 
  for ( i = 0 ; i < (AX88772_MAX_BULKIN_SIZE / 512) && pUsbIo != NULL; i++) {
    VOID* tmpAddr = 0;

    tmpPktCnt = 0;     
    tmpAddr = (VOID*) &pNicDevice->pBulkInbuf[LengthInBytes];
    ORI_TMP_LENG = TMP_LENG;
    Status = pUsbIo->UsbBulkTransfer ( pUsbIo,
                          USB_ENDPOINT_DIR_IN | BULK_IN_ENDPOINT,
                          tmpAddr,
                          &TMP_LENG,
                          BULKIN_TIMEOUT,
                          &TransferStatus );

    if (ORI_TMP_LENG == TMP_LENG) {
      Status = EFI_NOT_READY;
      goto no_pkt;
    }

    if (( !EFI_ERROR ( Status )) &&
        ( !EFI_ERROR ( TransferStatus )) &&
        TMP_LENG != 0) {
      UINT16 tmpLen;
      UINT16 tmpLenBar;
      UINT16 tmpTotalLen = 0;

      LengthInBytes += TMP_LENG;
      CURBufSize = CURBufSize - TMP_LENG;
      TMP_LENG = CURBufSize;
      do {
        tmpLen = *tmpHdr;
        tmpLenBar = *(tmpHdr + 1);    
        tmpTotalLen += ((tmpLen + 4 + 1) & 0xfffe); 
                
        if (((tmpLen ^ tmpLenBar) == 0xffff))  {
          if (tmpTotalLen == LengthInBytes) {
            tmpPktCnt++;
            Status = EFI_SUCCESS;
            goto done;
          } else if (tmpTotalLen > LengthInBytes) {
            break;
          }  
        } else if (((tmpLen ^ tmpLenBar) != 0xffff)) {
          if ( tmpPktCnt != 0) {
            Status = EFI_SUCCESS;
            goto done;
          }
          Status = EFI_NOT_READY;
          goto no_pkt;
        }
        tmpHdr += (tmpTotalLen / 2);
        tmpPktCnt++;
      }while (tmpTotalLen < LengthInBytes);
    } else if (( !EFI_ERROR ( Status )) &&
               ( !EFI_ERROR ( TransferStatus )) &&
               (TMP_LENG == 0)) {
      if ( tmpPktCnt != 0) {
        Status = EFI_SUCCESS;
        goto done;
      }
      Status = EFI_NOT_READY;
      goto no_pkt;
    } else if (EFI_TIMEOUT == Status && EFI_USB_ERR_TIMEOUT == TransferStatus) { 
      if ( tmpPktCnt != 0) {
        Status = EFI_SUCCESS;
        goto done;
      }
      Status = EFI_NOT_READY;
      goto no_pkt;
    } else {
      if ( tmpPktCnt != 0) {
        Status = EFI_SUCCESS;
        goto done;
      }
      Status = EFI_DEVICE_ERROR;
      goto no_pkt;
    }
  }
done:
  pNicDevice->PktCnt = tmpPktCnt;
  pNicDevice->pCurPktHdrOff = pNicDevice->pBulkInbuf;
  pNicDevice->pCurPktOff = pNicDevice->pBulkInbuf + 4; 
no_pkt:
  return Status;
}
#endif
