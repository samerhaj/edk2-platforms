/** @file
*  Differentiated System Description Table Fields (DSDT)
*
*  Copyright (c) 2011-2015, ARM Limited. All rights reserved.
*  Copyright (c) 2016, Linaro Limited. All rights reserved.
*  Copyright 2019 NXP
*
*  SPDX-License-Identifier: BSD-2-Clause
*
*  Based on the files under ArmPlatformPkg/ArmJunoPkg/AcpiTables/
*
**/

Scope(_SB)
{
  // PCIe Root bus
  Device (PCI0)
  {
    Name (_HID, "PNP0A08") // PCI Express Root Bridge
    Name (_CID, "PNP0A03") // Compatible PCI Root Bridge
    Name(_SEG, 0) // Segment of this Root complex
    Name(_BBN, 0) // Base Bus Number
    Name(_CCA, 1) // Cache Coherency Attribute
    Method (_CRS, 0, Serialized) { // Root complex resources
      Name (RBUF, ResourceTemplate () {
        WordBusNumber ( // Bus numbers assigned to this root
          ResourceProducer, MinFixed, MaxFixed, PosDecode,
          0,                           // AddressGranularity
          LX2160A_PCI_SEG_BUSNUM_MIN,  // AddressMinimum - Minimum Bus Number
          LX2160A_PCI_SEG_BUSNUM_MAX,  // AddressMaximum - Maximum Bus Number
          0,                           // AddressTranslation
          LX2160A_PCI_SEG_BUSNUM_RANGE // RangeLength - # of Busses
        )
        QWordMemory ( // 64-bit BAR Windows
          ResourceProducer,
          PosDecode,
          MinFixed,
          MaxFixed,
          NonCacheable,
          ReadWrite,
          0x0,                           // Granularity
          LX2160A_PCI_SEG0_MMIO64_MIN,   // Min Base Address
          LX2160A_PCI_SEG0_MMIO64_MAX,   // Max Base Address
          LX2160A_PCI_SEG0_MMIO64_XLATE, // Translate
          LX2160A_PCI_SEG0_MMIO64_SIZE   // Length
        )
      })
      Return (RBUF)
    }

    Device (RES0)
    {
      Name (_HID, "NXP0016") // NXP PCIe RC config base address
      Name (_CRS, ResourceTemplate (){
        Memory32Fixed (ReadWrite, LX2160A_PCI_SEG0_RC_CONFIG_BASE,
                       LX2160A_PCI_SEG0_RC_CONFIG_SIZE)
      })
    }

    // PCI Routing Table
    Name (_PRT, Package () {
            Package () { 0xFFFF, 0, Zero, LX2160A_PCI_SEG0_INTA },   // INTA
            Package () { 0xFFFF, 1, Zero, LX2160A_PCI_SEG0_INTB },   // INTB
            Package () { 0xFFFF, 2, Zero, LX2160A_PCI_SEG0_INTC },   // INTC
            Package () { 0xFFFF, 3, Zero, LX2160A_PCI_SEG0_INTD },   // INTD
            })
    //
    // OS Control Handoff
    //
    Name (SUPP, Zero) // PCI _OSC Support Field value
    Name (CTRL, Zero) // PCI _OSC Control Field value

    Method(_OSC,4)
    {
        // Check for proper UUID
        If (LEqual (Arg0, ToUUID ("33DB4D5B-1FF7-401C-9657-7441C03DD766")))
        {
            // Create DWord-adressable fields from the Capabilities Buffer
            CreateDWordField (Arg3, 0, CDW1)
            CreateDWordField (Arg3, 4, CDW2)
            CreateDWordField (Arg3, 8, CDW3)

            // Save Capabilities DWord2 & 3
            Store (CDW2, SUPP)
            Store (CDW3, CTRL)

            // Disable PCIe and SHPC hotplug, AER and PME
            // And (CTRL, 0x10, CTRL)

            // Capabilities bits were masked
            Or (CDW1, 0x10, CDW1)

            // Update DWORD3 in the buffer
            Store (CTRL, CDW3)
            Return (Arg3)
       } Else {
            Or (CDW1, 4, CDW1) // Unrecognized UUID
                Return (Arg3)
       }
    } // End _OSC
  } // Device(PCI0)

  Device (RES0)
  {
    Name (_HID, "NXP0016") // NXP PCIe RC config base address
    Name (_CID, "PNP0C02") // Motherboard reserved resource
    Name (_UID, 0x0)  //  Unique ID
    Name (_CRS, ResourceTemplate (){
      Memory32Fixed (ReadWrite, LX2160A_PCI_SEG0_RC_CONFIG_BASE,
              LX2160A_PCI_SEG0_RC_CONFIG_SIZE)
    })
  }

  Device (PCI1)
  {
    Name (_HID, "PNP0A08") // PCI Express Root Bridge
    Name (_CID, "PNP0A03") // Compatible PCI Root Bridge
    Name(_SEG, 1) // Segment of this Root complex
    Name(_BBN, 0) // Base Bus Number
    Name(_CCA, 1) // Cache Coherency Attribute
    Method (_CRS, 0, Serialized) { // Root complex resources
      Name (RBUF, ResourceTemplate () {
        WordBusNumber ( // Bus numbers assigned to this root
          ResourceProducer, MinFixed, MaxFixed, PosDecode,
          0,                           // AddressGranularity
          LX2160A_PCI_SEG_BUSNUM_MIN,  // AddressMinimum - Minimum Bus Number
          LX2160A_PCI_SEG_BUSNUM_MAX,  // AddressMaximum - Maximum Bus Number
          0,                           // AddressTranslation
          LX2160A_PCI_SEG_BUSNUM_RANGE // RangeLength - # of Busses
        )
        QWordMemory ( // 64-bit BAR Windows
          ResourceProducer,
          PosDecode,
          MinFixed,
          MaxFixed,
          NonCacheable,
          ReadWrite,
          0x0,                           // Granularity
          LX2160A_PCI_SEG1_MMIO64_MIN,   // Min Base Address
          LX2160A_PCI_SEG1_MMIO64_MAX,   // Max Base Address
          LX2160A_PCI_SEG1_MMIO64_XLATE, // Translate
          LX2160A_PCI_SEG1_MMIO64_SIZE   // Length
        )
      })
      Return (RBUF)
    }

    Device (RES1)
    {
      Name (_HID, "NXP0016") // NXP PCIe RC config base address
      Name (_CRS, ResourceTemplate (){
        Memory32Fixed (ReadWrite, LX2160A_PCI_SEG1_RC_CONFIG_BASE,
                       LX2160A_PCI_SEG1_RC_CONFIG_SIZE)
      })
    }

    // PCI Routing Table
    Name (_PRT, Package () {
            Package () { 0xFFFF, 0, Zero, LX2160A_PCI_SEG1_INTA },   // INTA
            Package () { 0xFFFF, 1, Zero, LX2160A_PCI_SEG1_INTB },   // INTB
            Package () { 0xFFFF, 2, Zero, LX2160A_PCI_SEG1_INTC },   // INTC
            Package () { 0xFFFF, 3, Zero, LX2160A_PCI_SEG1_INTD },   // INTD
            })
    //
    // OS Control Handoff
    //
    Name (SUPP, Zero) // PCI _OSC Support Field value
    Name (CTRL, Zero) // PCI _OSC Control Field value

    Method(_OSC,4)
    {
        // Check for proper UUID
        If (LEqual (Arg0, ToUUID ("33DB4D5B-1FF7-401C-9657-7441C03DD766")))
        {
            // Create DWord-adressable fields from the Capabilities Buffer
            CreateDWordField (Arg3, 0, CDW1)
            CreateDWordField (Arg3, 4, CDW2)
            CreateDWordField (Arg3, 8, CDW3)

            // Save Capabilities DWord2 & 3
            Store (CDW2, SUPP)
            Store (CDW3, CTRL)

            // Disable PCIe and SHPC hotplug, AER and PME
            // And (CTRL, 0x10, CTRL)

            // Capabilities bits were masked
            Or (CDW1, 0x10, CDW1)

            // Update DWORD3 in the buffer
            Store (CTRL, CDW3)
            Return (Arg3)
       } Else {
            Or (CDW1, 4, CDW1) // Unrecognized UUID
                Return (Arg3)
       }
    } // End _OSC
  } // Device(PCI1)

  Device (RES1)
  {
    Name (_HID, "NXP0016") // NXP PCIe RC config base address
    Name (_CID, "PNP0C02") // Motherboard reserved resource
    Name (_UID, 0x1)  //  Unique ID
    Name (_CRS, ResourceTemplate (){
      Memory32Fixed (ReadWrite, LX2160A_PCI_SEG1_RC_CONFIG_BASE,
              LX2160A_PCI_SEG1_RC_CONFIG_SIZE)
    })
  }

  Device (RESP)  //reserve for ecam resource
  {
     Name (_HID, "PNP0C02")
     Name (_CRS, ResourceTemplate (){
       QwordMemory ( //ECAM space for [bus 00-ff]
         ResourceConsumer,
         PosDecode,
         MinFixed,
         MaxFixed,
         NonCacheable,
         ReadWrite,
         0x0,                               // Granularity
         LX2160A_PCI_SEG0_CONFIG_BASE,      // Min Base Address
         LX2160A_PCI_SEG0_CONFIG_BASE_MAX,  // Max Base Address
         0x0,                               // Translate
         LX2160A_PCI_SEG0_CONFIG_SIZE       // Length
       )
       QwordMemory ( //ECAM space for [bus 00-ff]
         ResourceConsumer,
         PosDecode,
         MinFixed,
         MaxFixed,
         NonCacheable,
         ReadWrite,
         0x0,                               // Granularity
         LX2160A_PCI_SEG1_CONFIG_BASE,      // Min Base Address
         LX2160A_PCI_SEG1_CONFIG_BASE_MAX,  // Max Base Address
         0x0,                               // Translate
         LX2160A_PCI_SEG1_CONFIG_SIZE       // Length
       )
     })
  }
}

