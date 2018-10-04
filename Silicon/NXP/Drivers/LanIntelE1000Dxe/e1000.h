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

#ifndef _E1000_H_
#define _E1000_H_

#define MAJORVERSION 4
#define MINORVERSION 4
#define BUILDNUMBER  6


#include "e1000_api.h"
#include "vlan.h"
#include "hii.h"
#include "FirmwareManagement.h"
#include "startstop.h"

#include <Uefi.h>

#include <IndustryStandard/Pci.h>

#include <Protocol/ComponentName.h>
#include <Protocol/DriverBinding.h>
#include <Protocol/DriverSupportedEfiVersion.h>
#include <Protocol/FormBrowser2.h>
#include <Protocol/HiiConfigAccess.h>
#include <Protocol/HiiConfigRouting.h>
#include <Protocol/HiiDatabase.h>
#include <Protocol/HiiString.h>
#include <Protocol/LoadedImage.h>
#include <Protocol/NetworkInterfaceIdentifier.h>
#include <Protocol/PciIo.h>

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/CacheMaintenanceLib.h>
#include <Library/DevicePathLib.h>
#include <Library/HiiLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PrintLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>

//
// Debug levels for driver DEBUG_PRINT statements
//
#define NONE        0
#define INIT        (1 << 0)
#define DECODE      (1 << 1)
#define E1000       (1 << 2)
#define SHARED      (1 << 3)
#define DIAG        (1 << 4)
#define CFG         (1 << 5)
#define IO          (1 << 6)
#define VLAN        (1 << 7)
#define CRITICAL    (1 << 8)
#define CLP         (1 << 9)
#define TX          (1 << 10)
#define RX          (1 << 11)
#define HW          (1 << 12)
#define HII         (1 << 13)
#define IMAGE       (1 << 14)
#define WAIT        (1 << 15)

#ifndef DBG_LVL
#define DBG_LVL     (NONE)
#endif

#define MAX_NIC_INTERFACES 256

//
// Device and Vendor IDs
//
#define INTEL_VENDOR_ID         0x8086
#define E1000_VENDOR_ID         INTEL_VENDOR_ID
#define E1000_SUBVENDOR_ID      0x8086


#define TWO_PAIR_DOWNSHIFT_TIMEOUT 30

//
// PCI Base Address Register Bits
//
#define PCI_BAR_IO_MASK             0x00000003
#define PCI_BAR_IO_MODE             0x00000001

#define PCI_BAR_MEM_MASK            0x0000000F
#define PCI_BAR_MEM_MODE            0x00000000
#define PCI_BAR_MEM_64BIT           0x00000004

//
// Bit fields for the PCI command register
//
#define PCI_COMMAND_MWI     0x10
#define PCI_COMMAND_MASTER  0x04
#define PCI_COMMAND_MEM     0x02
#define PCI_COMMAND_IO      0x01
#define PCI_COMMAND         0x04
#define PCI_LATENCY_TIMER   0x0D

//
// PCI Capability IDs
//
#define PCI_EX_CAP_ID           0x10
#define PCI_CAP_PTR_ENDPOINT    0x00

//
// PCI Configuration Space Register Offsets
//
#define PCI_CAP_PTR         0x34    // PCI Capabilities pointer

//
// Register offsets for IO Mode read/write
//
#define IO_MODE_IOADDR      0x00
#define IO_MODE_IODATA      0x04

#define ETHER_MAC_ADDR_LEN 6

//
// PBA constants
//
#define E1000_PBA_16K 0x0010    // 16KB, default TX allocation
#define E1000_PBA_22K 0x0016
#define E1000_PBA_24K 0x0018
#define E1000_PBA_30K 0x001E
#define E1000_PBA_40K 0x0028
#define E1000_PBA_48K 0x0030    // 48KB, default RX allocation


//
// EEPROM Word Defines:
//
#define INIT_CONTROL_WORD_2     0x0F

//
// Initialization Control Word 2 indicates flash size
// 000: 64KB, 001: 128KB, 010: 256KB, 011: 512KB, 100: 1MB, 101: 2MB, 110: 4MB, 111: 8MB
// The Flash size impacts the requested memory space for the Flash and expansion ROM BARs.
//
#define FLASH_SIZE_MASK     0x0700
#define FLASH_SIZE_SHIFT    8

//
// "Main Setup Options Word"
//
#define SETUP_OPTIONS_WORD 0x30
#define SETUP_OPTIONS_WORD_LANB 0x34
#define SETUP_OPTIONS_WORD_LANC 0x38
#define SETUP_OPTIONS_WORD_LAND 0x3A

#define FDP_FULL_DUPLEX_BIT   0x1000
#define FSP_100MBS            0x0800
#define FSP_10MBS             0x0400
#define FSP_AUTONEG           0x0000
#define FSP_MASK              0x0C00
#define DISPLAY_SETUP_MESSAGE 0x0100

// UEFI 2.3 spec (Errata B) add following new definitions to UNDI interface:

//
// Return current media status.
//
#define PXE_OPFLAGS_GET_MEDIA_STATUS 0x0004

#define PXE_STATFLAGS_GET_STATUS_NO_MEDIA_MASK 0x0002
#define PXE_STATFLAGS_GET_STATUS_NO_MEDIA_NOT_SUPPORTED 0x0000
#define PXE_STATFLAGS_GET_STATUS_NO_MEDIA_SUPPORTED 0x0002

//
// This flag is set if there is no media detected.
//
#define PXE_STATFLAGS_GET_STATUS_NO_MEDIA 0x0040

//
// "Configuration Customization Word"
//
#define CONFIG_CUSTOM_WORD 0x31
#define CONFIG_CUSTOM_WORD_LANB 0x35
#define CONFIG_CUSTOM_WORD_LANC 0x39
#define CONFIG_CUSTOM_WORD_LAND 0x3B

#define SIG               0x4000
#define SIG_MASK          0xC000

#define EEPROM_CAPABILITIES_WORD       0x33
#define EEPROM_CAPABILITIES_SIG        0x4000
#define EEPROM_CAPABILITIES_SIG_MASK   0xC000
#define EEPROM_BC_BIT                  0x0001
#define EEPROM_UNDI_BIT                0x0002
#define EEPROM_PXE_BIT                 (EEPROM_BC_BIT | EEPROM_UNDI_BIT)
#define EEPROM_RPL_BIT                 0x0004
#define EEPROM_EFI_BIT                 0x0008
#define EEPROM_FCOE_BIT                0x0020
#define EEPROM_ISCSI_BIT               0x0010
#define EEPROM_LEGACY_BIT              (EEPROM_PXE_BIT | EEPROM_ISCSI_BIT)
#define EEPROM_SMCLP_BIT               0x0040
#define EEPROM_TYPE_MASK               (EEPROM_BC_BIT | EEPROM_UNDI_BIT | EEPROM_EFI_BIT | EEPROM_ISCSI_BIT)
#define EEPROM_ALL_BITS                (EEPROM_TYPE_MASK | EEPROM_RPL_BIT)

#define COMPATIBILITY_WORD          0x03
#define COMPATABILITY_LOM_BIT       0x0800 // bit 11
//
// UNDI_CALL_TABLE.state can have the following values
//
#define DONT_CHECK -1
#define ANY_STATE -1
#define MUST_BE_STARTED 1
#define MUST_BE_INITIALIZED 2

#define EFI_OPTIONAL_PTR                    0x00000001
#define EFI_INTERNAL_PTR                    0x00000004      // Pointer to internal runtime data
#define EVT_SIGNAL_VIRTUAL_ADDRESS_CHANGE   0x60000202

#define GIG_UNDI_DEV_SIGNATURE   SIGNATURE_32('P','R','0','g')
#define GIG_UNDI_PRIVATE_DATA_FROM_THIS(a) CR(a, GIG_UNDI_PRIVATE_DATA, NIIProtocol_31, GIG_UNDI_DEV_SIGNATURE)
#define GIG_UNDI_PRIVATE_DATA_FROM_DEVICE_PATH(a) \
        CR(a, GIG_UNDI_PRIVATE_DATA, Undi32BaseDevPath, GIG_UNDI_DEV_SIGNATURE)
#define GIG_UNDI_PRIVATE_DATA_FROM_FIRMWARE_MANAGEMENT(a) \
        CR(a, GIG_UNDI_PRIVATE_DATA, FirmwareManagement, GIG_UNDI_DEV_SIGNATURE)
#define GIG_UNDI_PRIVATE_DATA_FROM_DRIVER_STOP(a) \
        CR(a, GIG_UNDI_PRIVATE_DATA, DriverStop, GIG_UNDI_DEV_SIGNATURE)

//
// Macro to compare MAC addresses.  Returns TRUE if the MAC addresses match.
// a and b must be UINT8 pointers to the first byte of MAC address.
//
#ifdef EFI64
#define E1000_COMPARE_MAC(a, b) \
        ( (a[0]==b[0]) && (a[1]==b[1]) && (a[2]==b[2]) && (a[3]==b[3]) && (a[4]==b[4]) && (a[5]==b[5]))
#else
#define E1000_COMPARE_MAC(a, b) \
        ( *((UINT32*) a) == *((UINT32*) b) ) && ( *((UINT16*) (a+4)) == *((UINT16*) (b+4)) )
#endif

#ifdef EFI64
#define E1000_COPY_MAC(a, b) \
        a[0]=b[0]; a[1]=b[1]; a[2]=b[2]; a[3]=b[3]; a[4]=b[4]; a[5]=b[5];
#else
#define E1000_COPY_MAC(a, b) \
        *((UINT32*) a) = *((UINT32*) b); *((UINT16*) (a+4)) = *((UINT16*) (b+4))
#endif


typedef struct {
  UINT16 cpbsize;
  UINT16 dbsize;
  UINT16 opflags;
  UINT16 state;
  void (*api_ptr)();
} UNDI_CALL_TABLE;

typedef struct {
  EFI_NETWORK_INTERFACE_IDENTIFIER_PROTOCOL  *InterfacePointer;
  EFI_DEVICE_PATH_PROTOCOL                   *DevicePathPointer;
} NII_ENTRY;

typedef struct NII_CONFIG_ENTRY {
  UINT32                  NumEntries;
  UINT32                  Reserved;
  struct NII_CONFIG_ENTRY *NextLink;
  NII_ENTRY               NiiEntry[MAX_NIC_INTERFACES];
} NII_TABLE;

typedef struct _EFI_PRO1000_COM_PROTOCOL {
  EFI_NETWORK_INTERFACE_IDENTIFIER_PROTOCOL *NIIProtocol_31;
} EFI_PRO1000_COM_PROTOCOL;

typedef VOID (*ptr)(VOID);
typedef VOID (*bsptr_30)(UINTN);
typedef VOID (*virtphys_30)(UINT64, UINT64);
typedef VOID (*block_30)(UINT32);
typedef VOID (*mem_io_30)(UINT8, UINT8, UINT64, UINT64);

typedef VOID (*bsptr)(UINT64, UINTN);
typedef VOID (*virtphys)(UINT64, UINT64, UINT64);
typedef VOID (*block)(UINT64, UINT32);
typedef VOID (*mem_io)(UINT64, UINT8, UINT8, UINT64, UINT64);

typedef VOID (*map_mem)(UINT64, UINT64, UINT32, UINT32, UINT64);
typedef VOID (*unmap_mem)(UINT64, UINT64, UINT32, UINT32, UINT64);
typedef VOID (*sync_mem)(UINT64, UINT64, UINT32, UINT32, UINT64);

//
// Global Variable
//
extern EFI_DRIVER_BINDING_PROTOCOL gGigUndiDriverBinding;
extern EFI_COMPONENT_NAME_PROTOCOL gGigUndiComponentName;
extern EFI_TIME gTime;

#pragma pack(1)
typedef struct eth {
  UINT8  dest_addr[PXE_HWADDR_LEN_ETHER];
  UINT8  src_addr[PXE_HWADDR_LEN_ETHER];
  UINT16 type;
} ETHER_HEADER;

#pragma pack(1)
typedef struct PCI_CONFIG_HEADER_s {
  UINT16 VendorID;
  UINT16 DeviceID;
  UINT16 Command;
  UINT16 Status;
  UINT16 RevID;
  UINT16 ClassID;
  UINT8  CacheLineSize;
  UINT8  LatencyTimer;
  UINT8  HeaderType;
  UINT8  BIST;
  UINT32 BaseAddressReg_0;
  UINT32 BaseAddressReg_1;
  UINT32 BaseAddressReg_2;
  UINT32 BaseAddressReg_3;
  UINT32 BaseAddressReg_4;
  UINT32 BaseAddressReg_5;
  UINT32 CardBusCISPtr;
  UINT16 SubVendorID;
  UINT16 SubSystemID;
  UINT32 ExpansionROMBaseAddr;
  UINT8 CapabilitiesPtr;
  UINT8 reserved1;
  UINT16 Reserved2;
  UINT32 Reserved3;
  UINT8 int_line;
  UINT8 int_pin;
  UINT8 Min_gnt;
  UINT8 Max_lat;
} PCI_CONFIG_HEADER;
#pragma pack()

typedef struct e1000_rx_desc E1000_RECEIVE_DESCRIPTOR;


//
// TX Buffer size including crc and padding
//
#define RX_BUFFER_SIZE 2048

#define DEFAULT_RX_DESCRIPTORS 8
#define DEFAULT_TX_DESCRIPTORS 8

//
// Macro to conver byte memory requirement into pages
//
#define UNDI_MEM_PAGES(x) (((x) - 1) / 4096 + 1)

#pragma pack(1)
typedef struct _LocalReceiveBuffer {
  UINT8   RxBuffer[RX_BUFFER_SIZE - (sizeof(UINT64))];
  UINT64  BufferUsed;
} LOCAL_RX_BUFFER, *PLOCAL_RX_BUFFER;
#pragma pack()

typedef struct e1000_tx_desc E1000_TRANSMIT_DESCRIPTOR;

//
// If using a serial flash, this struct will be filled with the
// proper offsets, since I82540 and I82541 use different registers
// for Flash manipulation.
//
typedef struct _SERIAL_FLASH_OFFSETS
{
    struct
    {
        UINT32              RegisterOffset;
        UINT32              Sck;
        UINT32              Cs;
    } FlashClockRegister;
    struct
    {
        UINT32              RegisterOffset;
        UINT32              Si;
        UINT32              So;
        UINT8               SoPosition;
    } FlashDataRegister;

} SERIAL_FLASH_OFFSETS;


typedef struct _DRIVER_DATA_ {
  UINT16 State;   // stopped, started or initialized

  struct e1000_hw       hw;
  struct e1000_hw_stats stats;

  UINTN  Segment;
  UINTN  Bus;
  UINTN  Device;
  UINTN  Function;

  SERIAL_FLASH_OFFSETS SerialFlashOffsets;

  UINT8 BroadcastNodeAddress[PXE_MAC_LENGTH];

  UINT32 PciConfig[MAX_PCI_CONFIG_LEN];
  UINT32 NVData[MAX_EEPROM_LEN];

  UINTN  HwInitialized;
  UINTN  DriverBusy;
  UINT16 LinkSpeed;     // requested (forced) link speed
  UINT8  DuplexMode;    // requested duplex
  UINT8  CableDetect;   // 1 to detect and 0 not to detect the cable
  UINT8  LoopBack;

  UINT16 SetupWord;
  UINT16 BackupSetupWord;
  UINT16 CustomConfigWord;

  UINT64                  Unique_ID;
  EFI_PCI_IO_PROTOCOL     *PciIo;

  VOID (*Delay)(UINT64, UINTN);
  VOID (*Virt2Phys)(UINT64 unq_id, UINT64 virtual_addr, UINT64 physical_ptr);
  VOID (*Block)(UINT64 unq_id, UINT32 enable);
  VOID (*MemIo)(UINT64 unq_id, UINT8 read_write, UINT8 len, UINT64 port,
            UINT64 buf_addr);
  VOID (*MapMem)(UINT64 unq_id, UINT64 virtual_addr, UINT32 size,
            UINT32 Direction, UINT64 mapped_addr);
  VOID (*UnMapMem)(UINT64 unq_id, UINT64 virtual_addr, UINT32 size,
            UINT32 Direction, UINT64 mapped_addr);
  VOID (*SyncMem)(UINT64 unq_id, UINT64 virtual_addr,
            UINT32 size, UINT32 Direction, UINT64 mapped_addr);

  UINT8  IoBarIndex;

  UINT64 MemoryPtr;
  UINT32 MemoryLength;

  UINT16  Rx_Filter;
  UINT16  int_mask;
  UINT16  Int_Status;

  struct _McastList{
    UINT16 Length;
    UINT8  McAddr[MAX_MCAST_ADDRESS_CNT][PXE_MAC_LENGTH]; // 8*32 is the size
  } McastList;

  UINT16                    cur_rx_ind;
  UINT16                    cur_tx_ind;
  UINT8                     ReceiveStarted;
  E1000_RECEIVE_DESCRIPTOR  *rx_ring;
  E1000_TRANSMIT_DESCRIPTOR *tx_ring;
  LOCAL_RX_BUFFER           *local_rx_buffer;
  UINT16                    xmit_done_head;
  UINT64                    TxBufferUnmappedAddr[DEFAULT_TX_DESCRIPTORS];
  BOOLEAN                   MacAddrOverride;
  UINT64                    DebugRxBuffer[DEFAULT_RX_DESCRIPTORS];
  BOOLEAN                   VlanEnable;
  UINT16                    VlanTag;
  BOOLEAN                   FlashWriteInProgress;

  UINTN                     VersionFlag;  // Indicates UNDI version 3.0 or 3.1
} GIG_DRIVER_DATA, *PADAPTER_STRUCT;

typedef struct {
  UINTN                                       Signature;
  EFI_NETWORK_INTERFACE_IDENTIFIER_PROTOCOL   NIIProtocol_31;
  EFI_PRO1000_COM_PROTOCOL                    EfiPro1000Com;
  EFI_HANDLE                                  ControllerHandle;
  EFI_HANDLE                                  DeviceHandle;
  EFI_DEVICE_PATH_PROTOCOL                    *Undi32BaseDevPath;
  EFI_DEVICE_PATH_PROTOCOL                    *Undi32DevPath;
  GIG_DRIVER_DATA                             NicInfo;
  EFI_UNICODE_STRING_TABLE                    *ControllerNameTable;

  CHAR16                                      *Brand;

  // HII Configuration
  EFI_HII_HANDLE                              HiiHandle;
  GIG_DRIVER_CONFIGURATION                    Configuration;

  //
  // Consumed protocol
  //
  EFI_HII_DATABASE_PROTOCOL                   *HiiDatabase;
  EFI_HII_STRING_PROTOCOL                     *HiiString;
  EFI_HII_CONFIG_ROUTING_PROTOCOL             *HiiConfigRouting;
  EFI_FORM_BROWSER2_PROTOCOL                  *FormBrowser2;

  EFI_GUID                                    HiiFormGuid;

  //
  // Produced protocol
  //
  EFI_HII_CONFIG_ACCESS_PROTOCOL              ConfigAccess;
  EFI_DRIVER_STOP_PROTOCOL                    DriverStop;
  EFI_FIRMWARE_MANAGEMENT_PROTOCOL            FirmwareManagement;

} GIG_UNDI_PRIVATE_DATA;

struct GIG_UNDI_DMA_RESOURCES {
  E1000_RECEIVE_DESCRIPTOR rx_ring[DEFAULT_RX_DESCRIPTORS];
  E1000_TRANSMIT_DESCRIPTOR tx_ring[DEFAULT_TX_DESCRIPTORS];
  LOCAL_RX_BUFFER rx_buffer[DEFAULT_RX_DESCRIPTORS];
};

#define BYTE_ALIGN_64    0x7F

//
// We need enough space to store TX descriptors, RX descriptors,
// RX buffers, and enough left over to do a 64 byte alignment.
//
#define MEMORY_NEEDED (sizeof(struct GIG_UNDI_DMA_RESOURCES)+BYTE_ALIGN_64)

#define FOUR_GIGABYTE (UINT64)0x100000000

//
// functions defined in init.c
//

VOID
GetUndiControllerName (
  GIG_UNDI_PRIVATE_DATA *GigUndiPrivateData
  );

//
// functions defined in decode.c
//
VOID e1000_UNDI_GetState();
VOID e1000_UNDI_Start();
VOID e1000_UNDI_Stop();
VOID e1000_UNDI_GetInitInfo();
VOID e1000_UNDI_GetConfigInfo();
VOID e1000_UNDI_Initialize();
VOID e1000_UNDI_Reset();
VOID e1000_UNDI_Shutdown();
VOID e1000_UNDI_Interrupt();
VOID e1000_UNDI_RecFilter();
VOID e1000_UNDI_StnAddr();
VOID e1000_UNDI_Statistics();
VOID e1000_UNDI_ip2mac();
VOID e1000_UNDI_NVData();
VOID e1000_UNDI_Status();

VOID
e1000_UNDI_FillHeader (
  IN PXE_CDB         *CdbPtr,
  IN GIG_DRIVER_DATA *GigAdapterInfo
  );
VOID e1000_UNDI_Transmit();
VOID e1000_UNDI_Receive();

VOID e1000_UNDI_APIEntry();

VOID WaitForEnter(VOID);

PXE_STATCODE
e1000_Inititialize (
  GIG_DRIVER_DATA *GigAdapterInfo
  );

EFI_STATUS
e1000_FirstTimeInit (
  GIG_DRIVER_DATA *GigAdapterInfo
  );

VOID
e1000_ReceiveEnable (
  IN GIG_DRIVER_DATA *GigAdapterInfo
  );

VOID
e1000_ReceiveDisable (
  IN GIG_DRIVER_DATA *GigAdapterInfo
  );

UINTN
e1000_Transmit (
  GIG_DRIVER_DATA *GigAdapterInfo,
  UINT64          cpb,
  UINT16          opflags
  );

UINTN
e1000_Receive (
  GIG_DRIVER_DATA *GigAdapterInfo,
  UINT64          cpb,
  UINT64          db
  );

UINTN
e1000_Reset (
  GIG_DRIVER_DATA *GigAdapterInfo,
  UINT16          OpFlags
  );

UINTN
e1000_Shutdown (
  GIG_DRIVER_DATA *GigAdapter
  );


UINT16
e1000_FreeTxBuffers (
  IN GIG_DRIVER_DATA *GigAdapter,
  IN UINT16          NumEntries,
  OUT UINT64         *TxBuffer
  );

VOID
EepromMacAddressDefault(
  IN GIG_DRIVER_DATA *GigAdapter
  );

VOID
e1000_MemCopy (
  IN UINT8* Dest,
  IN UINT8* Source,
  IN UINT32 Count
  );

UINT32
e1000_SetRegBits (
  GIG_DRIVER_DATA *GigAdapter,
  UINT32           Register,
  UINT32           BitMask
  );

UINT32
e1000_ClearRegBits (
  GIG_DRIVER_DATA *GigAdapter,
  UINT32           Register,
  UINT32           BitMask
  );

//
// This is the Macro Section
//
#if DBG_LVL
#define DEBUGPRINT(lvl, msg) \
            if ((DBG_LVL & lvl) != 0) Aprint msg;
#define DEBUGWAIT(lvl) \
            if (((DBG_LVL | WAIT) & lvl) == (DBG_LVL | WAIT)) WaitForEnter();
#define DEBUGPRINTTIME(lvl) \
            if ((DBG_LVL & lvl) != 0) gRT->GetTime (&gTime, NULL);; \
            DEBUGPRINT(lvl, ("Timestamp - %dH:%dM:%dS:%dNS\n", \
            gTime.Hour, gTime.Minute, gTime.Second, gTime.Nanosecond));
#else
// Comment out the debug stuff
#define DEBUGPRINT(lvl, msg)
#define DEBUGWAIT(lvl)
#endif

//
// Time translations.
//
VOID DelayInMicroseconds (IN GIG_DRIVER_DATA *GigAdapter, UINTN MicroSeconds);
#define DelayInMilliseconds(x)  DelayInMicroseconds(GigAdapter, x*1000)

#endif // _E1000_H_
