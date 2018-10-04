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
#include "FirmwareManagement.h"

EFI_STATUS
GetImageInfo (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN OUT UINTN *ImageInfoSize,
  IN OUT EFI_FIRMWARE_IMAGE_DESCRIPTOR *ImageInfo,
  OUT UINT32 *DescriptorVersion,
  OUT UINT8 *DescriptorCount,
  OUT UINTN *DescriptorSize,
  OUT UINT32 *PackageVersion,
  OUT CHAR16 **PackageVersionName
  );

EFI_STATUS
FrmGetImage (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN UINT8 ImageIndex,
  IN OUT VOID *Image,
  IN OUT UINTN *ImageSize
  );

EFI_STATUS
SetImage (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN UINT8 ImageIndex,
  IN CONST VOID *Image,
  IN UINTN ImageSize,
  IN CONST VOID *VendorCode,
  IN EFI_FIRMWARE_MANAGEMENT_UPDATE_IMAGE_PROGRESS Progress,
  OUT CHAR16 **AbortReason
  );

EFI_STATUS
CheckImage (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN UINT8 ImageIndex,
  IN CONST VOID *Image,
  IN UINTN ImageSize,
  OUT UINT32 *ImageUpdatable
  );

EFI_STATUS
GetPackageInfo (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  OUT UINT32 *PackageVersion,
  OUT CHAR16 **PackageVersionName,
  OUT UINT32 *PackageVersionNameMaxLen,
  OUT UINT64 *AttributesSupported,
  OUT UINT64 *AttributesSetting
  );

EFI_STATUS
SetPackageInfo (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN CONST VOID *Image,
  IN UINTN ImageSize,
  IN CONST VOID *VendorCode,
  IN UINT32 PackageVersion,
  IN CONST CHAR16 *PackageVersionName
  );

EFI_GUID gEfiFirmwareManagementProtocolGuid = EFI_FIRMWARE_MANAGEMENT_PROTOCOL_GUID;

#define HAF_ROM_PCI_DATA_STRUCT_SIG     "PCIR"

#pragma pack(1)
typedef struct {
  UINT8   PciSignature[4];    /* PCIR */
  UINT16  VendorId;
  UINT16  DeviceId;
  UINT16  VpdOffset;          /* Pointer to optional Vital Product Data */
  UINT16  DataStructLength;
  UINT8   DataStructRev;
  UINT8   ClassCode[3];
  UINT16  ImageLength;
  UINT16  CodeRevision;
  UINT8   CodeType;
  UINT8   IndicatorByte;
  UINT16  Reserved;
} PCI_ROM_STRUCTURE;

#pragma pack(1)
typedef struct {
  UINT16  Signature;          /* 0xAA55 */
  UINT16  InitializationSize; /* Overall Size of Image */
  UINT32  EfiSignature;       /* EF1 */
  UINT16  EfiSubsystem;
  UINT16  EfiMachineType;
  UINT16  CompressionType;
  UINT8   Reserved[8];
  UINT16  ImageHeaderOffset;
  UINT16  PciOffset;          /* Pointer to the PCI ROM Data Structure */
  UINT8   Padding[2];
} EFI_ROM_HEADER;

#pragma pack(1)
typedef struct _CLP_ROM_HEADER
{
  UINT16      Signature;
  UINT8       InitializationSize;
  UINT8       Entry[4];
  UINT8       Checksum;
  UINT8       RomSignature[4];
  UINT8       LomBit;
  UINT8       reserved1;
  UINT32      pxe_offset;
  UINT8       pxe_size;
  UINT32      iscsi_offset;
  UINT8       iscsi_size;
  UINT16      PciDsOffset;
  UINT16      PnpOffset;
  UINT16      reserved2;
  UINT16      VersionOffset;
} CLP_ROM_HEADER;

#pragma pack(1)
typedef struct _PXE_ROM_HEADER
{
    UINT16      Signature;
    UINT8       InitializationSize;
    UINT8       Entry[4];
    UINT8       Checksum;
    UINT16      VendorOffset;
    UINT8       reserved[12];
    UINT16      RomIdOffset;
    UINT16      PciDsOffset;
    UINT16      PnpOffset;
    UINT16      RplOffset;
    UINT16      VersionOffset;
} PXE_ROM_HEADER;

#pragma pack(1)
typedef struct _FLB_HEADER
{
    UINT32      FlbTag;
    UINT8       Length;
    UINT8       Checksum;
    UINT32      ImageSize;
    UINT16      PciVendor;
    UINT16      PciDevice[16];
    UINT8       FileName[14];
    UINT8       MajVersion;
    UINT8       MinVersion;
    UINT8       BldNumber;
    UINT8       ImageType;
} FLB_HEADER;

#pragma pack(1)
typedef struct _PNP_ROM_HEADER
{
    UINT8       PnpSignature[4];
    UINT8       StructRev;
    UINT8       StructLength;
    UINT16      NextStruct;
    UINT8       reserved1;
    UINT8       StructCksum;
    UINT32      DeviceID;
    UINT16      MFGString;
    UINT16      ProdString;
    UINT8       BaseClass;
    UINT8       SubClass;
    UINT8       Interface;
    UINT8       DevInd;
    UINT16      BC;
    UINT16      DV;
    UINT16      BEV;
    UINT8       reserved2[2];
    UINT16      SRIV;
} PNP_ROM_HEADER;
#pragma pack()


#pragma pack()

#define FLB_TAG                       0x21424C46
#define CLP_SIG                       "$CLP"
#define PXE_ROM_SIG                   "!PXE"
#define UNDI_ROMID_SIG                "UNDI"
#define EFI_ROM_SIG                   0x0EF1
#define PCI_DATA_STRUCT_SIG           "PCIR"


#define PCI_ROM_BLOCK_SIZE              512
#define PCI_ROM_LAST_INDICATOR          0x80


//
// Macro to return the offset of a member within a struct.  This
// looks like it dereferences a null pointer, but it doesn't really.
//
#define STRUCT_OFFSET(Structure,Member)     ((UINTN)&(((Structure *)0)->Member))



EFI_STATUS
GetImageInfo (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN OUT UINTN *ImageInfoSize,
  IN OUT EFI_FIRMWARE_IMAGE_DESCRIPTOR *ImageInfo,
  OUT UINT32 *DescriptorVersion,
  OUT UINT8 *DescriptorCount,
  OUT UINTN *DescriptorSize,
  OUT UINT32 *PackageVersion,
  OUT CHAR16 **PackageVersionName
  )
/*++

  Routine Description:
    Returns information about the current firmware image(s) of the device.

  Arguments:
    This          - A pointer to the EFI_FIRMWARE_MANAGEMENT_PROTOCOL instance.
    ImageInfoSize - A pointer to the size, in bytes, of the ImageInfo buffer.
    ImageInfo     - A pointer to the buffer in which firmware places the current image(s) information.
    DescriptorVersion - A pointer to the location in which firmware returns the version number
      associated with the EFI_FIRMWARE_IMAGE_DESCRIPTOR.
    DescriptorCount - A pointer to the location in which firmware returns the number of descriptors or
      firmware images within this device.
    DescriptorSize - A pointer to the location in which firmware returns the size, in bytes,
      of an individual EFI_FIRMWARE_IMAGE_DESCRIPTOR.

  Returns:
    EFI Status code.

--*/

{
  DEBUGPRINT (IMAGE, ("GetImageInfo\n"));

  return EFI_UNSUPPORTED;
}

EFI_STATUS
FrmGetImage (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN UINT8 ImageIndex,
  IN OUT VOID *Image,
  IN OUT UINTN *ImageSize
  )
/*++

  Routine Description:
    Retrieves a copy of the firmware image from the device.

  Arguments:
    This        - A pointer to the EFI_FIRMWARE_MANAGEMENT_PROTOCOL instance.
    ImageIndex  - A unique number identifying the firmware image(s) within the device.
    Image       - Points to the buffer where the current image is copied to.
    ImageSize   -  On entry, points to the size of the buffer pointed to by
      Image, in bytes. On return, points to the length of the image, in bytes.

  Returns:
    EFI Status code.

--*/
{
  DEBUGPRINT (IMAGE, ("FrmGetImage\n"));

  return EFI_UNSUPPORTED;
}

BOOLEAN
_VerifyImageSize (
  IN GIG_UNDI_PRIVATE_DATA  *GigUndiPrivateData,
  IN UINTN ImageSize
  )
/*++

  Routine Description:
    Verifies image will fit in device flash.

  Arguments:
    GigUndiPrivateData  - Pointer to adapter structure
    ImageSize           - Size of image to flash to adapter

  Returns:
    TRUE                - image will fit on flash
    FALSE               - Image is too big

--*/
{
  UINT16      Word;
  UINTN       FlashSize;

  e1000_read_nvm (&GigUndiPrivateData->NicInfo.hw, INIT_CONTROL_WORD_2, 1, &Word);
  DEBUGPRINT (IMAGE, ("INIT_CONTROL_WORD_2 %x\n", Word));

  //
  // Calculate flash size in bytes
  //
  FlashSize = (Word & FLASH_SIZE_MASK) >> FLASH_SIZE_SHIFT;
  DEBUGPRINT (IMAGE, ("FlashSize bitmask = %x\n", FlashSize));
  FlashSize = (64 * 1024) << FlashSize;
  DEBUGPRINT (IMAGE, ("Usable FlashSize in bytes %d\n", FlashSize));

  if (FlashSize < ImageSize) {
    DEBUGPRINT (CRITICAL, ("ERROR: Flash size too small. ImageSize=%d > FlashSize=%d\n", ImageSize, FlashSize));
    return FALSE;
  }
  return TRUE;
}

BOOLEAN
_CheckFlashImageSupport (
  IN GIG_UNDI_PRIVATE_DATA *GigUndiPrivateData,
  IN UINT8                  *Image,
  IN UINTN                  ImageSize,
  OUT UINT8                 **ImageToProgram,
  OUT UINTN                 *SizeToProgram
  )
/*++

  Routine Description:
    Checks that the FLB image has support for this device ID, and makes sure the
    image type is the same as what is currently programmed on the adapter.

  Arguments:
    GigUndiPrivateData  - Pointer to adapter structure
    Image               - Pointer to the image to program
    ImageSize           - Size of image to flash to adapter
    ImageToProgram      - Returned pointer to the image to program to the flash
    SizeToProgram       - Size of the image contained in the FLB to program

  Returns:
    TRUE                - Image is compatible with device
    FALSE               - Image is not compatible

--*/
{
  EFI_ROM_HEADER    *EfiRomHeader;
  CLP_ROM_HEADER    *ClpRomHeader;
  FLB_HEADER        *FlbHeader;
  PXE_ROM_HEADER    *PxeRomHeader;
  PCI_ROM_STRUCTURE *PciRomStructure;
  UINTN             i;
  UINTN             Offset;
  UINT16            ImageType;
  UINT16            EepromCapabilities;

  Offset = 0;
  ImageType = 0;

  do {
    //
    // Check the FLB header to make sure this it supports this device
    //
    FlbHeader = (FLB_HEADER*) &Image[Offset];
    if (FlbHeader->FlbTag != FLB_TAG) {
      DEBUGPRINT (CRITICAL, ("ERROR: Invalid FLB tag.\n"));
      return FALSE;
    }


    DEBUGPRINT (IMAGE, ("FLB Device ID list: "));
    for (i = 0; i < 16; i++) {
      DEBUGPRINT (IMAGE, ("%04x ", FlbHeader->PciDevice[i]));
      if (FlbHeader->PciDevice[i] == GigUndiPrivateData->NicInfo.hw.device_id) {
        DEBUGPRINT (IMAGE, ("\nFound FLB device ID match: %04x\n", FlbHeader->PciDevice[i]));
        goto ImageFound;
      }
    }

    if (i == 16) {
       DEBUGPRINT (CRITICAL, ("\nERROR: FLB does not support device\n"));
    }

    //
    // Check for concatenated FLB image
    //
    Offset += FlbHeader->ImageSize + sizeof (FLB_HEADER);
    DEBUGPRINT (IMAGE, ("Checking for next FLB header. Curent FLB image size: %d\n", FlbHeader->ImageSize));
    if ((Offset + sizeof (FLB_HEADER)) >= ImageSize) {
      DEBUGPRINT (IMAGE, ("No concatenated FLB header, bailing.\n"));
      return FALSE;
    }
  }while (1);

ImageFound:
  Offset += sizeof(FLB_HEADER);
  *ImageToProgram = &Image[Offset];
  *SizeToProgram = FlbHeader->ImageSize;

  do {

    //
    // Check if this is a SMCLP header
    //
    ClpRomHeader = (CLP_ROM_HEADER*) &Image[Offset];
    DEBUGPRINT (IMAGE, ("Offset = %d\n", Offset));

    if (ClpRomHeader->Signature != 0xAA55) {
      DEBUGPRINT (CRITICAL, ("Invalid PCI option ROM signature: %04x\n", ClpRomHeader->Signature));
      return FALSE;
    }

    if (CompareMem(ClpRomHeader->RomSignature, CLP_SIG, 4) == 0) {
      DEBUGPRINT (IMAGE, ("Found CLP header\n"));
      ImageType |= EEPROM_SMCLP_BIT;

      if (ClpRomHeader->pxe_offset != 0) {
        DEBUGPRINT(IMAGE, ("Found PXE image\n"));
        ImageType |= EEPROM_PXE_BIT;
      }

      if (ClpRomHeader->iscsi_offset != 0) {
        DEBUGPRINT(IMAGE, ("Found iSCSI image"));
        ImageType |= EEPROM_ISCSI_BIT;
      }
    }

    //
    // Check if this is a PXE header
    //
    PxeRomHeader = (PXE_ROM_HEADER*) &Image[Offset];
    if (CompareMem((VOID*) &Image[Offset + PxeRomHeader->RomIdOffset], UNDI_ROMID_SIG, 4) == 0) {
      DEBUGPRINT (IMAGE, ("Found PXE header\n"));
      ImageType |= EEPROM_PXE_BIT;
    }

    //
    // Check if this is a EFI header
    //
    EfiRomHeader = (EFI_ROM_HEADER*) &Image[Offset];
    if (EfiRomHeader->EfiSignature == EFI_ROM_SIG) {
        DEBUGPRINT(IMAGE, ("Found EFI image\n"));
        ImageType |= EEPROM_EFI_BIT;
    }

    PciRomStructure = (PCI_ROM_STRUCTURE*) &Image[Offset + EfiRomHeader->PciOffset];
    if (CompareMem(PciRomStructure->PciSignature, PCI_DATA_STRUCT_SIG, 4) != 0) {
      DEBUGPRINT(CRITICAL, ("Invalid PCI ROM Structure.\n"));
      return FALSE;
    }

    if ((PciRomStructure->IndicatorByte & PCI_ROM_LAST_INDICATOR) == PCI_ROM_LAST_INDICATOR) {
      DEBUGPRINT(IMAGE, ("PCI last image indicator byte set.\n"));
      break;
    }

    Offset += (PciRomStructure->ImageLength * PCI_ROM_BLOCK_SIZE);
  } while (1);

  e1000_read_nvm(&GigUndiPrivateData->NicInfo.hw, EEPROM_CAPABILITIES_WORD, 1, &EepromCapabilities);
  DEBUGPRINT(IMAGE, ("EepromCapabilities = %04x, ImageType = %04x\n", EepromCapabilities, ImageType));
  if ((ImageType & EEPROM_TYPE_MASK) == (EepromCapabilities & EEPROM_TYPE_MASK)) {
    DEBUGPRINT(IMAGE, ("Imagetypes match!\n"));
    return TRUE;
  }
  DEBUGPRINT(CRITICAL, ("FLB image type different from image programmed to card!\n"));
  return FALSE;
}

VOID
_UpdateFlashFwChecksumRom(
    IN UINT8*      Buffer,
    IN UINT32      Len,
    IN UINT32      Offset
    )
/*++

  Routine Description:
    Calculates a checksum for a image

  Arguments:
    Buffer - image to checksum
    Len    - length of image
    Offset - location to put the checksum

  Returns:
    VOID

--*/
{
  UINT8   *MyBuffer;
  UINT16  Sum   = 0;
  UINT32  Count = 0;

  if (Buffer == NULL)
  {
      return;
  }

  MyBuffer = (UINT8*) Buffer;

  //
  // first, clear anything in the checksum field
  //
  MyBuffer[Offset] = 0;

  //
  // add it up
  //
  for (Count = 0; Count < Len; Count++)
  {
      Sum += MyBuffer[Count];
  }

  //
  // stuff it in
  //
  MyBuffer[Offset] = 0 - (UINT8)(Sum & 0xFF);
}


VOID
_UpdateFlashFwChecksumPnP(
  IN UINT8*      Buffer
  )
/*++

  Routine Description:
    Updates the checksum and PNP header of the image

  Arguments:
    Buffer - Pointer to the option ROM image to update

  Returns:
    VOID

--*/
{
  PXE_ROM_HEADER* RomHeader;
  PNP_ROM_HEADER* PnpHeader;

  if (Buffer == NULL)
  {
      return;
  }

  //
  // The PXE ROM header and CLP ROM header both have the PnPOffset member
  //
  RomHeader = (PXE_ROM_HEADER *)Buffer;
  if (RomHeader->PnpOffset)
  {
      DEBUGPRINT (IMAGE, ("Updating PNP checksum\n"));
      PnpHeader = (PNP_ROM_HEADER *)&Buffer[RomHeader->PnpOffset];
      _UpdateFlashFwChecksumRom(
        (UINT8 *)PnpHeader,
        PnpHeader->StructLength * 16,
        STRUCT_OFFSET(PNP_ROM_HEADER, StructCksum)
        );
  }
}

BOOLEAN
_UpdateChecksumAndId(
  IN OUT UINT8 *ImageBuffer,
  IN UINTN SizeToProgram,
  IN UINT16 DeviceId
  )
/*++

  Routine Description:
    Updates the checksum and device ID of the image.

  Arguments:
    ImageBuffer - Pointer to the option ROM image to update
    SizeToProgram - Size of image pointed to by ImageBuffer
    DeviceId - PCI device ID to set in the PCI ROM header

  Returns:
    TRUE if successful, FALSE if image is invalid

--*/
{
  EFI_ROM_HEADER    *EfiRomHeader;
  CLP_ROM_HEADER    *ClpRomHeader;
  PXE_ROM_HEADER    *PxeRomHeader;
  PCI_ROM_STRUCTURE *PciRomStructure;
  UINTN             Offset;

  Offset = 0;

  do {

    //
    // Check if this is a SMCLP header
    //
    ClpRomHeader = (CLP_ROM_HEADER*) &ImageBuffer[Offset];
    DEBUGPRINT (IMAGE, ("Offset = %d\n", Offset));

    if (ClpRomHeader->Signature != 0xAA55) {
      DEBUGPRINT (CRITICAL, ("Invalid PCI option ROM signature: %04x\n", ClpRomHeader->Signature));
      return FALSE;
    }

    if (CompareMem(ClpRomHeader->RomSignature, CLP_SIG, 4) == 0) {
      DEBUGPRINT (IMAGE, ("Found CLP header, setting device ID %04x\n", DeviceId));
      PciRomStructure = (PCI_ROM_STRUCTURE*) &ImageBuffer[Offset + ClpRomHeader->PciDsOffset];
      PciRomStructure->DeviceId = DeviceId;
      _UpdateFlashFwChecksumPnP((UINT8*) ClpRomHeader);
      _UpdateFlashFwChecksumRom(
        (UINT8*) ClpRomHeader,
        PciRomStructure->ImageLength * PCI_ROM_BLOCK_SIZE,
        STRUCT_OFFSET(CLP_ROM_HEADER, Checksum)
        );
    }

    //
    // Check if this is a PXE header
    //
    PxeRomHeader = (PXE_ROM_HEADER*) &ImageBuffer[Offset];
    if (CompareMem((VOID*) &ImageBuffer[Offset + PxeRomHeader->RomIdOffset], UNDI_ROMID_SIG, 4) == 0) {
      DEBUGPRINT (IMAGE, ("Found PXE header, setting device ID %04x\n", DeviceId));
      PciRomStructure = (PCI_ROM_STRUCTURE*) &ImageBuffer[Offset + PxeRomHeader->PciDsOffset];
      PciRomStructure->DeviceId = DeviceId;
      _UpdateFlashFwChecksumPnP((UINT8*) PxeRomHeader);
      _UpdateFlashFwChecksumRom(
        (UINT8*) PxeRomHeader,
        PciRomStructure->ImageLength * PCI_ROM_BLOCK_SIZE,
        STRUCT_OFFSET(PXE_ROM_HEADER, Checksum)
        );
    }

    //
    // Check if this is a EFI header
    //
    EfiRomHeader = (EFI_ROM_HEADER*) &ImageBuffer[Offset];
    if (EfiRomHeader->EfiSignature == EFI_ROM_SIG) {
        DEBUGPRINT(IMAGE, ("Found EFI image, setting device ID %04x\n", DeviceId));
        PciRomStructure = (PCI_ROM_STRUCTURE*) &ImageBuffer[Offset + EfiRomHeader->PciOffset];
        PciRomStructure->DeviceId = DeviceId;
    }

    PciRomStructure = (PCI_ROM_STRUCTURE*) &ImageBuffer[Offset + EfiRomHeader->PciOffset];
    if (CompareMem(PciRomStructure->PciSignature, PCI_DATA_STRUCT_SIG, 4) != 0) {
      DEBUGPRINT(CRITICAL, ("Invalid PCI ROM Structure.\n"));
      return FALSE;
    }

    if ((PciRomStructure->IndicatorByte & PCI_ROM_LAST_INDICATOR) == PCI_ROM_LAST_INDICATOR) {
      DEBUGPRINT(IMAGE, ("PCI last image indicator byte set.\n"));
      break;
    }

    Offset += (PciRomStructure->ImageLength * PCI_ROM_BLOCK_SIZE);
  } while (1);

  return TRUE;

}

EFI_STATUS
ProgramFlashImage(
  IN GIG_UNDI_PRIVATE_DATA *GigUndiPrivateData,
  UINT8                   *ImageBuffer,
  UINTN                   SizeToProgram
  )
{
  EFI_PCI_IO_PROTOCOL     *PciIoProtocol;
  EFI_STATUS              Status = EFI_SUCCESS;
  UINT32                  Reg;
  UINTN                   BarIndex;
  UINTN                   FlashOffset;
  UINTN                   i;
  UINT8                   Byte;

  PciIoProtocol = GigUndiPrivateData->NicInfo.PciIo;

  //
  // Clear the FWE bits to enable the flash erase
  //
  Reg = E1000_READ_REG(&GigUndiPrivateData->NicInfo.hw, E1000_FLASHOP);
  DEBUGPRINT(IMAGE, ("E1000_FLASHOP=%08x\n", Reg));

  DEBUGPRINT (IMAGE, ("ProgramFlashImage: Beginning flash erase\n"));
  DEBUGWAIT(IMAGE);
  e1000_ClearRegBits(&GigUndiPrivateData->NicInfo, E1000_EECD, FLASH_WRITE_MASK);
  gBS->Stall(1);
  Reg = E1000_READ_REG(&GigUndiPrivateData->NicInfo.hw, E1000_EECD);
  DEBUGPRINT(IMAGE, ("E1000_EECD=%08x\n", Reg));

  e1000_SetRegBits(&GigUndiPrivateData->NicInfo, E1000_FLA, E1000_FLA_FL_ER);
  Reg = E1000_READ_REG(&GigUndiPrivateData->NicInfo.hw, E1000_FLA);
  DEBUGPRINT(IMAGE, ("E1000_FLA=%08x\n", Reg));
  gBS->Stall(1);
 DEBUGPRINT (IMAGE, ("Wait for flash erase completion\n"));

  i = 0;
  do {
    gBS->Stall(1000);
    Reg = E1000_READ_REG(&GigUndiPrivateData->NicInfo.hw, E1000_FLA);
    i++;
    if (i > 20000) {
      DEBUGPRINT (IMAGE, ("Flash erase failed, E1000_FLA=%08x\n", Reg));
      return EFI_DEVICE_ERROR;
    }
  } while ((Reg & E1000_FLA_FL_BUSY) != 0);
  DEBUGPRINT (IMAGE, ("Flash erase complete\n"));

  //
  // Enable flash writes
  //
  Reg = E1000_READ_REG(&GigUndiPrivateData->NicInfo.hw, E1000_EECD);
  Reg = (Reg & ~FLASH_WRITE_MASK) | FLASH_WRITE_ENABLE;
  E1000_WRITE_REG(&GigUndiPrivateData->NicInfo.hw, E1000_EECD, Reg);

  do {
    Reg = E1000_READ_REG(&GigUndiPrivateData->NicInfo.hw, E1000_FLA);
  } while ((Reg & E1000_FLA_FL_BUSY) != 0);

  //
  // Begin writing to the flash.
  //
  BarIndex = 1;   // On pre-82580 the flash is memory mapped to PCI BAR 1
  FlashOffset = 0;
  if (GigUndiPrivateData->NicInfo.hw.mac.type == e1000_82580) {
    BarIndex = 0;   // On 82580 the flash is memory mapped to PCI BAR 0
    FlashOffset = FLASH_OFFSET_82580;
  }

  DEBUGPRINT (IMAGE, ("Beginning flash write\n"));
  //
  // On 82580 the flash is memory mapped after the devices general register set in BAR 0
  //
  for (i = 0; i < SizeToProgram; i++) {

    PciIoProtocol->Mem.Write (
                      PciIoProtocol,
                      EfiPciIoWidthUint8,
                      BarIndex,
                      i + FlashOffset,
                      1,
                      (VOID *) &ImageBuffer[i]
                      );

    do {
      Reg = E1000_READ_REG(&GigUndiPrivateData->NicInfo.hw, E1000_FLA);
    } while ((Reg & E1000_FLA_FL_BUSY) != 0);

    PciIoProtocol->Mem.Read (
                      PciIoProtocol,
                      EfiPciIoWidthUint8,
                      BarIndex,
                      i + FlashOffset,
                      1,
                      (VOID *) &Byte
                      );

    //
    // If the value read back does not match the value written, erase the flash so we do
    // not leave a bad image on the card, and return an error.
    //
    if (ImageBuffer[i] != Byte) {
      DEBUGPRINT (CRITICAL, ("Error writing flash at offset %x.  Wrote=%02x, Read=%02x\n", i, ImageBuffer[i], Byte));
      e1000_ClearRegBits(&GigUndiPrivateData->NicInfo, E1000_EECD, FLASH_WRITE_MASK);
      e1000_SetRegBits(&GigUndiPrivateData->NicInfo, E1000_FLA, E1000_FLA_FL_ER);

      do {
        Reg = E1000_READ_REG(&GigUndiPrivateData->NicInfo.hw, E1000_FLA);
      } while ((Reg & E1000_FLA_FL_BUSY) != 0);
      DEBUGPRINT (CRITICAL, ("Flash erase complete\n"));

      Status = EFI_DEVICE_ERROR;
      break;
    }
  }

  gBS->FreePool(ImageBuffer);

  DEBUGPRINT (IMAGE, ("Flash write complete\n"));

  //
  // Disable flash writes
  //
  Reg = E1000_READ_REG(&GigUndiPrivateData->NicInfo.hw, E1000_EECD);
  Reg = (Reg & ~FLASH_WRITE_MASK) | FLASH_WRITE_DISABLE;
  E1000_WRITE_REG(&GigUndiPrivateData->NicInfo.hw, E1000_EECD, Reg);

  return Status;
}


EFI_STATUS
SetImage (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN UINT8 ImageIndex,
  IN CONST VOID *Image,
  IN UINTN ImageSize,
  IN CONST VOID *VendorCode,
  IN EFI_FIRMWARE_MANAGEMENT_UPDATE_IMAGE_PROGRESS Progress,
  OUT CHAR16 **AbortReason
  )
/*++

  Routine Description:
    Updates the firmware image of the device.

  Arguments:
    This    - A pointer to the EFI_FIRMWARE_MANAGEMENT_PROTOCOL instance.
    ImageIndex - A unique number identifying the firmware image(s) within the device. The number is
      between 1 and DescriptorCount.
    Image - Points to the new image.
    ImageSize - Size of the new image in bytes.
    VendorCode - This enables vendor to implement vendor-specific firmware image update policy.
    Progress - A function used by the driver to report the progress of the firmware update.
    AbortReason - A pointer to a pointer to a null-terminated Unicode string providing more
      details for the aborted operation. The buffer is allocated by this function with
      AllocatePool(), and it is the caller?s responsibility to free it with a call to FreePool().

  Returns:
    EFI Status code.

--*/
{
  GIG_UNDI_PRIVATE_DATA   *GigUndiPrivateData;
  UINT8                   *ImageToProgram;
  UINTN                   SizeToProgram;
  BOOLEAN                 Supported;
  UINT8                   *ImageBuffer;
  EFI_STATUS              Status;

  ImageToProgram = NULL;
  SizeToProgram = 0;
  GigUndiPrivateData = GIG_UNDI_PRIVATE_DATA_FROM_FIRMWARE_MANAGEMENT(This);

  Supported = _CheckFlashImageSupport (
                GigUndiPrivateData,
                (UINT8*) Image,
                ImageSize,
                &ImageToProgram,
                &SizeToProgram
                );

  if (!Supported) {
    if (gBS->AllocatePool (EfiBootServicesData, 80, (VOID **) AbortReason) == EFI_SUCCESS) {
      UnicodeSPrint(*AbortReason, 80, L"ERROR: Incorrect image type for adapter");
    }
    return EFI_INVALID_PARAMETER;
  }

  DEBUGPRINT (IMAGE, ("ImageToProgram = %02X %02X, ImageSize %d\n", ImageToProgram[0], ImageToProgram[1], SizeToProgram));

  //
  // Verify the image will fit in flash
  //
  if (_VerifyImageSize ( GigUndiPrivateData, SizeToProgram) == FALSE) {
    if (gBS->AllocatePool (EfiBootServicesData, 80, (VOID **) AbortReason) == EFI_SUCCESS) {
      UnicodeSPrint(*AbortReason, 80, L"ERROR: Image too large for adapter flash");
    }
    return EFI_INVALID_PARAMETER;
  }

  //
  // Allocate malleable buffer to copy image into so we can set checksum and device ID
  // in image headers
  //
  Status = gBS->AllocatePool (EfiBootServicesData, ImageSize, (VOID **) &ImageBuffer);
  if (EFI_ERROR(Status)) {
    DEBUGPRINT (CRITICAL, ("AllocatePool failed %r\n", Status));
    return Status;
  }

  CopyMem (ImageBuffer, ImageToProgram, SizeToProgram);

  if (_UpdateChecksumAndId(ImageBuffer, SizeToProgram, GigUndiPrivateData->NicInfo.hw.device_id) == FALSE) {
    DEBUGPRINT (CRITICAL, ("_UpdateChecksumAndId failed.\n"));
    return EFI_INVALID_PARAMETER;
  }

  Status = ProgramFlashImage(GigUndiPrivateData, ImageBuffer, SizeToProgram);

  if (Status == EFI_DEVICE_ERROR) {
    if ( gBS->AllocatePool (EfiBootServicesData, 80, (VOID **) AbortReason)== EFI_SUCCESS ) {
      UnicodeSPrint(*AbortReason, 80, (L"ERROR: flash write failed"));
    }
  }

  return Status;
}


EFI_STATUS
CheckImage (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN UINT8 ImageIndex,
  IN CONST VOID *Image,
  IN UINTN ImageSize,
  OUT UINT32 *ImageUpdatable
  )
/*++

  Routine Description:
    Checks if the firmware image is valid for the device.

  Arguments:
    This        - A pointer to the EFI_FIRMWARE_MANAGEMENT_PROTOCOL instance.
    ImageIndex  - A unique number identifying the firmware image(s) within the device. The number is between 1 and DescriptorCount.
    Image       - Points to the new image.
    ImageSize   - Size of the new image in bytes.
    ImageUpdatable - Indicates if the new image is valid for update. It also provides, if available, additional information if the
      image is invalid.

  Returns:
    EFI Status code.

--*/
{
  GIG_UNDI_PRIVATE_DATA  *GigUndiPrivateData;
  UINT8                   *ImageToProgram;
  UINTN                   SizeToProgram;
  BOOLEAN                 Supported;

  ImageToProgram = NULL;
  SizeToProgram = 0;
  GigUndiPrivateData = GIG_UNDI_PRIVATE_DATA_FROM_FIRMWARE_MANAGEMENT(This);

  DEBUGPRINT (IMAGE, ("CheckImage\n"));

  Supported = _CheckFlashImageSupport (
                GigUndiPrivateData,
                (UINT8*) Image,
                ImageSize,
                &ImageToProgram,
                &SizeToProgram
                );

  if (!Supported) {
    DEBUGPRINT (IMAGE, ("Image invalid.\n"));
    *ImageUpdatable = IMAGE_UPDATABLE_INVALID;
    return EFI_INVALID_PARAMETER;
  }

  DEBUGPRINT (IMAGE, ("Image valid.\n"));
  *ImageUpdatable = IMAGE_UPDATABLE_VALID;
  return EFI_SUCCESS;
}

EFI_STATUS
GetPackageInfo (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  OUT UINT32 *PackageVersion,
  OUT CHAR16 **PackageVersionName,
  OUT UINT32 *PackageVersionNameMaxLen,
  OUT UINT64 *AttributesSupported,
  OUT UINT64 *AttributesSetting
  )
/*++

  Routine Description:
    Returns information about the firmware package.

  Arguments:
  This - A pointer to the EFI_FIRMWARE_MANAGEMENT_PROTOCOL instance.
  PackageVersion - A version number that represents all the firmware images in the device.
  PackageVersionName - A pointer to a pointer to a null-terminated Unicode string
    representing the package version name.
  PackageVersionNameMaxLen - The maximum length of package version name if device supports
    update of package version name.
  AttributesSupported - Package attributes that are supported by this device. See ?Package
    Attribute Definitions? for possible returned values of this parameter. A value of 1
    indicates the attribute is supported and the current setting value is indicated in
    AttributesSetting.
  AttributesSetting - Package attributes. See ?Package Attribute Definitions? for possible
    returned values of this parameter.

  Returns:
  EFI_SUCCESS     - The package information was successfully returned.
  EFI_UNSUPPORTED - The operation is not supported.

--*/
{
  DEBUGPRINT (IMAGE, ("GetPackageInfo\n"));

  return EFI_UNSUPPORTED;
}

EFI_STATUS
SetPackageInfo (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN CONST VOID *Image,
  IN UINTN ImageSize,
  IN CONST VOID *VendorCode,
  IN UINT32 PackageVersion,
  IN CONST CHAR16 *PackageVersionName
  )
/*++

  Routine Description:
    Updates information about the firmware package.

  Arguments:
    This - A pointer to the EFI_FIRMWARE_MANAGEMENT_PROTOCOL instance.
    Image - Points to the authentication image. Null if authentication is not required.
    ImageSize - Size of the authentication image in bytes. 0 if authentication is not required.
    VendorCode - This enables vendor to implement vendor-specific firmware image update policy.
      Null indicates the caller did not specify this policy or use the default policy.
    PackageVersion - The new package version.
    PackageVersionName - A pointer to the new null-terminated Unicode string representing the
      package version name. The string length is equal to or less than the value returned in
      PackageVersionNameMaxLen.

  Returns:
    EFI_SUCCESS - The device was successfully updated with the new package information
    EFI_INVALID_PARAMETER - The PackageVersionName length is longer than the value returned
      in PackageVersionNameMaxLen.
    EFI_UNSUPPORTED - The operation is not supported.
    EFI_SECURITY_VIOLATION - The operation could not be performed due to an authentication
      failure.
--*/
{
  DEBUGPRINT (IMAGE, ("SetPackageInfo\n"));

  return EFI_UNSUPPORTED;

}

EFI_STATUS
InitFirmwareManagementProtocol (
  IN GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
  )
/*++

  Routine Description:
    Installs the Firmware Management protocol on the device handle.

  Arguments:
    XgbeUndiPrivateData  - Points to the driver instance private data.

  Returns:
    EFI_SUCCESS if Firmware Management protocol interface installed
    correctly, otherwise EFI error code is returned.

--*/
{
  EFI_STATUS Status;

  DEBUGPRINT (IMAGE, ("InitFirmwareManagementProtocol\n"));

  GigUndiPrivateData->FirmwareManagement.GetImageInfo    = GetImageInfo;
  GigUndiPrivateData->FirmwareManagement.GetImage        = FrmGetImage;
  GigUndiPrivateData->FirmwareManagement.SetImage        = SetImage;
  GigUndiPrivateData->FirmwareManagement.CheckImage      = CheckImage;
  GigUndiPrivateData->FirmwareManagement.GetPackageInfo  = GetPackageInfo;
  GigUndiPrivateData->FirmwareManagement.SetPackageInfo  = SetPackageInfo;

  Status = gBS->InstallProtocolInterface (
                  &GigUndiPrivateData->DeviceHandle,
                  &gEfiFirmwareManagementProtocolGuid,
                  EFI_NATIVE_INTERFACE,
                  &GigUndiPrivateData->FirmwareManagement
                  );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("InstallProtocolInterface error: %r\n", Status));
    return Status;
  }
  DEBUGPRINT (IMAGE, ("Install Firmware Management Protocol success: %r\n", Status));

  return Status;
}


EFI_STATUS
UninstallFirmwareManagementProtocol (
  IN GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
  )
/*++

  Routine Description:
    Installs the Firmware Management protocol on the device handle.

  Arguments:
    XgbeUndiPrivateData  - Points to the driver instance private data.

  Returns:
    EFI_SUCCESS if Firmware Management protocol interface installed
    correctly, otherwise EFI error code is returned.

--*/
{
  EFI_STATUS Status;

  DEBUGPRINT (IMAGE, ("InitFirmwareManagementProtocol\n"));

  Status = gBS->UninstallProtocolInterface (
                  GigUndiPrivateData->DeviceHandle,
                  &gEfiFirmwareManagementProtocolGuid,
                  &GigUndiPrivateData->FirmwareManagement
                  );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("UninstallProtocolInterface error: %r\n", Status));
    return Status;
  }
  DEBUGPRINT (IMAGE, ("Uninstall Firmware Management Protocol success: %r\n", Status));

  return Status;
}



