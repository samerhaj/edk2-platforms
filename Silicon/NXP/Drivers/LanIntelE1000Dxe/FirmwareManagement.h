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

#ifndef _FRMMGMT_H_
#define _FRMMGMT_H_

#include "e1000.h"

#define EFI_FIRMWARE_MANAGEMENT_PROTOCOL_GUID \
{ 0x86c77a67, 0xb97, 0x4633, { 0xa1, 0x87, 0x49, 0x10, \
0x4d, 0x6, 0x85, 0xc7 } }

//**************************************************************
// Image Attribute Definitions
//**************************************************************
#define IMAGE_ATTRIBUTE_IMAGE_UPDATABLE         0x0000000000000001
#define IMAGE_ATTRIBUTE_RESET_REQUIRED          0x0000000000000002
#define IMAGE_ATTRIBUTE_AUTHENTICATION_REQUIRED 0x0000000000000004
#define IMAGE_ATTRIBUTE_IN_USE                  0x0000000000000008

//**************************************************************
// Image Compatibility Definitions
//**************************************************************
#define IMAGE_COMPATIBILITY_CHECK_SUPPORTED 0x0000000000000001

//**************************************************************
// Package Attribute Definitions
//**************************************************************
#define PACKAGE_ATTRIBUTE_VERSION_UPDATABLE 0x0000000000000001
#define PACKAGE_ATTRIBUTE_RESET_REQUIRED 0x0000000000000002
#define PACKAGE_ATTRIBUTE_AUTHENTICATION_REQUIRED 0x0000000000000004

//**************************************************************
// ImageUpdatable Definitions
//**************************************************************
#define IMAGE_UPDATABLE_VALID         0x0000000000000001
#define IMAGE_UPDATABLE_INVALID       0x0000000000000002
#define IMAGE_UPDATABLE_INVALID_TYPE  0x0000000000000004
#define IMAGE_UPDATABLE_INVALID_OLD   0x0000000000000008

//**************************************************************
// Descriptor Version
//**************************************************************
#define EFI_FIRMWARE_IMAGE_DESCRIPTOR_VERSION 1

#define E1000_FLASHOP     0x103C

//
// EEC register FLASH commands
//
#define FLASH_WRITE_DISABLE         0x0010
#define FLASH_WRITE_ENABLE          0x0020
#define FLASH_WRITE_ERASE           0x0000
#define FLASH_WRITE_MASK            0x0030

//
// Flash Access Register
//
#define E1000_FLA_FL_SCK   0x00000001 /* Serial Flash Clock */
#define E1000_FLA_FL_CS    0x00000002 /* Serial Flash Chip Select (CS) */
#define E1000_FLA_FL_SI    0x00000004 /* Serial Flash Serial Data In */
#define E1000_FLA_FL_SO    0x00000008 /* Serial Flash Serial Data Out */
#define E1000_FLA_FL_REQ   0x00000010 /* Request flash access */
#define E1000_FLA_FL_GNT   0x00000020 /* Grant flash access */
#define E1000_FLA_FL_BUSY  0x40000000 /* Flash busy */
#define E1000_FLA_FL_ER    0x80000000 /* Flash erase */

//
// Start of flash memory in memory mapped IO BAR on 82580
//
#define FLASH_OFFSET_82580 (128 * 1024)

//**************************************************************
// Image Attribute ? Authentication Required
//**************************************************************
//typedef struct {
//  UINT64 MonotonicCount;
//  WIN_CERTIFICATE_UEFI_GUID AuthInfo;
//} EFI_FIRMWARE_IMAGE_AUTHENTICATION;

//**************************************************************
// EFI_FIRMWARE_IMAGE_DESCRIPTOR
//**************************************************************
typedef struct {
  UINT8 ImageIndex;
  EFI_GUID ImageTypeId;
  UINT64 ImageId;
  CHAR16 *ImageIdName;
  UINT32 Version;
  CHAR16 *VersionName;
  UINTN Size;
  UINT64 AttributesSupported;
  UINT64 AttributesSetting;
  UINT64 Compatibilities;
} EFI_FIRMWARE_IMAGE_DESCRIPTOR;

//
// Anonymous declarations to get the rest of the file to compile
//
typedef struct _EFI_FIRMWARE_MANAGEMENT_PROTOCOL  EFI_FIRMWARE_MANAGEMENT_PROTOCOL;

typedef
EFI_STATUS
(EFIAPI *EFI_FIRMWARE_MANAGEMENT_PROTOCOL_GET_IMAGE_INFO) (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN OUT UINTN *ImageInfoSize,
  IN OUT EFI_FIRMWARE_IMAGE_DESCRIPTOR *ImageInfo,
  OUT UINT32 *DescriptorVersion,
  OUT UINT8 *DescriptorCount,
  OUT UINTN *DescriptorSize,
  OUT UINT32 *PackageVersion,
  OUT CHAR16 **PackageVersionName
  );

typedef
EFI_STATUS
(EFIAPI *EFI_FIRMWARE_MANAGEMENT_PROTOCOL_GET_IMAGE) (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN UINT8 ImageIndex,
  IN OUT VOID *Image,
  IN OUT UINTN *ImageSize
  );

typedef
EFI_STATUS
(EFIAPI *EFI_FIRMWARE_MANAGEMENT_UPDATE_IMAGE_PROGRESS) (
  IN UINTN Completion
  );

typedef
EFI_STATUS
(EFIAPI *EFI_FIRMWARE_MANAGEMENT_PROTOCOL_SET_IMAGE) (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN UINT8 ImageIndex,
  IN CONST VOID *Image,
  IN UINTN ImageSize,
  IN CONST VOID *VendorCode,
  IN EFI_FIRMWARE_MANAGEMENT_UPDATE_IMAGE_PROGRESS Progress,
  OUT CHAR16 **AbortReason
  );

typedef
EFI_STATUS
(EFIAPI *EFI_FIRMWARE_MANAGEMENT_PROTOCOL_CHECK_IMAGE) (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN UINT8 ImageIndex,
  IN CONST VOID *Image,
  IN UINTN ImageSize,
  OUT UINT32 *ImageUpdatable
  );

typedef
EFI_STATUS
(EFIAPI *EFI_FIRMWARE_MANAGEMENT_PROTOCOL_GET_PACKAGE_INFO) (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  OUT UINT32 *PackageVersion,
  OUT CHAR16 **PackageVersionName,
  OUT UINT32 *PackageVersionNameMaxLen,
  OUT UINT64 *AttributesSupported,
  OUT UINT64 *AttributesSetting
  );

typedef
EFI_STATUS
(EFIAPI *EFI_FIRMWARE_MANAGEMENT_PROTOCOL_SET_PACKAGE_INFO) (
  IN EFI_FIRMWARE_MANAGEMENT_PROTOCOL *This,
  IN CONST VOID *Image,
  IN UINTN ImageSize,
  IN CONST VOID *VendorCode,
  IN UINT32 PackageVersion,
  IN CONST CHAR16 *PackageVersionName
  );

typedef struct _EFI_FIRMWARE_MANAGEMENT_PROTOCOL {
  EFI_FIRMWARE_MANAGEMENT_PROTOCOL_GET_IMAGE_INFO GetImageInfo;
  EFI_FIRMWARE_MANAGEMENT_PROTOCOL_GET_IMAGE GetImage;
  EFI_FIRMWARE_MANAGEMENT_PROTOCOL_SET_IMAGE SetImage;
  EFI_FIRMWARE_MANAGEMENT_PROTOCOL_CHECK_IMAGE CheckImage;
  EFI_FIRMWARE_MANAGEMENT_PROTOCOL_GET_PACKAGE_INFO GetPackageInfo;
  EFI_FIRMWARE_MANAGEMENT_PROTOCOL_SET_PACKAGE_INFO SetPackageInfo;
} EFI_FIRMWARE_MANAGEMENT_PROTOCOL;



#endif /* _FRMMGMT_H_ */

