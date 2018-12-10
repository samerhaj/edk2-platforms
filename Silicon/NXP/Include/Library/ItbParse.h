/** @file
  PCI memory configuration for NXP

  Copyright 2018 NXP

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php.

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS, WITHOUT
  WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __ITB_PARSE__
#define __ITB_PARSE__

#include <Uefi.h>

#define FIT_DESC_PROP      "description"
#define FIT_IMAGES_PATH    "/images"
#define FIT_KERNEL_IMAGE   "kernel"
#define FIT_INITRD_IMAGE   "ramdisk"
#define FIT_FIRMWARE_IMAGE "firmware"
#define FIT_FDT_IMAGE      "fdt"
#define FIT_CONFS_PATH     "/configurations"
#define FIT_DEFAULT_PROP   "default"
#define FIT_IMAGE_DATA     "data"
#define FIT_IMAGE_LOAD     "load"

/**
  This function retrieves that address and size from address-size pairs in Prop property
  Maximum #address-cells and #size-cells = 2 is supported.

  @param[in]  Dtb         Input Dtb File
  @param[in]  NodeOffset  Offset of Node whose property is to be read
  @param[in]  PropName    Name of property containing address and size like "reg" or "ranges" etc
  @param[in]  Index       index of address-size cell to read. it is zero based.
  @param[out] Address     Address read from property
  @param[out] Size        Size read from property

  @retval  EFI_SUCCESS    Address read successfully
  @retval  EFI_NOT_FOUND  Address not found in property
**/
EFI_STATUS
FdtGetAddressSize (
  IN  VOID*    Dtb,
  IN  INT32    NodeOffset,
  IN  CHAR8*   PropName,
  IN  INT32    Index,
  OUT UINT64   *Address,
  OUT UINT64   *Size
  );
/**

 FitCheckHeader() - Check FIT image header

 @param      FitImage            FIT Image address

 @retval     EFI_SUCCESS         If FIT Image has valid header
 @retval     EFI_UNSUPPORTED     If FIT Image has invalid heade

**/
EFI_STATUS
FitCheckHeader (
  EFI_PHYSICAL_ADDRESS FitImage
);

/**

 FitGetConfNode() - Get configuration node fron FIT image

 @param      FitImage            FIT image address
 @param      ConfigPtr           Pointer to configuration section in FIT image
 @param      NodeOffset          Upon exit, Contains offset of configuration
                                 section in FIT image.

 @retval     EFI_SUCCESS         If configuration section in FIT image found
 @retval     EFI_UNSUPPORTED     If configuration section in FIT image not found

**/
EFI_STATUS
FitGetConfNode (
  EFI_PHYSICAL_ADDRESS  FitImage,
  VOID*                 ConfigPtr,
  INT32*                NodeOffset
);

/**

 FitGetNodeFromConf() - Get offset of specified property from FIT image

 @param      FitImage          FIT image address
 @param      CfgNodeOffset     Configuration Node offset in FIT image
 @param      ConfPropName      Property Name to be searched
 @param      NodeOffset        Upon exit, Contains offset of specified property
                               in FIT image

 @retval     EFI_SUCCESS       On Success
 @retval     EFI_UNSUPPORTED   On Failure

**/
EFI_STATUS
FitGetNodeFromConf (
  EFI_PHYSICAL_ADDRESS  FitImage,
  INT32                 CfgNodeOffset,
  CHAR8*                ConfPropName,
  INT32*                NodeOffset
);

/**

 FitGetNodeData() - Get data from specified node in FIT image

 @param      FitImage          FIT image address
 @param      NodeOffset        Node offset in FIT image from where data need
                               to be copied
 @param      Addr              Address where data need to be copied
 @param      Size              Size of data which need to be copied

 @retval     EFI_SUCCESS       If data is successfully copied to Addr
 @retval     EFI_UNSUPPORTED   If no data found at specified NodeOffset

**/
EFI_STATUS
FitGetNodeData (
  EFI_PHYSICAL_ADDRESS  FitImage,
  INT32                 NodeOffset,
  EFI_PHYSICAL_ADDRESS* Addr,
  INT32*                Size
);

/**

 FitImageGetNode() - Get configuration node fron FIT image

 @param      FitImageAddr      Address of the FIT image
 @param      ImageUnitName     Target image name
 @param      NodeOffset        Pointer to area where the node offset
                               is to be returned

 @retval     EFI_SUCCESS       On Success
 @retval     EFI_UNSUPPORTED   On Failure

**/
EFI_STATUS
FitImageGetNode (
  EFI_PHYSICAL_ADDRESS FitImageAddr,
  CHAR8 *ImageUnitName,
  INT32 *NodeOffset
  );

#endif //__ITB_PARSE__
