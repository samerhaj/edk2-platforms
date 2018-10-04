/** RngDxe.c
  RNG driver to provide gEfiRngProtocolGuid support.

  Copyright 2017-2018 NXP

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/BaseMemoryLib.h>
#include <Library/BaseMemoryLib/MemLibInternals.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/SecureMonRngLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include <Protocol/Rng.h>

#define MAX_RNG_BYTES           8   // max number of bytes in a returned rng
#define NUMBER_SUPPORTED_ALGO   1   // max number of rng algorithms supported by driver

// Array of supported rng algorithms
EFI_RNG_ALGORITHM gSuppAlgoList[NUMBER_SUPPORTED_ALGO];

/**
 * Retrieves a list of the supported RNG algorithms.
 *
 * Note: We only return the type EFI_RNG_ALGORITHM_RAW
 * and a size of EFI_GUID.
 *
 * @param[in]      This                  The instance of the EFI_RNG_PROTOCOL.
 * @param[in,out]  RNGAlgorithmListSize  The size of the RNG algorithm list.
 * @param[out]     RNGAlgorithmList      List of supported RNG alogorithms.
 *
 * @retval EFI_SUCCESS            RNG algorithm list returned successfully.
 * @retval Others                 Failed to retrieve RNG alogorithm list.
**/

EFI_STATUS
GetInfo (
  IN     EFI_RNG_PROTOCOL     *This,
  IN OUT UINTN                *RNGAlgorithmListSize,
  OUT    EFI_RNG_ALGORITHM    *RNGAlgorithmList
  )
{
  EFI_STATUS    Status;
  UINTN         RequiredSize;

  Status = EFI_SUCCESS;

  if ((This == NULL) || (RNGAlgorithmListSize == NULL)
          || (RNGAlgorithmList == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  RequiredSize = sizeof (gSuppAlgoList);

  if (*RNGAlgorithmListSize < RequiredSize) {
    Status = EFI_BUFFER_TOO_SMALL;
  } else {
    //
    // Return algorithm list supported by driver.
    //
    CopyMem (RNGAlgorithmList, gSuppAlgoList, RequiredSize);
  }

  *RNGAlgorithmListSize = (sizeof (gSuppAlgoList));

  return Status;
}

/**
 * Retrieves a random number of a given size using a Secure call to access
 * the hardware RNG.
 *
 * @param[in]   This              The instance of the RNG protocol.
 * @param[in]   RNGAlgorithm      The RNG algorthm to use to generate random
 *                                number.
 * @param[in]   RNGValueLength    The length of the random number to return in
 *                                bytes.
 * @param[out]  RNGValue          The random number returned.
 *
 * @retval EFI_SUCCESS            RNG request completed successfully.
 * @retval Others                 Failed to generate a random number.
**/

EFI_STATUS
GetRNG (
  IN  EFI_RNG_PROTOCOL      *This,
  IN  EFI_RNG_ALGORITHM     *RNGAlgorithm,
  IN  UINTN                  RNGValueLength,
  OUT UINT8                 *RNGValue
  )
{
  EFI_STATUS   Status;

  Status = EFI_SUCCESS;

  DEBUG ((DEBUG_INFO, "RNGValueLength (in bytes) %d\n", RNGValueLength));

  if ((NULL != RNGValue) && (0 < RNGValueLength) &&
      (MAX_RNG_BYTES >= RNGValueLength)) {
    if ((NULL == RNGAlgorithm) ||
        (CompareGuid (RNGAlgorithm, &gEfiRngAlgorithmRaw))) {
      Status = getRawRng (RNGValueLength, RNGValue);
    } else {
      DEBUG ((DEBUG_ERROR, "Requested RNG algorithm is not supported\n"));
      Status = EFI_UNSUPPORTED;
    }
  } else {
    Status = EFI_INVALID_PARAMETER;
  }

  return Status;
}

EFI_RNG_PROTOCOL gRngProtocol = {
  GetInfo,
  GetRNG
};

/**
 * The entry point for Rng driver which installs the RNG
 * protocol on its ImageHandle.
 *
 * @param[in]  ImageHandle        The image handle of the driver.
 * @param[in]  SystemTable        The system table.
 *
 * @retval EFI_SUCCESS            If the RNG protocol installation is
 *                                successfull.
 * @retval Others                 Failed to install the protocol.
**/

EFI_STATUS
EFIAPI
RngDriverEntryPoint (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS     Status;

  Status = EFI_SUCCESS;

  // initialize the array of supported rng algorithms
  // Note: if you are going to add anything to this array, you
  //       must first increase its size
  CopyGuid (gSuppAlgoList, &gEfiRngAlgorithmRaw);

  // install the Random Number Generator Architectural Protocol
  Status = gBS->InstallMultipleProtocolInterfaces (&ImageHandle,
                                                   &gEfiRngProtocolGuid,
                                                   &gRngProtocol,
                                                   NULL
                                                   );
  return Status;
}
