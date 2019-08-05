#  @file
#
#  Copyright (c) 2012-2013, ARM Limited. All rights reserved.
#  Copyright 2019 NXP
#
#  This program and the accompanying materials
#  are licensed and made available under the terms and conditions of the BSD License
#  which accompanies this distribution.  The full text of the license may be found at
#  http://opensource.org/licenses/bsd-license.php
#
#  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
#  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
#
#

#include <AsmMacroIoLibV8.h>
#include <AutoGen.h>

.text
.align 2

GCC_ASM_IMPORT(ArmReadMpidr)

ASM_FUNC(ArmPlatformIsPrimaryCore)
  tst x0, #3
  cset x0, eq
  ret

ASM_FUNC(ArmPlatformPeiBootAction)
  ret

ASM_FUNC(ArmPlatformGetPrimaryCoreMpId)
  MOV32 (x0, FixedPcdGet32(PcdArmPrimaryCore))
  ldrh   w0, [x0]
  ret
