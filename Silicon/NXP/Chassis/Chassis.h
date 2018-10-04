/** @file
*  Header defining the Base addresses, sizes, flags etc for chassis 1
*
*  Copyright 2017 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#define TP_ITYP_AV_MASK            0x00000001  /* Initiator available */
#define TP_ITYP_TYPE_MASK(x)       (((x) & 0x6) >> 1) /* Initiator Type */
#define TP_ITYP_TYPE_ARM           0x0
#define TP_ITYP_TYPE_PPC           0x1
#define TP_ITYP_TYPE_OTHER         0x2  /* StarCore DSP */
#define TP_ITYP_TYPE_HA            0x3  /* HW Accelerator */
#define TP_ITYP_THDS(x)            (((x) & 0x18) >> 3)  /* # threads */
#define TP_ITYP_VERSION(x)         (((x) & 0xe0) >> 5)  /* Initiator Version */
#define TP_CLUSTER_INIT_MASK       0x0000003f  /* initiator mask */
#define TP_INIT_PER_CLUSTER        4

#define TY_ITYP_VERSION_A7         0x1
#define TY_ITYP_VERSION_A53        0x2
#define TY_ITYP_VERSION_A57        0x3
#define TY_ITYP_VERSION_A72        0x4

STATIC
inline
UINTN
CpuMaskNext (
  IN  UINTN  Cpu,
  IN  UINTN  Mask
  )
{
  for (Cpu++; !((1 << Cpu) & Mask); Cpu++)
    ;

  return Cpu;
}

#define ForEachCpu(Iter, Cpu, NumCpus, Mask) \
  for (Iter = 0, Cpu = CpuMaskNext(-1, Mask); \
    Iter < NumCpus; \
    Iter++, Cpu = CpuMaskNext(Cpu, Mask)) \

#define CPU_TYPE_ENTRY(N, V, NC) \
           { .Name = #N, .SocVer = SVR_##V, .NumCores = (NC)}

#define SVR_WO_E                    0xFFFFFE
#define SVR_LS1043A                 0x879200
#define SVR_LS1046A                 0x870700
#define SVR_LS2088A                 0x870901
#define SVR_LX2160A                 0x873601

#define SVR_MAJOR(svr)              (((svr) >> 4) & 0xf)
#define SVR_MINOR(svr)              (((svr) >> 0) & 0xf)
#define SVR_SOC_VER(svr)            (((svr) >> 8) & SVR_WO_E)
#define IS_E_PROCESSOR(svr)         (!((svr >> 8) & 0x1))

#define MEGA_HZ                     1000000

typedef struct {
  CHAR8  Name[16];
  UINT32 SocVer;
  UINT32 NumCores;
} CPU_TYPE;

typedef struct {
  UINTN CpuClk;  /* CPU clock in Hz! */
  UINTN BusClk;
  UINTN MemClk;
  UINTN PciClk;
  UINTN SdhcClk;
} SOC_CLOCK_INFO;

/*
 * Print Soc information
 */
VOID
PrintSoc (
  VOID
  );

/*
 * Initialize Clock structure
 */
VOID
ClockInit (
  VOID
  );

/*
 * Setup SMMU in bypass mode
 * and also set its pagesize
 */
VOID
SmmuInit (
  VOID
  );

/*
 * Print CPU information
 */
VOID
PrintCpuInfo (
  VOID
  );

/*
 * Dump RCW (Reset Control Word) on console
 */
VOID
PrintRCW (
  VOID
  );

UINT32
InitiatorType (
  IN UINT32 Cluster,
  IN UINTN InitId
  );

/*
 *  Return the mask for number of cores on this SOC.
 */
UINT32
CpuMask (
  VOID
  );

/*
 *  Return the number of cores on this SOC.
 */
UINTN
CpuNumCores (
  VOID
  );

/*
 * Return the type of initiator for core/hardware accelerator for given core index.
 */
UINT32
QoriqCoreToType (
  IN UINTN Core
  );

/*
 *  Return the cluster of initiator for core/hardware accelerator for given core index.
 */
UINT32
QoriqCoreToCluster (
  IN UINTN Core
  );

#endif /* __CHASSIS_H__ */
