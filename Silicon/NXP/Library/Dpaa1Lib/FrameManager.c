/** @file
  DPAA Frame manager implementation

  Copyright (c) 2016, Freescale Semiconductor, Inc. All rights reserved.

  This Program And The Accompanying Materials
  Are Licensed And Made Available Under The Terms And Conditions Of The BSD
  License Which Accompanies This Distribution. The Full Text Of The License
  May Be Found At
  http://Opensource.Org/Licenses/Bsd-License.Php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Bitops.h>
#include <Library/BaseMemoryLib/MemLibInternals.h>
#include <Library/DebugLib.h>
#include <Library/Dpaa1Lib.h>
#include <Library/Dpaa1DebugLib.h>
#include <Library/Dpaa1EthernetMacLib.h>
#include <Library/FrameManager.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiBootServicesTableLib.h>

#define FMAN_FW_VERSION     1

//
// FMan controller configuration address auto increment enable
//
#define FMAN_CONTROLLER_ADDR_AIE_MASK   BIT(31)

//
// FMan controller configuration data ready flag
//
#define FMAN_CONTROLLER_DATA_READY_MASK BIT(31)

//
// FMan controller write timeout
//
#define FMAN_CONTROLLER_WRITE_TIMEOUT  1000000u

//
// Private memory area for each FMan instance
//
STATIC FMAN_MEMORY gFmanMem[NUM_FMAN];

STATIC CONST  EFI_LOCK TxLock = EFI_INITIALIZE_LOCK_VARIABLE(TPL_CALLBACK);
STATIC CONST  EFI_LOCK RxLock = EFI_INITIALIZE_LOCK_VARIABLE(TPL_CALLBACK);

/**
   Computes the CRC-32 used for firmware images

   @param[in]     Data      pointer to the memory block
   @param[in]     DataSize  block size in bytes
                  memory

   @retval Computed CRC

 **/
UINT32
ComputeCrc32(
  IN  CONST VOID    *Data,
  IN  UINTN         DataSize
  )
{

  STATIC CONST UINT32 CrcTable[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F,
    0xE963A535, 0x9E6495A3, 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91, 0x1DB71064, 0x6AB020F2,
    0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9,
    0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C,
    0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423,
    0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
    0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D, 0x76DC4190, 0x01DB7106,
    0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D,
    0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950,
    0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7,
    0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9, 0x5005713C, 0x270241AA,
    0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81,
    0xB7BD5C3B, 0xC0BA6CAD, 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
    0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683, 0xE3630B12, 0x94643B84,
    0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB,
    0x196C3671, 0x6E6B06E7, 0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
    0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5, 0xD6D6A3E8, 0xA1D1937E,
    0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55,
    0x316E8EEF, 0x4669BE79, 0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
    0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F, 0xC5BA3BBE, 0xB2BD0B28,
    0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F,
    0x72076785, 0x05005713, 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
    0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21, 0x86D3D2D4, 0xF1D4E242,
    0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69,
    0x616BFFD3, 0x166CCF45, 0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
    0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB, 0xAED16A4A, 0xD9D65ADC,
    0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693,
    0x54DE5729, 0x23D967BF, 0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
    0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
  };

  CONST UINT32 *WordPtr = Data;
  UINT32 Crc = 0x0;
  UINTN WordsLeft;

  ASSERT(DataSize % sizeof(UINT32) == 0);

  for (WordsLeft = DataSize / sizeof(UINT32); WordsLeft != 0; --WordsLeft) {
    Crc ^= *WordPtr++;
    Crc = CrcTable[Crc & 0xff] ^ (Crc >> 8);
    Crc = CrcTable[Crc & 0xff] ^ (Crc >> 8);
    Crc = CrcTable[Crc & 0xff] ^ (Crc >> 8);
    Crc = CrcTable[Crc & 0xff] ^ (Crc >> 8);
  }

  return Crc;
}

UINT16 MemReadMasked (
  IN  UINT16 *Addr
  )
{
  UINTN Base;
  UINT32 Val;
  INT32 Pos;
  UINT16 Ret;

  Base = (UINTN)Addr & ~0x3UL;
  Val = MmioReadBe32(Base);

  Pos = (UINTN)Addr & 0x3UL;
  if (Pos)
    Ret = (UINT16)(Val & MASK_UPPER_16);
  else
    Ret = (UINT16)((Val & MASK_LOWER_16) >> 16);

  return Ret;
}

VOID MemWriteMasked (
  IN  UINT16 *Addr, 
  IN  UINT16 Data
  )
{
  UINTN Base = (UINTN)Addr & ~0x3UL;
  UINT32 Org = MmioReadBe32(Base);
  UINT32 Val;
  INT32 Pos;

  Pos = (UINT64)Addr & 0x3;
  if (Pos)
    Val = (Org & MASK_LOWER_16) | Data;
  else
    Val = (Org & MASK_UPPER_16) | ((UINT32)Data << 16);

  MmioWriteBe32(Base, Val);
}

VOID
DumpFmanCfg (
  IN VOID
  )
{ 
  FMAN_CCSR *FmanRegs = (FMAN_CCSR *)PcdGet64(PcdDpaa1FmanAddr);
  FMAN_CONTROLLER_CONFIG *Cfg = &FmanRegs->FmanControllerCfg;

  DPAA1_DEBUG_MSG(" ------------- DUMP CFG 0x%x -------------\n", Cfg);
  DPAA1_DEBUG_MSG("Address      : 0x%x \n", MmioReadBe32((UINTN)&Cfg->Address      ));
  DPAA1_DEBUG_MSG("Data         : 0x%x \n", MmioReadBe32((UINTN)&Cfg->Data         ));
  DPAA1_DEBUG_MSG("TimingConfig : 0x%x \n", MmioReadBe32((UINTN)&Cfg->TimingConfig ));
  DPAA1_DEBUG_MSG("DataReady    : 0x%x \n", MmioReadBe32((UINTN)&Cfg->DataReady    ));
}

VOID
DumpFmanFpm (
  IN VOID
  )
{
  FMAN_CCSR *FmanRegs = (FMAN_CCSR *)PcdGet64(PcdDpaa1FmanAddr);
  FMAN_FPM *Fpm = &FmanRegs->FmanFpm;
  UINT32 I = 0;

  DPAA1_DEBUG_MSG(" ------------- DUMP FPM 0x%x -------------\n", Fpm);
  DPAA1_DEBUG_MSG("Tnc  : 0x%x \n", MmioReadBe32((UINTN)&Fpm->Tnc  ));
  DPAA1_DEBUG_MSG("Prc  : 0x%x \n", MmioReadBe32((UINTN)&Fpm->Prc  ));
  DPAA1_DEBUG_MSG("Flc  : 0x%x \n", MmioReadBe32((UINTN)&Fpm->Flc  ));
  DPAA1_DEBUG_MSG("Dist1: 0x%x \n", MmioReadBe32((UINTN)&Fpm->Dist1));
  DPAA1_DEBUG_MSG("Dist2: 0x%x \n", MmioReadBe32((UINTN)&Fpm->Dist2));
  DPAA1_DEBUG_MSG("Epi  : 0x%x \n", MmioReadBe32((UINTN)&Fpm->Epi  ));
  DPAA1_DEBUG_MSG("Rie  : 0x%x \n", MmioReadBe32((UINTN)&Fpm->Rie  ));
  for (I =0; I < 0x4; I++)
    DPAA1_DEBUG_MSG("Fcev[%d]  : 0x%x \n",I, MmioReadBe32((UINTN)&Fpm->Fcev[I]));

  for (I =0; I < 0x4; I++)
    DPAA1_DEBUG_MSG("Fcmask[%d]  : 0x%x \n",I, MmioReadBe32((UINTN)&Fpm->Fcmask[I]));

  DPAA1_DEBUG_MSG("Tsc1 : 0x%x \n", MmioReadBe32((UINTN)&Fpm->Tsc1));
  DPAA1_DEBUG_MSG("Tsc2  : 0x%x \n", MmioReadBe32((UINTN)&Fpm->Tsc2));
  DPAA1_DEBUG_MSG("Tsp  : 0x%x \n", MmioReadBe32((UINTN)&Fpm->Tsp ));
  DPAA1_DEBUG_MSG("Tsf  : 0x%x \n", MmioReadBe32((UINTN)&Fpm->Tsf ));
  DPAA1_DEBUG_MSG("Rcr  : 0x%x \n", MmioReadBe32((UINTN)&Fpm->Rcr ));

  for (I =0; I < 0x4; I++)
    DPAA1_DEBUG_MSG("Drd[%d]  : 0x%x \n", I, MmioReadBe32((UINTN)&Fpm->Drd[I]));

  DPAA1_DEBUG_MSG("Dra    : 0x%x \n", MmioReadBe32((UINTN)&Fpm->Dra   ));
  DPAA1_DEBUG_MSG("IpRev1 : 0x%x \n", MmioReadBe32((UINTN)&Fpm->IpRev1));
  DPAA1_DEBUG_MSG("IpRev2 : 0x%x \n", MmioReadBe32((UINTN)&Fpm->IpRev2));
  DPAA1_DEBUG_MSG("Rstc   : 0x%x \n", MmioReadBe32((UINTN)&Fpm->Rstc  ));
  DPAA1_DEBUG_MSG("Cldc   : 0x%x \n", MmioReadBe32((UINTN)&Fpm->Cldc  ));
  DPAA1_DEBUG_MSG("Npi    : 0x%x \n", MmioReadBe32((UINTN)&Fpm->Npi   ));
  DPAA1_DEBUG_MSG("FpEe   : 0x%x \n", MmioReadBe32((UINTN)&Fpm->FpEe  ));

  for (I =0; I < 0x4; I++)
    DPAA1_DEBUG_MSG("Cev[%d]  : 0x%x \n", I, MmioReadBe32((UINTN)&Fpm->Cev[I]));

  for (I =0; I < 0x40; I++)
    DPAA1_DEBUG_MSG("Ps[%d]  : 0x%x \n",I, MmioReadBe32((UINTN)&Fpm->Ps[I]));

  for (I =0; I < 0x80; I++)
    DPAA1_DEBUG_MSG("Ts[%d]  : 0x%x \n",I, MmioReadBe32((UINTN)&Fpm->Ts[I]));
}

VOID
DumpFmanQmi (
  IN VOID
  )
{
  FMAN_CCSR *FmanRegs = (FMAN_CCSR *)PcdGet64(PcdDpaa1FmanAddr);
  FMAN_QMI_COMMON *Qmi = &FmanRegs->QmiCommon;

  DPAA1_DEBUG_MSG(" ------------- DUMP QMI COMMON 0x%x -------------\n", Qmi);
  DPAA1_DEBUG_MSG(" Gc    : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Gc   ));
  DPAA1_DEBUG_MSG(" Eie   : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Eie  ));
  DPAA1_DEBUG_MSG(" Eien  : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Eien ));
  DPAA1_DEBUG_MSG(" Eif   : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Eif  ));
  DPAA1_DEBUG_MSG(" Ie    : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Ie   ));
  DPAA1_DEBUG_MSG(" Ien   : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Ien  ));
  DPAA1_DEBUG_MSG(" If    : 0x%x \n", MmioReadBe32((UINTN)&Qmi->If   ));
  DPAA1_DEBUG_MSG(" Gs    : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Gs   ));
  DPAA1_DEBUG_MSG(" Ts    : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Ts   ));
  DPAA1_DEBUG_MSG(" Etfc  : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Etfc ));
  DPAA1_DEBUG_MSG(" Dtfc  : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Dtfc ));
  DPAA1_DEBUG_MSG(" Dc0   : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Dc0  ));
  DPAA1_DEBUG_MSG(" Dc1   : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Dc1  ));
  DPAA1_DEBUG_MSG(" Dc2   : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Dc2  ));
  DPAA1_DEBUG_MSG(" Dc3   : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Dc3  ));
  DPAA1_DEBUG_MSG(" Dfnoc : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Dfnoc));
  DPAA1_DEBUG_MSG(" Dfcc  : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Dfcc ));
  DPAA1_DEBUG_MSG(" Dffc  : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Dffc ));
  DPAA1_DEBUG_MSG(" Dcc   : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Dcc  ));
  DPAA1_DEBUG_MSG(" Dtrc  : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Dtrc ));
  DPAA1_DEBUG_MSG(" Efddd : 0x%x \n", MmioReadBe32((UINTN)&Qmi->Efddd));
}

VOID
DumpFmanBmi (
  IN VOID
  )
{
  FMAN_CCSR *FmanRegs = (FMAN_CCSR *)PcdGet64(PcdDpaa1FmanAddr);
  FMAN_BMI_COMMON *Bmi = &FmanRegs->BmiCommon;
  UINT32 I = 0;

  DPAA1_DEBUG_MSG(" ------------- DUMP BMI COMMON 0x%x -------------\n", Bmi);
  DPAA1_DEBUG_MSG(" Init;           : 0x%x \n", MmioReadBe32((UINTN)&Bmi->Init));
  DPAA1_DEBUG_MSG(" Cfg1;           : 0x%x \n", MmioReadBe32((UINTN)&Bmi->Cfg1));
  DPAA1_DEBUG_MSG(" Cfg2;           : 0x%x \n", MmioReadBe32((UINTN)&Bmi->Cfg2));
  DPAA1_DEBUG_MSG(" Ievr;           : 0x%x \n", MmioReadBe32((UINTN)&Bmi->Ievr));
  DPAA1_DEBUG_MSG(" Ier;            : 0x%x \n", MmioReadBe32((UINTN)&Bmi->Ier ));
  DPAA1_DEBUG_MSG(" Ifr;            : 0x%x \n", MmioReadBe32((UINTN)&Bmi->Ifr ));
  DPAA1_DEBUG_MSG(" Gde;            : 0x%x \n", MmioReadBe32((UINTN)&Bmi->Gde ));
  for (I =0; I < 0x3f; I++)
    DPAA1_DEBUG_MSG("Pp[%x] : 0x%x \n", I, MmioReadBe32((UINTN)&Bmi->Pp[I]));
  
  for (I =0; I < 0x3f; I++)
    DPAA1_DEBUG_MSG("Pfs[%x] : 0x%x \n", I, MmioReadBe32((UINTN)&Bmi->Pfs[I]));

  for (I =0; I < 0x3f; I++)
    DPAA1_DEBUG_MSG("Ppid[%x] : 0x%x \n", I, MmioReadBe32((UINTN)&Bmi->Ppid[I]));
}

VOID
DumpFmanDma (
  IN VOID
  )
{
  FMAN_CCSR *FmanRegs = (FMAN_CCSR *)PcdGet64(PcdDpaa1FmanAddr);
  FMAN_DMA *Dma = &FmanRegs->FmanDma;
  UINT32 I = 0;

  DPAA1_INFO_MSG(" ------------- DUMP FMAN DMA 0x%x -------------\n", Dma);
  DPAA1_INFO_MSG(" Sr   : 0x%x \n", MmioReadBe32((UINTN)&Dma->Sr  ));
  DPAA1_INFO_MSG(" Mr   : 0x%x \n", MmioReadBe32((UINTN)&Dma->Mr  ));
  DPAA1_INFO_MSG(" Tr   : 0x%x \n", MmioReadBe32((UINTN)&Dma->Tr  ));
  DPAA1_INFO_MSG(" Hy   : 0x%x \n", MmioReadBe32((UINTN)&Dma->Hy  ));
  DPAA1_INFO_MSG(" Setr : 0x%x \n", MmioReadBe32((UINTN)&Dma->Setr));
  DPAA1_INFO_MSG(" Tah  : 0x%x \n", MmioReadBe32((UINTN)&Dma->Tah ));
  DPAA1_INFO_MSG(" Tal  : 0x%x \n", MmioReadBe32((UINTN)&Dma->Tal ));
  DPAA1_INFO_MSG(" Tcid : 0x%x \n", MmioReadBe32((UINTN)&Dma->Tcid));
  DPAA1_INFO_MSG(" Ra   : 0x%x \n", MmioReadBe32((UINTN)&Dma->Ra  ));
  DPAA1_INFO_MSG(" Rd   : 0x%x \n", MmioReadBe32((UINTN)&Dma->Rd  ));
  DPAA1_INFO_MSG(" Dcr  : 0x%x \n", MmioReadBe32((UINTN)&Dma->Dcr ));
  DPAA1_DEBUG_MSG(" Emsr : 0x%x \n", MmioReadBe32((UINTN)&Dma->Emsr));
  for (I =0; I < 32; I++)
    DPAA1_INFO_MSG("Plr[%d] : 0x%x \n", I, MmioReadBe32((UINTN)&Dma->Plr[I]));
}

VOID
DumpEthDev (
  IN  ETH_DEVICE *FmanEthDevice
  )
{
  DPAA1_DEBUG_MSG(" ------------- DUMPING BMI PORTS -------------\n");
  DPAA1_DEBUG_MSG("  FmIndex 	: 	0x%x \n", FmanEthDevice->FmIndex);
  DPAA1_DEBUG_MSG("  Num    	: 	0x%x \n", FmanEthDevice->Num    );

  DPAA1_DEBUG_MSG("  TxPort	: 	0x%x -------- \n", FmanEthDevice->TxPort);
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTcfg;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTcfg));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTst;   	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTst ));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTda;   	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTda ));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTfp;   	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTfp ));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTfed;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTfed));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTicp;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTicp));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTfne;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTfne));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTfca;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTfca));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTcfqid;	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTcfqid));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTfeqid;	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTfeqid));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTfene; 	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTfene));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTrlmts;	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTrlmts));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTrlmt; 	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTrlmt));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTstc;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTstc ));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTfrc;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTfrc ));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTfdc;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTfdc ));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTfledc;	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTfledc));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTfufdc;	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTfufdc));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTbdc;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTbdc ));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTpc;   	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTpc  ));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTpcp;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTpcp ));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTccn;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTccn ));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTtuc;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTtuc ));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTtcquc;	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTtcquc));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTduc;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTduc ));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTfuc;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTfuc ));
  DPAA1_DEBUG_MSG("  TxPort->FmanBmTdcfg;  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTdcfg));

  DPAA1_DEBUG_MSG("  RxPort	: 	0x%x ---------\n", FmanEthDevice->RxPort);
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRcfg      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRcfg));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRst 	 	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRst));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRda       	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRda));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRfp       	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRfp));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRfed      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRfed));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRicp      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRicp));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRim 	 	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRim ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRebm      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRebm));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRfne      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRfne));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRfca      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRfca));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRfpne     	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRfpne));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRpso      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRpso ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRpp       	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRpp  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRccb      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRccb ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRprai[0x8]	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRprai[0x8] ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRfqid     	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRfqid ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRefqid    	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRefqid));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRfsdm     	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRfsdm ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRfsem     	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRfsem ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRfene     	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRfene ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmEbmpi[0x8]	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmEbmpi[0x8] ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmAcnt[0x8] 	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmAcnt[0x8]  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmCgm[0x8]  	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmCgm[0x8]   ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmMpd       	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmMpd   ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRstc      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRstc  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRfrc      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRfrc  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRfbc      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRfbc  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRlfc      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRlfc  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRffc      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRffc  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRfdc      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRfdc  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRfldec    	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRfldec));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRodc      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRodc  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRbdc      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRbdc  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRpc       	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRpc   ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRpcp      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRpcp  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRccn      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRccn  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRtuc      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRtuc  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRrquc     	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRrquc ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRduc      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRduc  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRfuc      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRfuc  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRpac      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRpac  ));
  DPAA1_DEBUG_MSG("  RxPort->FmanBmRdbg      	: 	0x%x \n", MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRdbg  ));
                                                                                
  DPAA1_DEBUG_MSG("  Type       	: 	0x%x \n", FmanEthDevice->Type);            
  DPAA1_DEBUG_MSG("  Mac        	: 	0x%x \n", FmanEthDevice->Mac);             
  DPAA1_DEBUG_MSG("  RxPram    	: 	0x%x \n", FmanEthDevice->RxPram);         
  DPAA1_DEBUG_MSG("  TxPram    	: 	0x%x \n", FmanEthDevice->TxPram);         
  DPAA1_DEBUG_MSG("  RxBdRing 	: 	0x%x \n", FmanEthDevice->RxBdRing);      
  DPAA1_DEBUG_MSG("  CurRxbd   	: 	0x%x \n", FmanEthDevice->CurRxbd);        
  DPAA1_DEBUG_MSG("  RxBuf     	: 	0x%x \n", FmanEthDevice->RxBuf);          
  DPAA1_DEBUG_MSG("  TxBdRing 	: 	0x%x \n", FmanEthDevice->TxBdRing);      
  DPAA1_DEBUG_MSG("  CurPendingTxbdId 	: 	%d \n", FmanEthDevice->CurPendingTxbdId);
  DPAA1_DEBUG_MSG("  CurUsedTxbdId      : 	%d \n", FmanEthDevice->CurUsedTxbdId);
  DPAA1_DEBUG_MSG("  TotalPendingTxbd   : 	%u \n", FmanEthDevice->TotalPendingTxbd);
}                                                                               
                                                                                
VOID                                                                            
DumpBD (                                                                        
  IN  BD *Bd                                                     
  )                                                                             
{                                                                               
   UINT32 I =0;
   DPAA1_DEBUG_MSG(" ------------- DUMPING BD -------------\n");
   for (I = 0; I< 8; I++) {
   DPAA1_DEBUG_MSG("  Status	: 	0x%x \n", MemReadMasked(&Bd->Status)); 
   DPAA1_DEBUG_MSG("  Len          : 	0x%x \n", MemReadMasked(&Bd->Len)); 
   DPAA1_DEBUG_MSG("  BufPtrHi   : 	0x%x \n", MemReadMasked(&Bd->BufPtrHi));
   DPAA1_DEBUG_MSG("  BufPtrLo   : 	0x%x \n", MmioReadBe32((UINTN)&Bd->BufPtrLo));
   Bd++;
  }
}

VOID
DumpQD (
  IN  FMAN_QUEUE_DESC *Qd
  )
{
   DPAA1_DEBUG_MSG(" ------------- DUMPING QD -------------\n");
   DPAA1_DEBUG_MSG("  Gen               : 0x%x \n", MemReadMasked(&Qd->Gen)); 
   DPAA1_DEBUG_MSG("  BdRingBaseHi   : 	0x%x \n", MemReadMasked(&Qd->BdRingBaseHi)); 
   DPAA1_DEBUG_MSG("  BdRingBaseLo   : 	0x%x \n", MmioReadBe32((UINTN)&Qd->BdRingBaseLo));
   DPAA1_DEBUG_MSG("  BdRingSize      : 	0x%x \n", MemReadMasked(&Qd->BdRingSize));
   DPAA1_DEBUG_MSG("  OffsetIn         : 	0x%x \n", MemReadMasked(&Qd->OffsetIn));
   DPAA1_DEBUG_MSG("  OffsetOut        : 	0x%x \n", MemReadMasked(&Qd->OffsetOut));
}

STATIC VOID *
FmanMemAlloc(
  INT32 FmanIndex,
  UINT32 Size,
  UINTN Alignment
  )
{
  UINT32 UnalignedRemainder;
  VOID  *MemAddr;
  UINTN AlignMask = Alignment - 1;
  VOID* OldAllocCursor = gFmanMem[FmanIndex].AllocCursor;

  /* check if first free address of MURAM meet the asked alignment */
  UnalignedRemainder = (UINTN)OldAllocCursor & AlignMask;
  ASSERT(Alignment > UnalignedRemainder);
  if (UnalignedRemainder != 0) {
    gFmanMem[FmanIndex].AllocCursor += (Alignment - UnalignedRemainder);
  }

  UnalignedRemainder = Size & AlignMask;
  if (UnalignedRemainder != 0) {
    Size += (Alignment - UnalignedRemainder);
  }

  if (gFmanMem[FmanIndex].AllocCursor + Size >= gFmanMem[FmanIndex].Top) {
    gFmanMem[FmanIndex].AllocCursor = OldAllocCursor;
    DPAA1_ERROR_MSG("FMAN%d: WARNING: Out of memory for requested alignment\n",
           FmanIndex + 1);
    return NULL;
  }

  MemAddr = gFmanMem[FmanIndex].AllocCursor;
  gFmanMem[FmanIndex].AllocCursor += Size;

  InternalMemZeroMem(MemAddr, Size);

  DPAA1_DEBUG_MSG("FMAN alloc: 0x%x , size 0x%x \n",MemAddr, Size);

  return MemAddr;
}

VOID PopulateEthDev (
  IN  ETH_DEVICE *FmanEthDevice,
  IN  FMAN_MEMAC_ID Id
  )
{
  FMAN_CCSR *FmanRegs = (FMAN_CCSR *)PcdGet64(PcdDpaa1FmanAddr);
  UINT32 RxId = 0, TxId = 0;

  FmanEthDevice->TxSyncLock = TxLock;

  FmanEthDevice->RxSyncLock = RxLock;

  if (Id == FM1_DTSEC_9 || Id == FM1_DTSEC_10) {
    FmanEthDevice->Type = FM_ETH_10G;
    RxId = FMAN_RX_10G_PORT_ID_BASE + Id - FM1_DTSEC_9;
    TxId = FMAN_TX_10G_PORT_ID_BASE + Id - FM1_DTSEC_9;
  } else {
    FmanEthDevice->Type = FM_ETH_1G;
    RxId = FMAN_RX_1G_PORT_ID_BASE + Id - 1;
    TxId = FMAN_TX_1G_PORT_ID_BASE + Id - 1;
  }

  FmanEthDevice->Num = Id-1;

  FmanEthDevice->RxPort = (VOID *)&FmanRegs->port[RxId -1].FmanBmi;
  FmanEthDevice->TxPort = (VOID *)&FmanRegs->port[TxId -1].FmanBmi;
  DPAA1_DEBUG_MSG("Type %d, RxId = 0x%x TxId 0x%x , "
	"FmanEthDevice->RxPort 0x%x, FmanEthDevice->TxPort 0x%x \n",
	FmanEthDevice->Type, RxId-1, TxId-1, FmanEthDevice->RxPort,
	FmanEthDevice->TxPort);
}

EFI_STATUS EthDevInitMac (
  IN  ETH_DEVICE *FmEth
  )
{
  ENET_MAC *Mac;
  INT32 Num;
  UINTN Base, PhyRegs;
  FMAN_CCSR *Reg = (FMAN_CCSR *)PcdGet64(PcdDpaa1FmanAddr);

  Num = FmEth->Num;

  Base = (UINTN)&Reg->memac[Num].FmanMemac;
  PhyRegs = (UINTN)&Reg->memac[Num].FmanMemacMdio;

  Mac = AllocateZeroPool(sizeof(ENET_MAC));
  if (!Mac)
     return EFI_OUT_OF_RESOURCES;

  Mac->Base = (VOID *)Base;
  Mac->PhyRegs = (VOID *)PhyRegs;
  FmEth->Mac = Mac;

  return EFI_SUCCESS;
}

EFI_STATUS FmRxPortParamInit (
  IN  ETH_DEVICE *FmanEthDevice
  )
{
  FMAN_GLOBAL_PARAM *Pram;
  UINT32 PramPageOffset;
  VOID *RxRingBase;
  VOID *RxBufPool;
  BD *RxBd;
  FMAN_QUEUE_DESC *RxQd;
  BMI_RX_PORT *BmiRxPort = FmanEthDevice->RxPort;
  UINT32 BdRingBaseLo, BdRingBaseHi;
  UINT32 BufLo, BufHi;
  INT32 I;

  /* alloc global parameter from MURAM */
  Pram = (FMAN_GLOBAL_PARAM *)FmanMemAlloc(
                0, FMAN_PRAM_SIZE, FMAN_PRAM_ALIGN);

  if (!Pram) {
    DPAA1_ERROR_MSG("No muram for Rx global parameter\n");
    return EFI_OUT_OF_RESOURCES;
  }

  FmanEthDevice->RxPram = Pram;

  /* parameter page offset to MURAM */
  PramPageOffset = (VOID *)Pram - gFmanMem[0].Base;
  DPAA1_DEBUG_MSG("pram 0x%x base 0x%x PramPageOffset 0x%x \n",
	Pram, gFmanMem[0].Base, PramPageOffset);

  /* enable global mode- snooping data buffers and BDs */
  MmioWriteBe32((UINTN)&Pram->Mode, PARAM_MODE_GLOBAL);

  /* init Rx queue descriptor pointer */
  MmioWriteBe32((UINTN)&Pram->RxqdPtr, PramPageOffset + 0x20);

  /* set max receive buffer length, power of 2 (4096 - 2^12)*/
  MemWriteMasked(&Pram->Mrblr, MAX_RXBUF_LOG2);

  /* alloc Rx buffer descriptors from main memory */
  RxRingBase = AllocateZeroPool(sizeof(BD)
                * RX_RING_SIZE);
  if (!RxRingBase) {
    DPAA1_ERROR_MSG("Alloc memory failed for RX ring\n");
    return EFI_OUT_OF_RESOURCES;
  }

  /* Alloc Rx buffer from main memory */
  RxBufPool = AllocateZeroPool(MAX_RXBUF_LEN * RX_RING_SIZE);
  if (!RxBufPool) {
    DPAA1_ERROR_MSG("Alloc memory failed for RxBufPool\n");
    return EFI_OUT_OF_RESOURCES;
  }

  FmanEthDevice->RxBdRing = RxRingBase;
  FmanEthDevice->CurRxbd = RxRingBase;
  FmanEthDevice->RxBuf = RxBufPool;

  /* init Rx BD ring */
  RxBd = (BD *)RxRingBase;
  for (I = 0; I < RX_RING_SIZE; I++) {
    MemWriteMasked(&RxBd->Status, Rx_EMPTY);
    MemWriteMasked(&RxBd->Len, 0);
    BufHi = Upper32Bits((UINTN)(RxBufPool +
			I * MAX_RXBUF_LEN));
    BufLo = Lower32Bits((UINTN)(RxBufPool +
			I * MAX_RXBUF_LEN));
    MemWriteMasked(&RxBd->BufPtrHi, (UINT16)BufHi);
    MmioWriteBe32((UINTN)&RxBd->BufPtrLo, BufLo);
    RxBd++;
  }

  /* set Rx queue descriptor */
  RxQd = &Pram->Rxqd;
  MemWriteMasked(&RxQd->Gen, 0);
  BdRingBaseHi = Upper32Bits((UINTN)RxRingBase);
  BdRingBaseLo = Lower32Bits((UINTN)RxRingBase);
  MemWriteMasked(&RxQd->BdRingBaseHi, (UINT16)BdRingBaseHi);
  MmioWriteBe32((UINTN)&RxQd->BdRingBaseLo, BdRingBaseLo);
  MemWriteMasked(&RxQd->BdRingSize, sizeof(BD)
                * RX_RING_SIZE);
  MemWriteMasked(&RxQd->OffsetIn, 0);
  MemWriteMasked(&RxQd->OffsetOut, 0);

  /* set IM parameter ram pointer to Rx Frame Queue ID */
  MmioWriteBe32((UINTN)&BmiRxPort->FmanBmRfqid, PramPageOffset);

  return EFI_SUCCESS;
}

EFI_STATUS FmTxPortParamInit (
  IN  ETH_DEVICE *FmanEthDevice
  )
{
  FMAN_GLOBAL_PARAM *Pram;
  UINT32 PramPageOffset;
  VOID *TxRingBase;
  BD *TxBd;
  FMAN_QUEUE_DESC *TxQd;
  UINT32 BdRingBaseLo, BdRingBaseHi;
  BMI_TX_PORT *BmiTxPort = FmanEthDevice->TxPort;
  INT32 I;

  /* alloc global parameter ram from MURAM */
  Pram = (FMAN_GLOBAL_PARAM *)FmanMemAlloc(
                0, FMAN_PRAM_SIZE, FMAN_PRAM_ALIGN);

  if (!Pram) {
    DPAA1_ERROR_MSG("No muram for Tx global parameter\n");
    return EFI_OUT_OF_RESOURCES;
  }

  FmanEthDevice->TxPram = Pram;

  PramPageOffset = (VOID *)Pram - gFmanMem[0].Base;

  /* enable global mode- snooping data buffers and BDs */
  MmioWriteBe32((UINTN)&Pram->Mode, PARAM_MODE_GLOBAL);

  /* init Tx queue descriptor pointer */
  MmioWriteBe32((UINTN)&Pram->TxqdPtr, PramPageOffset + 0x40);

  TxRingBase = AllocateZeroPool(sizeof(BD)
                * TX_RING_SIZE);
  if (!TxRingBase) {
    DPAA1_ERROR_MSG("Alloc memory failed for TX ring\n");
    return EFI_OUT_OF_RESOURCES;
  }

  FmanEthDevice->TxBdRing = TxRingBase;
  FmanEthDevice->CurPendingTxbdId = -1;
  FmanEthDevice->CurUsedTxbdId = 0;
  FmanEthDevice->TotalPendingTxbd = 0;

  /* init Tx ring */
  TxBd = (BD *)TxRingBase;
  for (I = 0; I < TX_RING_SIZE; I++) {
    MemWriteMasked(&TxBd->Status, Tx_LAST);
    MemWriteMasked(&TxBd->Len, 0);
    MemWriteMasked(&TxBd->BufPtrHi, 0);
    MmioWriteBe32((UINTN)&TxBd->BufPtrLo, 0);
    TxBd++;
  }

  /* set Tx queue decriptor */
  TxQd = &Pram->Txqd;
  BdRingBaseHi = Upper32Bits((UINTN)TxRingBase);
  BdRingBaseLo = Lower32Bits((UINTN)TxRingBase);
  MemWriteMasked(&TxQd->BdRingBaseHi, (UINT16)BdRingBaseHi);
  MmioWriteBe32((UINTN)&TxQd->BdRingBaseLo, BdRingBaseLo);
  MemWriteMasked(&TxQd->BdRingSize, sizeof(BD)
                * TX_RING_SIZE);
  MemWriteMasked(&TxQd->OffsetIn, 0);
  MemWriteMasked(&TxQd->OffsetOut, 0);

  /* set IM parameter ram pointer to Tx Confirmation Frame Queue ID */
  MmioWriteBe32((UINTN)&BmiTxPort->FmanBmTcfqid, PramPageOffset);

  return EFI_SUCCESS;
}

/**
   Load one FMan firmware microcode image from NOR flash to DRAM, for a given FMan

   @param[in]     FmanIndex FMan instance index
   @param[in]     FManControllerInstrMem Pointer to FMan controller instruction
                  memory
   @param[in]     CodeStart Pointer to the beginning of the Microcode image in NOR flash
   @param[in]     CodeSize Size in bytes of the Microcode image in NOR flash

   @retval EFI_SUCCESS   on success
   @retval !EFI_SUCCESS  on failure.

 **/
STATIC
EFI_STATUS
FmanLoadMicrocode (
  IN INTN           FmanIndex,
  IN CONST FMAN_CONTROLLER_CONFIG *FmanControllerConfig,
  IN CONST UINT32   *CodeStart,
  IN UINTN          CodeSize
  )
{
  CONST UINT32 *CodeWord;
  CONST UINT32 *CodeEnd = &CodeStart[CodeSize / sizeof(UINT32)];
  UINTN Timeout = FMAN_CONTROLLER_WRITE_TIMEOUT;

  ASSERT(CodeSize % sizeof(UINT32) == 0);

  DPAA1_DEBUG_MSG("FmanLoadMicrocode:Start 0x%x, size 0x%x \n",
		CodeStart, CodeSize);

  //
  // Enable FMan controller configuration address auto increment
  //
  MmioWriteBe32((UINTN)&FmanControllerConfig->Address,
		FMAN_CONTROLLER_ADDR_AIE_MASK);

  //
  // Write microcode to FMan controller memory
  //
  for (CodeWord = CodeStart; CodeWord < CodeEnd; CodeWord++)
    MmioWrite32((UINTN)&FmanControllerConfig->Data, *CodeWord);

  //
  // Wait for the writes to complete:
  //
  MmioWriteBe32((UINTN)&FmanControllerConfig->Address, 0);
  do {
    MicroSecondDelay(1);
    if (MmioRead32((UINTN)&FmanControllerConfig->Data) ==
        *CodeStart) {
      break;
    }

    --Timeout;
  } while (Timeout != 0);

  if (Timeout == 0) {
    DPAA1_ERROR_MSG("FMAN[%d] Microcode loading timed out\n", FmanIndex);
    return EFI_DEVICE_ERROR;
  }

  /*
   * Tell FMan that it can access the loaded microcode
   */
  MmioWriteBe32((UINTN)&FmanControllerConfig->DataReady,
                FMAN_CONTROLLER_DATA_READY_MASK);

  return EFI_SUCCESS;
}


/**
   Load an FMan firmware image from NOR flash to DRAM, for a given Fman

   @param[in]     FmanIndex FMan instance index
   @param[in]     FManControllerInstrMem Pointer to FMan controller instruction
                  memory
   @param[in]     Firmware Pointer to FMan firmware image in NOR flash

   @retval EFI_SUCCESS   on success
   @retval !EFI_SUCCESS  on failure.

 **/
STATIC
EFI_STATUS
EFIAPI
FmanLoadFirmware (
  IN INTN FmanIndex,
  IN CONST FMAN_CONTROLLER_CONFIG *FmanControllerConfig,
  IN CONST FMAN_FIRMWARE *Firmware
  )
{
  EFI_STATUS        Status;
  UINTN             Length;
  UINTN             ActualLength;
  UINT32            Crc;
  UINT32            ComputedCrc;
  UINTN             I;
  CONST UINT32      *CodeStart;
  CONST FMAN_FW_HEADER *FwHdr = &Firmware->Header;

  if (!Firmware) {
    DPAA1_ERROR_MSG("FMan%d:Invalid firmware address\n", FmanIndex + 1);
    return EFI_INVALID_PARAMETER;
  }

  if (FwHdr->Magic[0] != 'Q' || FwHdr->Magic[1] != 'E' ||
      FwHdr->Magic[2] != 'F') {
    DPAA1_ERROR_MSG("FMan%d:Invalid firmware header\n", FmanIndex + 1);
    return EFI_UNSUPPORTED;
  }

  if (FwHdr->Version != FMAN_FW_VERSION) {
    DPAA1_ERROR_MSG("FMan%d:Firmware version not supported: %u\n",
           FmanIndex + 1, FwHdr->Version);
    return EFI_UNSUPPORTED;
  }

  Length = BE32_TO_CPU(FwHdr->Length);

  //
  // Compute actual length of the firmware image:
  //
  ActualLength = sizeof(*Firmware);
  for (I = 0; I < Firmware->Count; I++) {
    ActualLength += sizeof(FMAN_FW_MICROCODE_HEADER) +
                    BE32_TO_CPU(Firmware->MicrocodeHeaders[I].Count) * sizeof(UINT32);
  }

  if (Length != ActualLength + sizeof(Crc)) {
      DPAA1_ERROR_MSG(
             "FMan%d:Invalid length in firmware header: %u "
             "(actual length: %u)\n", FmanIndex + 1, Length,
             ActualLength + sizeof(Crc));
      return EFI_UNSUPPORTED;
  }

  //
  // Validate CRC:
  //
  ComputedCrc = ComputeCrc32((UINT8 *)Firmware, ActualLength);
  Crc = BE32_TO_CPU(*(UINT32 *)((UINT8 *)Firmware + ActualLength));
  if (Crc != ComputedCrc) {
    DPAA1_ERROR_MSG("FMan%d:Firmware CRC does not match "
           "(expected: %x, actual: %x)\n",
           FmanIndex, Crc, ComputedCrc);
    return EFI_CRC_ERROR;
  }

  //
  // Load each microcode image present in the firmware image:
  //
  for (I = 0; I < Firmware->Count; I++) {
    CONST FMAN_FW_MICROCODE_HEADER *MicrocodeHdr = &Firmware->MicrocodeHeaders[I];
    UINT32 CodeOffset = BE32_TO_CPU(MicrocodeHdr->CodeOffset);
    UINTN CodeSize = BE32_TO_CPU(MicrocodeHdr->Count) * sizeof(UINT32);

    if (BE32_TO_CPU(MicrocodeHdr->CodeOffset) == 0) {
      //
      // Skip empty microcode header
      //
      continue;
    }

    DPAA1_INFO_MSG(
           "FMan%d:Loading firmware microcode image version %u.%u.%u\n",
           FmanIndex + 1, MicrocodeHdr->Major, MicrocodeHdr->Minor,
           MicrocodeHdr->Revision);

    CodeStart = (UINT32 *)((UINT8 *)Firmware + CodeOffset);
    DPAA1_DEBUG_MSG("Fman: Uploading microcode from 0x%x\n",CodeStart);

    Status = FmanLoadMicrocode(FmanIndex, FmanControllerConfig, CodeStart, CodeSize);
    if (EFI_ERROR(Status)) {
      return Status;
    }
  }

  return EFI_SUCCESS;
}

STATIC VOID
FmanMemInit(
  INT32  FmanIndex,
  VOID   *MemAddr
  )
{
  VOID *Base = MemAddr;

  ASSERT((UINTN)MemAddr <= 0xffffffffu);
  gFmanMem[FmanIndex].Base = Base;
  gFmanMem[FmanIndex].Size = FMAN_MEM_SIZE;
  gFmanMem[FmanIndex].AllocCursor = Base + FMAN_MEM_RES_SIZE;
  gFmanMem[FmanIndex].Top = Base + FMAN_MEM_SIZE;
}

STATIC
VOID
FmanInitQmanInterface(FMAN_QMI_COMMON *Qmi)
{
  // Disable enqueue and dequeue of Qman interface
  MmioClearBitsBe32((UINTN)&Qmi->Gc, QMI_GC_ENQ_EN | QMI_GC_DEQ_EN);

  // Disable all error interrupts
  MmioWriteBe32((UINTN)&Qmi->Eien, 0x0);

  // Clear all error events ("write 1 to clear" bits)
  MmioWriteBe32((UINTN)&Qmi->Eie, QMI_EIE_ALL);

  // Disable all interrupts */
  MmioWriteBe32((UINTN)&Qmi->Ien, 0x0);

  // Clear all interrupts ("write 1 to cleae" bits)
  MmioWriteBe32((UINTN)&Qmi->Ie, QMI_IE_ALL);
}

STATIC
UINT32
BuildFpmPcrRegValue(UINTN PortId)
{
  UINT32 Controller;
  UINT32 RegValue;

  //
  // For Independent Mode (IM), associate even port IDs to
  // FMan controller 1 and odd port IDs to FMan controller 2
  //
  if (PortId % 2 == 0) {
    Controller = FPM_PRC_FMAN_CONTROLLER1;
  } else {
    Controller = FPM_PRC_FMAN_CONTROLLER2;
  }

  RegValue = (PortId << FPM_PRC_PORTID_SHIFT) & FPM_PRC_PORTID_MASK;
  RegValue |= Controller << FPM_PRC_ORA_SHIFT;
  RegValue |= Controller;
  return RegValue;
}


STATIC
VOID
FmanInitFPM(FMAN_FPM *Fpm)
{
  UINTN Index;
  UINT32 Port;
  UINT32 RegValue;

  //
  // Initialize FPM event and enable register:
  //
  MmioSetBitsBe32((UINTN)&Fpm->FpEe,
     FPM_FPEE_EHM | FPM_FPEE_UEC | FPM_FPEE_CER | FPM_FPEE_DER);

  /* IM mode, each even port ID to RISC#1, each odd port ID to RISC#2 */
  //
  // Initialize Offline and Host (O/H) ports:
  //
  for(Index = 0; Index < FMAN_MAX_NUM_OFFLINE_PORTS; Index++) {
    Port = FMAN_OFFLINE_PORT_ID_BASE + Index;
    RegValue = BuildFpmPcrRegValue(Port);
    MmioWriteBe32((UINTN)&Fpm->Prc, RegValue);
  }

  //
  // Initialize Rx 1G ports:
  //
  for(Index = 0; Index < FMAN_MAX_NUM_RX_1G_PORTS; Index++) {
    Port = FMAN_RX_1G_PORT_ID_BASE + Index;
    RegValue = BuildFpmPcrRegValue(Port);
    MmioWriteBe32((UINTN)&Fpm->Prc, RegValue);
  }

  //
  // Initialize Tx 1G ports:
  //
  for(Index = 0; Index < FMAN_MAX_NUM_TX_1G_PORTS; Index++) {
    Port = FMAN_TX_1G_PORT_ID_BASE + Index;
    RegValue = BuildFpmPcrRegValue(Port);
    MmioWriteBe32((UINTN)&Fpm->Prc, RegValue);
  }

  //
  // Initialize Rx 10G port:
  //
  RegValue = BuildFpmPcrRegValue(FMAN_RX_10G_PORT_ID_BASE);
  MmioWriteBe32((UINTN)&Fpm->Prc, RegValue);

  //
  // Initialize Tx 10G port:
  //
  RegValue = BuildFpmPcrRegValue(FMAN_TX_10G_PORT_ID_BASE);
  MmioWriteBe32((UINTN)&Fpm->Prc, RegValue);

  //
  // For independent mode (IM), disable the dispatch limit:
  //
  MmioWriteBe32((UINTN)&Fpm->Flc, FPM_FLC_DISPATCH_LIMIT_NONE);

  //
  // Clear events (write 1 to clear):
  //
  MmioWriteBe32((UINTN)&Fpm->FpEe, FPM_FPEE_ALL_EVENTS);

  //
  // Clear CPU events (write 1 to clear):
  //
  for(Index = 0; Index < ARRAY_SIZE(Fpm->Cev); Index++) {
    MmioWriteBe32((UINTN)&Fpm->Cev[Index], 0xFFFFFFFF);
  }

  //
  // Clear ECC events (write 1 to clear) for Fman internal memory access:
  //
  MmioWriteBe32((UINTN)&Fpm->Rcr, FPM_RCR_MDEC | FPM_RCR_IDEC);
}


STATIC
EFI_STATUS
FmanInitBmanInterface(
  INTN              FmanIndex,
  FMAN_BMI_COMMON   *Bmi
  )
{
  VOID *Base;
  UINT32 FreeBufferPoolSizeInBlocks;
  UINT32 FreeBufferPoolOffsetInBlocks;
  UINT32 PortIndex;
  UINT32 Index;
  UINT32 RegValue;

  Base = FmanMemAlloc(FmanIndex, FMAN_FREE_POOL_SIZE,
                      FMAN_FREE_POOL_ALIGNMENT);
  if (Base == 0) {
      return EFI_OUT_OF_RESOURCES;
  }

  //
  // Set free buffer pool offset and size in BMI's register CFG1
  //
  FreeBufferPoolOffsetInBlocks = (Base - gFmanMem[FmanIndex].Base) /
                                 FMAN_FREE_POOL_ALIGNMENT;
  FreeBufferPoolSizeInBlocks = FMAN_FREE_POOL_SIZE / FMAN_FREE_POOL_ALIGNMENT;

  RegValue = 0;

  __SET_BIT_FIELD32(RegValue, BMI_CFG1_FBPO_MASK, BMI_CFG1_FBPO_SHIFT,
                FreeBufferPoolOffsetInBlocks);
  __SET_BIT_FIELD32(RegValue, BMI_CFG1_FBPS_MASK, BMI_CFG1_FBPS_SHIFT,
                FreeBufferPoolSizeInBlocks - 1);

  MmioWriteBe32((UINTN)&Bmi->Cfg1, RegValue);

  //
  // Disable and clear all interrupts:
  //
  MmioWriteBe32((UINTN)&Bmi->Ier, 0x0);
  MmioWriteBe32((UINTN)&Bmi->Ievr, BMI_IEVR_ALL);

  //
  // Configure offline ports:
  // - port parameters: max tasks=1, max dma=1, no extra
  // - port FIFO size: 256 bytes, no extra
  //
  for(Index = 0; Index < FMAN_MAX_NUM_OFFLINE_PORTS; Index++) {
    PortIndex = FMAN_OFFLINE_PORT_ID_BASE + Index - 1;
    MmioWriteBe32((UINTN)&Bmi->Pp[PortIndex], 0x0);
    MmioWriteBe32((UINTN)&Bmi->Pfs[PortIndex], 0x0);
  }

  //
  // Configure Rx 1G ports:
  // port params:  max tasks=4, max dma=1, no extra
  // - port FIFO size: 4KB, no extra
  //
  for(Index = 0; Index < FMAN_MAX_NUM_RX_1G_PORTS; Index++) {
    PortIndex = FMAN_RX_1G_PORT_ID_BASE + Index - 1;
    RegValue = 0;
    __SET_BIT_FIELD32(RegValue, BMI_PP_MXT_MASK, BMI_PP_MXT_SHIFT, 4 - 1);
    MmioWriteBe32((UINTN)&Bmi->Pp[PortIndex], RegValue);
    RegValue = 0;
    __SET_BIT_FIELD32(RegValue, BMI_PFS_IFSZ_MASK, BMI_PFS_IFSZ_SHIFT, 0xf);
    MmioWriteBe32((UINTN)&Bmi->Pfs[PortIndex], RegValue);
  }

  //
  // Configure Tx 1G ports:
  // port params:  max tasks=4, max dma=1, no extra
  // - port FIFO size: 4KB, no extra
  //
  for(Index = 0; Index < FMAN_MAX_NUM_TX_1G_PORTS; Index++) {
    PortIndex = FMAN_TX_1G_PORT_ID_BASE + Index - 1;
    RegValue = 0;
    __SET_BIT_FIELD32(RegValue, BMI_PP_MXT_MASK, BMI_PP_MXT_SHIFT, 4 - 1);
    MmioWriteBe32((UINTN)&Bmi->Pp[PortIndex], RegValue);
    RegValue = 0;
    __SET_BIT_FIELD32(RegValue, BMI_PFS_IFSZ_MASK, BMI_PFS_IFSZ_SHIFT, 0xf);
    MmioWriteBe32((UINTN)&Bmi->Pfs[PortIndex], RegValue);
  }

  //
  // Configure Rx 10G port:
  // port params:  max tasks=12, max dma=3, no extra
  // - port FIFO size: 4KB, no extra
  //
  PortIndex = FMAN_RX_10G_PORT_ID_BASE - 1;
  RegValue = 0;
  __SET_BIT_FIELD32(RegValue, BMI_PP_MXT_MASK, BMI_PP_MXT_SHIFT, 12 - 1);
  __SET_BIT_FIELD32(RegValue, BMI_PP_MXD_MASK, BMI_PP_MXD_SHIFT, 3 - 1);
  MmioWriteBe32((UINTN)&Bmi->Pp[PortIndex], RegValue);
  RegValue = 0;
  __SET_BIT_FIELD32(RegValue, BMI_PFS_IFSZ_MASK, BMI_PFS_IFSZ_SHIFT, 0xf);
  MmioWriteBe32((UINTN)&Bmi->Pfs[PortIndex], RegValue);

  //
  // Configure Tx 10G port:
  // port params:  max tasks=12, max dma=3, no extra
  // - port FIFO size: 4KB, no extra
  //
  PortIndex = FMAN_TX_10G_PORT_ID_BASE - 1;
  RegValue = 0;
  __SET_BIT_FIELD32(RegValue, BMI_PP_MXT_MASK, BMI_PP_MXT_SHIFT, 12 - 1);
  __SET_BIT_FIELD32(RegValue, BMI_PP_MXD_MASK, BMI_PP_MXD_SHIFT, 3 - 1);
  MmioWriteBe32((UINTN)&Bmi->Pp[PortIndex], RegValue);
  RegValue = 0;
  __SET_BIT_FIELD32(RegValue, BMI_PFS_IFSZ_MASK, BMI_PFS_IFSZ_SHIFT, 0xf);
  MmioWriteBe32((UINTN)&Bmi->Pfs[PortIndex], RegValue);

  //
  // Initialize internal buffers data base:
  //
  MmioWriteBe32((UINTN)&Bmi->Init, BMI_INIT_START);

  return EFI_SUCCESS;
}

EFI_STATUS
ReceiveFrame(
  ETH_DEVICE *FmanEthDevice,
  UINTN *BuffSize,
  VOID *Data
  )
{
  FMAN_GLOBAL_PARAM *Pram;
  BD *Rxbd, *RxbdBase;
  UINT16 Status;
  UINT32 Len;
  UINT16 OffsetOut;
  UINT8 *Buffer;
  UINT32 BufLo, BufHi;

  Pram = FmanEthDevice->RxPram;
  Rxbd = FmanEthDevice->CurRxbd;

  EfiAcquireLock(&FmanEthDevice->RxSyncLock);
  Status = MemReadMasked(&Rxbd->Status);

  if (!(Status & Rx_EMPTY)) {
    if (!(Status & Rx_ERROR)) {
      BufHi = MemReadMasked(&Rxbd->BufPtrHi);
      BufLo = MmioReadBe32((UINTN)&Rxbd->BufPtrLo);
      Buffer = (UINT8 *)((UINTN)(BufHi << 16) << 16 | BufLo);
      Len = MemReadMasked(&Rxbd->Len);
    } else {
      DPAA1_ERROR_MSG("Rx error\n");
      EfiReleaseLock(&FmanEthDevice->RxSyncLock);
      return EFI_DEVICE_ERROR;
    }
    /* Update Buffersize to actual value */
    if ( *BuffSize < Len ) {
      DPAA1_ERROR_MSG("RX Buffersize is too small %d,required to be %d\n",
                       *BuffSize, Len);
      *BuffSize = Len;
      return EFI_BUFFER_TOO_SMALL;
    } else
      *BuffSize = Len;

    InternalMemCopyMem(Data, (VOID *)Buffer, Len);

    /* clear the RxBDs */
    MemWriteMasked(&Rxbd->Status, Rx_EMPTY);
    MemWriteMasked(&Rxbd->Len, 0);
    MemoryFence();

    /* advance RxBD */
    Rxbd++;
    RxbdBase = (BD *)FmanEthDevice->RxBdRing;
    if (Rxbd >= (RxbdBase + RX_RING_SIZE))
      Rxbd = RxbdBase;

    /* update RxQD */
    OffsetOut = MemReadMasked(&Pram->Rxqd.OffsetOut);
    OffsetOut += sizeof(BD);
    if (OffsetOut >= MemReadMasked(&Pram->Rxqd.BdRingSize))
      OffsetOut = 0;

    MemWriteMasked(&Pram->Rxqd.OffsetOut, OffsetOut);
    MemoryFence();
  }

  FmanEthDevice->CurRxbd = (VOID *)Rxbd;
  EfiReleaseLock(&FmanEthDevice->RxSyncLock);

  return EFI_SUCCESS;
}

/*
* Transmit Frame : Enqueue a buffer to TxBD circular queue
* for transmitting
*/
EFI_STATUS
TransmitFrame (
  IN  ETH_DEVICE *FmanEthDevice,
  IN VOID * Data, 
  IN UINTN BuffSize
  )
{
  FMAN_GLOBAL_PARAM *Pram;
  UINT8 TxbdId;
  BD *Txbd;
  UINT16 Offset;

  Pram = FmanEthDevice->TxPram;

  if (FmanEthDevice->TotalPendingTxbd == TX_RING_SIZE)
    return EFI_NOT_READY;

  TxbdId = (FmanEthDevice->CurPendingTxbdId+1) % TX_RING_SIZE;

  Txbd = &(((BD *)FmanEthDevice->TxBdRing)[TxbdId]);

  EfiAcquireLock(&FmanEthDevice->TxSyncLock);

  if (MemReadMasked(&Txbd->Status) & Tx_READY) {
    EfiReleaseLock(&FmanEthDevice->TxSyncLock);
    return EFI_NOT_READY;
  }

  /* setup TxBD */
  MemWriteMasked(&Txbd->BufPtrHi, (UINT16)Upper32Bits((UINTN)Data));
  MmioWriteBe32((UINTN)&Txbd->BufPtrLo, Lower32Bits((UINTN)Data));
  MemWriteMasked(&Txbd->Len, BuffSize);

  MemoryFence();
  MemWriteMasked(&Txbd->Status, Tx_READY | Tx_LAST);
  MemoryFence();

  /* update TxQD, let RISC to send the packet */
  Offset = MemReadMasked(&Pram->Txqd.OffsetIn);
  Offset += sizeof(BD);
  if (Offset >= MemReadMasked(&Pram->Txqd.BdRingSize))
    Offset = 0;

  MemWriteMasked(&Pram->Txqd.OffsetIn, Offset);
  MemoryFence();

  /* update current txbd and peding TxBd count */
  FmanEthDevice->CurPendingTxbdId = TxbdId;
  FmanEthDevice->TotalPendingTxbd++;
  EfiReleaseLock(&FmanEthDevice->TxSyncLock);

  return EFI_SUCCESS;
}

/*
* GetTransmitStatus : Dequeue a buffer from TxBD circular queue
* that has been transmitted
*/
EFI_STATUS
GetTransmitStatus (
  IN  ETH_DEVICE *FmanEthDevice,
  IN VOID        **TxBuf
  )
{
  BD *Txbd;
  UINT32 BufLo, BufHi;

  Txbd = &(((BD *)FmanEthDevice->TxBdRing)[FmanEthDevice->CurUsedTxbdId]);

  EfiAcquireLock(&FmanEthDevice->TxSyncLock);

  if ((0 == FmanEthDevice->TotalPendingTxbd) ||
     (MemReadMasked(&Txbd->Status) & Tx_READY))
    *TxBuf = NULL;
  else {
    BufHi = MemReadMasked(&Txbd->BufPtrHi);
    BufLo = MmioReadBe32((UINTN)&Txbd->BufPtrLo);
    *TxBuf = (UINT8 *)((UINTN)(BufHi << 16) << 16 | BufLo);
    FmanEthDevice->CurUsedTxbdId = (FmanEthDevice->CurUsedTxbdId + 1) % TX_RING_SIZE;
    FmanEthDevice->TotalPendingTxbd--;
  }
  EfiReleaseLock(&FmanEthDevice->TxSyncLock);

  return EFI_SUCCESS;
}

/**
   Initialize a given frame manager (FMan) instance

   @param[in]     FmanIndex FMan instance index
   @param[in]     FManCcsrBase Base address of the FMan instance's CCSR registers
                  memory

   @retval EFI_SUCCESS   on success
   @retval !EFI_SUCCESS  on failure.

 **/
EFI_STATUS
FmanInit (
  INTN  FmanIndex,
  FMAN_CCSR *FmanRegs
  )
{
  EFI_STATUS Status;
  CONST FMAN_FIRMWARE *FwAddr;

  FwAddr = (VOID*)(UINTN)PcdGet32(PcdFManFwFlashAddr);

  Status = FmanLoadFirmware(FmanIndex, &FmanRegs->FmanControllerCfg, FwAddr);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  FmanMemInit(FmanIndex, &FmanRegs->FmanMemory);
  FmanInitQmanInterface(&FmanRegs->QmiCommon);
  FmanInitFPM(&FmanRegs->FmanFpm);

  Status = FmanInitBmanInterface(FmanIndex, &FmanRegs->BmiCommon);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  return EFI_SUCCESS;
}
