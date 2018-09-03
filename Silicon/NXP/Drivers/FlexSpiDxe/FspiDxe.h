/** @FspiDxe.h
  Header Defining The Fspi Flash Controller Constants (Base Addresses, Sizes,
  Flags), Function Prototype, Structures etc

  Copyright 2018 NXP

  This Program And The Accompanying Materials
  Are Licensed And Made Available Under The Terms And Conditions Of The BSD
  License Which Accompanies This Distribution. The Full Text Of The License
  May Be Found At
  http://Opensource.Org/Licenses/Bsd-License.Php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __FSPI_DXE_H__
#define __FSPI_DXE_H__

#include <Pi/PiSpi.h>
#include <Protocol/SpiHc.h>
#include <libfdt.h>

#define TX_IPBUF_SIZE        0x200 /* 64 * 64bits  */
#define RX_IPBUF_SIZE        SIZE_1KB /* 128 * 64bits */
#define RX_AHBBUF_SIZE       SIZE_2KB /* 256 * 64bits */
#define TX_AHBBUF_SIZE       0x40  /* 8 * 64bits   */

#define RX_WMRK              0 // Watermark level is (RXWMRK+1)*64 Bits.
#define TX_WMRK              0 // Watermark level is (TXWMRK+1)*64 Bits.

#define LUTCR_LOCK           (0x1)
#define LUTCR_UNLOCK         (0x2)
#define LUT_KEY              (0x5af05af0)

/*
 * The definition of the LUT register shows below:
 *
 *  ---------------------------------------------------
 *  | INSTR1 | PAD1 | OPRND1 | INSTR0 | PAD0 | OPRND0 |
 *  ---------------------------------------------------
 */
#define LUT_OPRND0_SHIFT      (0)
#define LUT_PAD0_SHIFT        (8)
#define LUT_INSTR0_SHIFT      (10)
#define LUT_OPRND1_SHIFT      (16)

#define LUT_STOP              0x00
#define LUT_CMD               0x01
#define LUT_ADDR              0x02
#define LUT_CADDR_SDR         0x03
#define LUT_MODE              0x04
#define LUT_MODE2             0x05
#define LUT_MODE4             0x06
#define LUT_MODE8             0x07
#define LUT_WRITE             0x08
#define LUT_READ              0x09
#define LUT_LEARN_SDR         0x0A
#define LUT_DATSZ_SDR         0x0B
#define LUT_DUMMY             0x0C
#define LUT_DUMMY_RWDS_SDR    0x0D
#define LUT_JMP_ON_CS         0x1F
#define LUT_CMD_DDR           0x21
#define LUT_ADDR_DDR          0x22
#define LUT_CADDR_DDR         0x23
#define LUT_MODE_DDR          0x24
#define LUT_MODE2_DDR         0x25
#define LUT_MODE4_DDR         0x26
#define LUT_MODE8_DDR         0x27
#define LUT_WRITE_DDR         0x28
#define LUT_READ_DDR          0x29
#define LUT_LEARN_DDR         0x2A
#define LUT_DATSZ_DDR         0x2B
#define LUT_DUMMY_DDR         0x2C
#define LUT_DUMMY_RWDS_DDR    0x2D

#define LUT_PAD_1             0
#define LUT_PAD_2             1
#define LUT_PAD_4             2
#define LUT_PAD_8             3

#define SEQ_ID_IP_MODE       (1) // LUT entry to use

/* Module Configuration */
#define MCR0_SWRESET           BIT0 // Software Reset
#define MCR0_MDIS              BIT1 // Module Disable
#define MCR0_END_CFG_MASK      (BIT2|BIT3)
#define MCR0_END_CFD_64BIT_LE  0
#define MCR0_END_CFD_32BIT_LE  BIT3
#define MCR0_RXCLKSRC_MASK     (BIT4|BIT5)
#define MCR0_RXCLKSRC_DEFAULT  0
#define MCR0_ARDFEN            BIT6 // Enable AHB bus Read Access to IP RX FIFO.
#define MCR0_ATDFEN            BIT7 // Enable AHB bus Write Access to IP TX FIFO.
#define MCR0_HSEN              BIT11 // Half Speed Serial Flash access Enable.
#define MCR0_DOZEEN            BIT12 // Doze mode enable bit.
#define MCR0_COMBINATIONEN     BIT13 // support Flash Octal mode access by combining Port A and B Data pins
#define MCR0_SCKFREERUNEN      BIT14 // force SCK output free-running.
#define MCR0_LEARNEN           BIT15 // This bit is used to enable/disable data learning feature.

#define MCR2_CLRAHBBUFOPT      BIT11 // This bit determines whether AHB RX Buffer and AHB TX Buffer will be
                                     // cleaned automaticaly when FlexSPI returns STOP mode ACK.
#define MCR2_SAMEDEVICEEN      BIT15 // All external devices are same devices (both in types and size) for A1/A2/B1/B2.

#define FLSHCR0_FLSHSZ_MASK    0x7FFFFF // Flash Size in KByte.
#define FLSHCR1_WA             BIT10 // Word Addressable.
#define FLSHCR1_CAS_MASK       (BIT11|BIT12|BIT13|BIT14) // Column Address Size.

#define AHBCR_APAREN           BIT0 // Parallel mode enabled for AHB triggered Command (both read and write).
#define AHBCR_PREFETCHEN       BIT5 // AHB Read Prefetch Enable.
#define AHBCR_READADDROPT      BIT6 // AHB Read Address option bit. This option bit is intend to remove
                                    // AHB burst start address alignment limitation.

#define AHBRXBUFxCR_PREFETCHEN BIT31 // AHB Read Prefetch Enable for current AHB RX Buffer corresponding Master.

#define IPCR1_IPAREN           BIT31 // Parallel mode Enabled for IP command.
#define IPCR1_ISEQID_SHIFT     16

#define IPTXFCR_CLRIPTXF       BIT0 // Clear all valid data entries in IP TX FIFO.
#define IPTXFCR_TXWMRK_MASK    0x7F // Bits 2-8
#define IPTXFCR_TXWMRK_SHIFT   2

#define IPRXFCR_CLRIPRXF       BIT0 // Clear all valid data entries in IP RX FIFO.
#define IPRXFCR_RXWMRK_MASK    0x7F // Bits 2-8
#define IPRXFCR_RXWMRK_SHIFT   2

#define STS0_ARBIDLE            BIT1

#define IPCMD_TRG              BIT0 // Setting this bit will trigger an IP Command.

#define INTR_SEQTIMEOUT        BIT11 // Sequence execution timeout
#define INTR_AHBBUSTIMEOUT     BIT10 // AHB Bus timeout
#define INTR_SCKSTOPBYWR       BIT9 // SCK is stopped during command sequence because Async TX FIFO empty
#define INTR_SCKSTOPBYRD       BIT8 // SCK is stopped during command sequence because Async RX FIFO full
#define INTR_DATALEARNFAIL     BIT7 // Data Learning failed
#define INTR_IPTXWE            BIT6 // IP TX FIFO has no less empty space than WaterMark level
#define INTR_IPRXWA            BIT5 // IP RX FIFO has no less valid data than WaterMark level
#define INTR_AHBCMDERR         BIT4 // AHB triggered Command Sequences Error Detected
#define INTR_IPCMDERR          BIT3 // IP triggered Command Sequences Error Detected
#define INTR_AHBCMDGE          BIT2 // AHB triggered Command Sequences Grant Timeout
#define INTR_IPCMDGE           BIT1 // IP triggered Command Sequences Grant Timeout
#define INTR_IPCMDDONE         BIT0 // IP triggered Command Sequences Execution finished

// Error Categories
#define CMD_GRANT_ERR_MASK     (INTR_AHBCMDGE | INTR_IPCMDGE) // Command grant error
#define CMD_CHECK_ERR_MASK     (INTR_AHBCMDERR | INTR_IPCMDERR) // Command check error
#define CMD_EXEC_ERR_MASK      (CMD_CHECK_ERR_MASK | INTR_SEQTIMEOUT) // Command execution error
#define AHB_TOUT_ERR_MASK      INTR_AHBBUSTIMEOUT // AHB Bus timeout
#define DLEARN_ERR_MASK        INTR_DATALEARNFAIL // Data Learning Failed

#define FLEXSPI_ERR_MASK       (CMD_GRANT_ERR_MASK | CMD_CHECK_ERR_MASK |\
                                CMD_EXEC_ERR_MASK | AHB_TOUT_ERR_MASK | DLEARN_ERR_MASK)

#define IPRXFSTS_FILL_MASK     0xFF // Fill level of IP RX FIFO
#define IPRXFSTS_FILL_SHIFT    0 // Valid Data entries in IP RX FIFO is: FILL * 64 Bits.

///
/// Fspi Controller supports write only and write then read type of requests.
/// if we encounter any other type of request, we return unsupported.
///

typedef enum {
  SPI_REQUEST_NONE = 0,

  ///
  /// Data flowing from the host to the SPI peripheral.
  ///
  SPI_REQUEST_WRITE_ONLY,
  ///
  /// Data first flowing from the host to the SPI peripheral and then data flows from the SPI peripheral to the host.
  /// These types of operations get used for SPI flash devices when control data (opcode, address) must be passed to
  /// the SPI peripheral to specify the data to be read.
  ///
  SPI_REQUEST_WRITE_THEN_READ,

  SPI_REQUEST_INVALID
} SPI_REQUEST_TYPE;

typedef struct {
  ///
  /// Type of SPI request
  ///
  SPI_REQUEST_TYPE   Type;
  ///
  /// Address to read/write from
  ///
  UINT32             Address;
  ///
  /// Buffer to/from which data is to be written/read
  ///
  VOID               *Buffer;
  ///
  /// Buffer size
  ///
  UINT32             Length;
  ///
  /// Lut Index (0..15) to use to complete this request
  ///
  UINT8              LutId;
} FSPI_REQUEST;

///
/// Conventions Used :
/// RW : Read Write, RO : Read Only, WO : Write Only, W1C : Write 1 to Clear
/// Fspi Controller Registers
/// ---------------------------------------------------------------------------------------------
/// Offset     | Register                                          | Width | Access | Reset Value
///            |                                                   |(Bits) |        |
/// ---------------------------------------------------------------------------------------------
/// 0h         | Module Control Register 0 (MCR0)                  |  32   |  RW    |  FFFF80C2h
/// 4h         | Module Control Register 1 (MCR1)                  |  32   |  RW    |  FFFFFFFFh
/// 8h         | Module Control Register 2 (MCR2)                  |  32   |  RW    |  200081F7h
/// Ch         | AHB Bus Control Register (AHBCR)                  |  32   |  RW    |  00000018h
/// 10h        | Interrupt Enable Register (INTEN)                 |  32   |  RW    |  00000000h
/// 14h        | Interrupt Register (INTR)                         |  32   |  W1C   |  00000040h
/// 18h        | LUT Key Register (LUTKEY)                         |  32   |  RW    |  5AF05AF0h
/// 1Ch        | LUT Control Register (LUTCR)                      |  32   |  RW    |  00000002h
/// 20h        | AHB RX Buffer 0 Control Register 0 (AHBRXBUF0CR0) |  32   |  RW    |  80000020h
/// 24h        | AHB RX Buffer 1 Control Register 0 (AHBRXBUF1CR0) |  32   |  RW    |  80010020h
/// 28h        | AHB RX Buffer 2 Control Register 0 (AHBRXBUF2CR0) |  32   |  RW    |  80020020h
/// 2Ch        | AHB RX Buffer 3 Control Register 0 (AHBRXBUF3CR0) |  32   |  RW    |  80030020h
/// 30h        | AHB RX Buffer 4 Control Register 0 (AHBRXBUF4CR0) |  32   |  RW    |  80040020h
/// 34h        | AHB RX Buffer 5 Control Register 0 (AHBRXBUF5CR0) |  32   |  RW    |  80050020h
/// 38h        | AHB RX Buffer 6 Control Register 0 (AHBRXBUF6CR0) |  32   |  RW    |  80060020h
/// 3Ch        | AHB RX Buffer 7 Control Register 0 (AHBRXBUF7CR0) |  32   |  RW    |  80070020h
/// 40h - 5Ch  | AHB RX Buffer a Control 1                         |  32   |  RW    |  00000000h
///            | (AHBRXBUF0CR1 - AHBRXBUF7CR1)                     |       |        |
/// 60h        | Flash A1 Control Register 0 (FLSHA1CR0)           |  32   |  RW    |  00010000h
/// 64h        | Flash A2 Control Register 0 (FLSHA2CR0)           |  32   |  RW    |  00010000h
/// 68h        | Flash B1 Control Register 0 (FLSHB1CR0)           |  32   |  RW    |  00010000h
/// 6Ch        | Flash B2 Control Register 0 (FLSHB2CR0)           |  32   |  RW    |  00010000h
/// 70h        | Flash A1 Control Register 1 (FLSHA1CR1)           |  32   |  RW    |  00000063h
/// 74h        | Flash A2 Control Register 1 (FLSHA2CR1)           |  32   |  RW    |  00000063h
/// 78h        | Flash B1 Control Register 1 (FLSHB1CR1)           |  32   |  RW    |  00000063h
/// 7Ch        | Flash B2 Control Register 1 (FLSHB2CR1)           |  32   |  RW    |  00000063h
/// 80h        | Flash A1 Control Register 2 (FLSHA1CR2)           |  32   |  RW    |  00000000h
/// 84h        | Flash A2 Control Register 2 (FLSHA2CR2)           |  32   |  RW    |  00000000h
/// 88h        | Flash B1 Control Register 2 (FLSHB1CR2)           |  32   |  RW    |  00000000h
/// 8Ch        | Flash B2 Control Register 2 (FLSHB2CR2)           |  32   |  RW    |  00000000h
/// 90h        | Flash Control Register 3 (FLSHCR3)                |  32   |  RW    |  0D000C0Ch
/// 94h        | Flash Control Register 4 (FLSHCR4)                |  32   |  RW    |  00000000h
/// 98h        | Flash Control Register 5 (FLSHCR5)                |  32   |  RW    |  00000000h
/// 9Ch        | Flash Control Register 6 (FLSHCR6)                |  32   |  RW    |  00000000h
/// A0h        | IP Control Register 0 (IPCR0)                     |  32   |  RW    |  00000000h
/// A4h        | IP Control Register 1 (IPCR1)                     |  32   |  RW    |  00000000h
/// A8h        | IP Control Register 2 (IPCR2)                     |  32   |  RW    |  00000000h
/// ACh        | IP Control Register 3 (IPCR3)                     |  32   |  RW    |  00000000h
/// B0h        | IP Command Register (IPCMD)                       |  32   |  RW    |  00000000h
/// B4h        | Data Learn Pattern Register (DLPR)                |  32   |  RW    |  00000000h
/// B8h        | IP RX FIFO Control Register (IPRXFCR)             |  32   |  RW    |  00000000h
/// BCh        | IP TX FIFO Control Register (IPTXFCR)             |  32   |  RW    |  00000000h
/// C0h        | DLL Control Register 0 (DLLACR)                   |  32   |  RW    |  00000100h
/// C4h        | DLL Control Register 0 (DLLBCR)                   |  32   |  RW    |  00000100h
/// C8h        | Misc Control Register 2 (MISCCR2)                 |  32   |  RW    |  00000002h
/// CCh        | Misc Control Register 3 (MISCCR3)                 |  32   |  RW    |  00000000h
/// D0h        | Misc Control Register 4 (MISCCR4)                 |  32   |  RW    |  00000000h
/// D4h        | Misc Control Register 5 (MISCCR5)                 |  32   |  RW    |  00000000h
/// D8h        | Misc Control Register 6 (MISCCR6)                 |  32   |  RW    |  00000000h
/// DCh        | Misc Control Register 7 (MISCCR7)                 |  32   |  RW    |  00000000h
/// E0h        | Status Register 0 (STS0)                          |  32   |  RO    |  00000003h
/// E4h        | Status Register 1 (STS1)                          |  32   |  RO    |  00000000h
/// E8h        | Status Register 2 (STS2)                          |  32   |  RO    |  01000100h
/// ECh        | AHB Suspend Status Register (AHBSPNDSTS)          |  32   |  RO    |  00000000h
/// F0h        | IP RX FIFO Status Register (IPRXFSTS)             |  32   |  RO    |  00000000h
/// F4h        | IP TX FIFO Status Register (IPTXFSTS)             |  32   |  RO    |  00000000h
/// 100h - 17Ch| IP RX FIFO Data a (RFDR0 - RFDR31)                |  32   |  RO    |  00000000h
/// 180h - 1FCh| IP TX FIFO Data a (TFDR0 - TFDR31)                |  32   |  WO    |  00000000h
/// 200h - 3FCh| LUT a (LUT0 - LUT127)                             |  32   |  RW    |  See description.
///
typedef struct {
  UINT32  Mcr0;
  UINT32  Mcr1;
  UINT32  Mcr2;
  UINT32  AhbCr;
  UINT32  IntEn;
  UINT32  Intr;
  UINT32  LutKey;
  UINT32  LutCr;
  UINT32  AhbRxBuf0Cr0;
  UINT32  AhbRxBuf1Cr0;
  UINT32  AhbRxBuf2Cr0;
  UINT32  AhbRxBuf3Cr0;
  UINT32  AhbRxBuf4Cr0;
  UINT32  AhbRxBuf5Cr0;
  UINT32  AhbRxBuf6Cr0;
  UINT32  AhbRxBuf7Cr0;
  UINT32  AhbRxBuf0Cr1;
  UINT32  AhbRxBuf1Cr1;
  UINT32  AhbRxBuf2Cr1;
  UINT32  AhbRxBuf3Cr1;
  UINT32  AhbRxBuf4Cr1;
  UINT32  AhbRxBuf5Cr1;
  UINT32  AhbRxBuf6Cr1;
  UINT32  AhbRxBuf7Cr1;
  UINT32  FlshA1Cr0;
  UINT32  FlshA2Cr0;
  UINT32  FlshB1Cr0;
  UINT32  FlshB2Cr0;
  UINT32  FlshA1Cr1;
  UINT32  FlshA2Cr1;
  UINT32  FlshB1Cr1;
  UINT32  FlshB2Cr1;
  UINT32  FlshA1Cr2;
  UINT32  FlshA2Cr2;
  UINT32  FlshB1Cr2;
  UINT32  FlshB2Cr2;
  UINT32  FlshCr3;
  UINT32  FlshCr4;
  UINT32  FlshCr5;
  UINT32  FlshCr6;
  UINT32  IpCr0;
  UINT32  IpCr1;
  UINT32  IpCr2;
  UINT32  IpCr3;
  UINT32  IpCmd;
  UINT32  Dlpr;
  UINT32  IpRxfCr;
  UINT32  IpTxfCr;
  UINT32  DllaCr;
  UINT32  DllbCr;
  UINT32  MiscCr2;
  UINT32  MiscCr3;
  UINT32  MiscCr4;
  UINT32  MiscCr5;
  UINT32  MiscCr6;
  UINT32  MiscCr7;
  UINT32  Sts0;
  UINT32  Sts1;
  UINT32  Sts2;
  UINT32  AhbSpndSts;
  UINT32  IpRxfSts;
  UINT32  IpTxfSts;
  UINT32  Reserved[2];
  UINT32  Rfdr[32];
  UINT32  Tfdr[32];
  UINT32  Lut[128];
} FSPI_REGISTERS;

typedef enum {
  FSPI_CHIP_SELECT_0 = 0,
  FSPI_CHIP_SELECT_1,
  FSPI_CHIP_SELECT_2,
  FSPI_CHIP_SELECT_3,
  FSPI_CHIP_SELECT_MAX
} FSPI_CHIP_SELECT;

typedef struct {
  ///
  /// Handle on which EFI_SPI_HC_PROTOCOL and DEVICE_PATH_PROTOCOL are installed
  ///
  EFI_HANDLE                         Handle;
  ///
  /// Base address of FSPI controller registers
  ///
  FSPI_REGISTERS                     *Regs;
  ///
  /// First address of FlexSPI address space on system memory map.
  ///
  EFI_PHYSICAL_ADDRESS               AmbaBase;
  ///
  /// Offset of current chip-select in system memory map relative to AmbaBase.
  ///
  UINT32                             CurCSOffset;
  ///
  /// Total number of Chip-selects attached to FSPI controller
  ///
  UINT32                             NumChipselect;
  ///
  /// EFI_SPI_HC_PROTOCOL to be installed on Handle
  ///
  EFI_SPI_HC_PROTOCOL                FspiHcProtocol;
  ///
  /// FSPI controller device path
  ///
  EFI_SPI_DEVICE_PATH                DevicePath;
  ///
  /// FSPI controller Register Read function
  ///
  UINT32                             (*Read32) (IN  UINTN  Address);
  ///
  /// FSPI controller Register write function
  ///
  UINT32                             (*Write32) (IN  UINTN  Address, IN  UINT32  Value);
  ///
  /// FSPI controller Register ClearBits function
  ///
  UINT32                             (*ClearBits32) (IN  UINTN  Address, IN  UINT32  Mask);
  ///
  /// FSPI controller Register SetBits function
  ///
  UINT32                             (*SetBits32) (IN  UINTN  Address, IN  UINT32  Bits);
  ///
  /// FSPI controller Register ClearSet function
  ///
  UINT32                             (*ClearSet32) (IN  UINTN  Address, IN  UINT32  Mask, IN  UINT32  Value);
  ///
  /// The Input clock frequency to FSPI module in Hz.
  ///
  UINT64                             ClockHz;
  ///
  /// if the Fspi controller is runtime, then VirtualNotifyEvent
  ///
  EFI_EVENT                          Event;
} FSPI_MASTER;

/**
 This function parses the device tree and for each FSPI controller node found,
 allocates runtime memory for the internal data structure containing a handle and installs
 the SpiMaster Protocol and Device Path protocol on to that handle.

 @param[in]  Fdt                 Platform's device tree blob
 @param[out] FspiCount        Total number of Flex SPI controllers found.

 @retval EFI_UNSUPPORTED         The device tree node properties of FSPI controller are
                                 not supported by driver
 @retval EFI_INVALID_PARAMETER   No Parent node of FSPI node found in device tree.
 @retval EFI_OUT_OF_RESOURCES    No pool memory available
 @retval EFI_ALREADY_STARTED     The controller is busy with another transaction.
 @retval EFI_DEVICE_ERROR        There was an error during installing protocols.
 @retval EFI_SUCCESS             The protocols were installed successfully.
**/
EFI_STATUS
ParseDeviceTree (
  IN  VOID         *Fdt,
  OUT UINTN        *FspiCount
  );

/**
 Configure The Fspi controller at startup
 FlexSPI controller iniitialization sequence is as following:
  - Enable controller clocks (AHB clock/IP Bus clock/Serial root clock) in System level.
  - Set MCR0[MDIS] to 0x1 (Make sure controller is configured in module stop mode)
  - Configure module control registers: MCR0, MCR1, MCR2. (Don't change MCR0[MDIS])
  - Configure AHB bus control register (AHBCR) and AHB RX Buffer control registers
    (AHBRXBUFxCR) optionally if AHB command will be used
  - Configure Flash control registers (FLSHxCR0,FLSHxCR1,FLSHxCR2) according to external device type
  - Configure DLL control register (DLLxCR) according to sample clock source selection
  - set MCR0[MDIS] to 0x0 (Exit module stop mode)
  - Configure LUT as needed (For AHB command or IP command)
  - Reset controller optionally (by set MCR0[SWRESET] to 0x1)

 @param[in] Fspi       Pointer to FSPI_MASTER structure of a FSPI controller
 @param[in] AmbaTotalSize Total Size for Memory mapped SPI flash devices

 @retval EFI_TIMEOUT    Timeout occured while configuring Fspi Controller
         EFI_SUCCESS    Successfully configured the controller
         EFI_DEVICE_ERROR  Error Occurred while setting up the controller
**/
EFI_STATUS
FspiSetup (
  IN  FSPI_MASTER            *Fspi,
  IN  UINT64                 AmbaTotalSize
  );

/**
  This function performs the WRITE_THEN_READ operation on SPI device.

  The FlexSPI module has two different methods for reading from the external device:
  IP commands and AHB commands.
  Each of these methods has its own internal buffers.

  Before calling this function ensure that a correct read sequence in programmed in the LUT in
  accordance with the device connected on board. and the valid address, number of data bytes,
  and buffer to read data into is provided to this function. The LUT entry to use, should also
  be provided to this function.

  As programmed in LUT entry, first some control information is sent to SPI device
  (like command, address etc) and then the device output data which is read into internal
  buffers. Hence the operation is WRITE_THEN_READ. Then the data is copied from internal
  buffers to buffer provided in FSPI Request structure.

  @param[in]   Fspi          FSPI_MASTER structure of a FSPI controller
  @param[in]   Request          FSPI Request structure which provides the LUT entry to use,
                                Address to read from, number of data bytes to read and buffer
                                in which data is to be read.

  @retval EFI_ALREADY_STARTED   The controller is busy with another transaction.
  @retval EFI_DEVICE_ERROR      There was an SPI error during the transaction.
  @retval EFI_SUCCESS           The transaction completed successfully.
**/
EFI_STATUS
ReadTransaction (
  IN  FSPI_MASTER      *Fspi,
  IN  FSPI_REQUEST     *Request
  );

/**
  This function performs the WRITE_ONLY operation on SPI device.

  The FlexSPI module has only one method for writing to the external device:
  IP commands.
  This method has its own internal buffers.

  Before calling this function ensure that a correct write sequence in programmed in the LUT in
  accordance with the device connected on board. and the valid address, number of data bytes,
  and buffer to write data from is provided to this function. The LUT entry to use, should also
  be provided to this function.

  First the data to be written is copied from buffer provided to internal buffer.
  Then, as programmed in LUT entry, first some control information is sent to SPI device
  (like command, address etc) and then the device is able to receive data which is
  written into internal buffers.

  @param[in]   Fspi          FSPI_MASTER structure of a FSPI controller
  @param[in]   Request          FSPI Request structure which provides the LUT entry to use,
                                Address to write to, number of data bytes to write and buffer
                                from which data is to be written.

  @retval EFI_ALREADY_STARTED   The controller is busy with another transaction.
  @retval EFI_DEVICE_ERROR      There was an SPI error during the transaction.
  @retval EFI_SUCCESS           The transaction completed successfully.
**/
EFI_STATUS
WriteTransaction (
  IN  FSPI_MASTER      *Fspi,
  IN  FSPI_REQUEST     *Request
  );

/**
  This function converts incoming SPI request to FSPI request
  Fspi Controller supports write only and write then read type of requests.
  Incoming SPI request packet is parsed and the type of request is determined.
  if the request is supported by FSPI controller, FSPI request data structure is
  filled with values needed to complete the request.

  Additionally the LUT table entries are also programmed, which will be required
  to complete the request. Which LUT entry is used, is specified by Request parameter.

  @param[in]   Fspi                FSPI_MASTER structure of a FSPI controller
  @param[in]   RequestPacket          Incoming SPI request packet
  @param[in, out]  Request            FSPI Request generated after parsing RequestPacket
                                      on Input this provides the LUT table entry to fill.

  @retval EFI_INVALID_PARAMETER       The parameters specified in RequestPacket are not
                                      Valid or the input parameters to function are null
  @retval EFI_UNSUPPORTED             The incoming request packet is not supported by FSPI
                                      controller. EFI_SPI_CONTROLLER_CAPABILITIES field of
-                                     EFI_SPI_HC_PROTOCOL can be checked to debug this.
  @retval EFI_SUCCESS                 The Incoming RequestPacket is supported and specified
                                      Lut entry has been filled and FSPI Request structure is
                                      filled with values needed to complete request packet.
**/
EFI_STATUS
ParseRequest(
  IN      FSPI_MASTER                   *Fspi,
  IN      EFI_SPI_REQUEST_PACKET        *RequestPacket,
  IN OUT  FSPI_REQUEST                  *Request;
  );

/**
  Perform the SPI transaction on the SPI peripheral using the SPI host
  controller.

  This routine is called at TPL_NOTIFY.
  This routine synchronously returns EFI_SUCCESS indicating that the
  asynchronous SPI transaction was started. The routine then waits for
  completion of the SPI transaction prior to returning the final transaction
  status.

  @param[in] This            Pointer to an EFI_SPI_HC_PROTOCOL structure.
  @param[in] RequestPacket   Pointer to a EFI_SPI_REQUEST_PACKET containing
                             the description of the SPI transactions to perform.

  @retval EFI_SUCCESS             The transaction completed successfully.
  @retval EFI_ALREADY_STARTED     The controller is busy with another transaction.
  @retval EFI_BAD_BUFFER_SIZE     The Length value in SPI Transaction is wrong.
  @retval EFI_DEVICE_ERROR        There was an SPI error during the transaction.
  @retval EFI_INVALID_PARAMETER   The parameters specified in RequestPacket are not
                                  Valid. or the RequestPacket is NULL.
  @retval EFI_NO_RESPONSE         The SPI device is not responding to the slave
                                  address.  EFI_DEVICE_ERROR will be returned if
                                  the controller cannot distinguish when the NACK
                                  occurred.
  @retval EFI_UNSUPPORTED         The controller does not support the requested
                                  transaction.
**/
EFI_STATUS
FspiTransaction(
  IN CONST EFI_SPI_HC_PROTOCOL     *This,
  IN EFI_SPI_REQUEST_PACKET        *RequestPacket
  );

/**
  Set up the clock generator to produce the correct clock frequency, phase and
  polarity for a SPI chip.

  This routine is called at TPL_NOTIFY.
  This routine updates the clock generator to generate the correct frequency
  and polarity for the SPI clock.

  @param[in] This           Pointer to an EFI_SPI_HC_PROTOCOL structure.
  @param[in] SpiPeripheral  Pointer to a EFI_SPI_PERIPHERAL data structure from
                            which the routine can access the ClockParameter,
                            ClockPhase and ClockPolarity fields. The routine
                            also has access to the names for the SPI bus and
                            chip which can be used during debugging.
  @param[in] ClockHz        Pointer to the requested clock frequency. The SPI
                            host controller will choose a supported clock
                            frequency which is less then or equal to this
                            value. Specify zero to turn the clock generator
                            off. The actual clock frequency supported by the
                            SPI host controller will be returned.

  @retval EFI_SUCCESS      The clock was set up successfully
  @retval EFI_UNSUPPORTED  The SPI controller was not able to support the
                           frequency requested by ClockHz

**/
EFI_STATUS
FspiClock (
  IN CONST EFI_SPI_HC_PROTOCOL  *This,
  IN CONST EFI_SPI_PERIPHERAL   *SpiPeripheral,
  IN UINT32                     *ClockHz
  );

/**
  Assert or deassert the SPI chip select.

  This routine is called at TPL_NOTIFY.
  Update the value of the chip select line for a SPI peripheral. The SPI bus
  layer calls this routine either in the board layer or in the SPI controller
  to manipulate the chip select pin at the start and end of a SPI transaction.

  @param[in] This           Pointer to an EFI_SPI_HC_PROTOCOL structure.
  @param[in] SpiPeripheral  The address of an EFI_SPI_PERIPHERAL data structure
                            describing the SPI peripheral whose chip select pin
                            is to be manipulated. The routine may access the
                            ChipSelectParameter field to gain sufficient
                            context to complete the operation.
  @param[in] PinValue       The value to be applied to the chip select line of
                            the SPI peripheral.

  @retval EFI_SUCCESS            The chip select was set as requested
  @retval EFI_NOT_READY          Support for the chip select is not properly
                                 initialized
  @retval EFI_INVALID_PARAMETER  The ChipSeLect value or its contents are
                                 invalid

**/
EFI_STATUS
FspiChipSelect (
  IN CONST EFI_SPI_HC_PROTOCOL  *This,
  IN CONST EFI_SPI_PERIPHERAL   *SpiPeripheral,
  IN BOOLEAN                    PinValue
  );

/**
  This routine is called by SPI bus layer to configure Host Controller from
  SpiIO.UpdateSpiPeripheral.

  Support socketed SPI parts by allowing the SPI peripheral driver to replace
  the SPI peripheral after the connection is made. An example use is socketed
  SPI NOR flash parts, where the size and parameters change depending upon
  device is in the socket.

  @param[in] This           Pointer to an EFI_SPI_HC_PROTOCOL structure.
  @param[in] SpiPeripheral  Pointer to a EFI_SPI_PERIPHERAL data structure from
                            which the routine can access the ConfigurationData.
			    The routine also has access to the names for the SPI bus and
                            chip which can be used during debugging.

  @retval EFI_SUCCESS       The SPI peripheral was updated successfully
  @retval EFI_DEVICE_ERROR  NOT able to update the SPI peripheral
  @retval EFI_INVALID_PARAMETER The ChipSeLect value or its contents are
                                invalid
**/
EFI_STATUS
FspiUpdateSpiPeripheral (
  IN CONST EFI_SPI_HC_PROTOCOL  *This,
  IN CONST EFI_SPI_PERIPHERAL   *SpiPeripheral
  );

/**
 Installs the SpiMaster Protocol and Device Path protocol on to that handle.

 Also this function registers for an Virtual Address change event, to convert the runtime
 memory allocated from physical address space to virtual address space.

 @param[in]   Fspi      Pointer to FSPI_MASTER strcture of a FSPI controller
 @param[in]   Runtime      Weather Fspi controller is to be used at runtime or not

 @retval EFI_DEVICE_ERROR      Not able to Install SPI Host Controller Protocol or if the Fspi
                               Controller is runtime, then not able to set the memory attributes for this controller.
 @retval EFI_SUCCESS           Protocols installed successfully on FSPI controllers' handles.
**/
EFI_STATUS
EFIAPI
FspiInstallProtocol (
  IN  FSPI_MASTER      *Fspi,
  IN  BOOLEAN          Runtime
  );

#endif //__FSPI_DXE_H__
