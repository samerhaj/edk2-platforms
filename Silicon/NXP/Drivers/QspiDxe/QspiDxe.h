/** @QspiDxe.h
  Header Defining The Qspi Flash Controller Constants (Base Addresses, Sizes,
  Flags), Function Prototype, Structures etc

  Copyright 2017 NXP

  This Program And The Accompanying Materials
  Are Licensed And Made Available Under The Terms And Conditions Of The BSD
  License Which Accompanies This Distribution. The Full Text Of The License
  May Be Found At
  http://Opensource.Org/Licenses/Bsd-License.Php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __QSPI_DXE_H__
#define __QSPI_DXE_H__

#include <Pi/PiSpi.h>
#include <Protocol/SpiHc.h>
#include <libfdt.h>

#define TX_BUFFER_SIZE       (0x40)
#define RX_BUFFER_SIZE       (0x80)

#define LCKCR_LOCK           (0x1)
#define LCKCR_UNLOCK         (0x2)
#define LUT_KEY              (0x5af05af0)

/*
 * The definition of the LUT register shows below:
 *
 *  ---------------------------------------------------
 *  | INSTR1 | PAD1 | OPRND1 | INSTR0 | PAD0 | OPRND0 |
 *  ---------------------------------------------------
 */
#define LUT_OPRND0_SHIFT        (0)
#define LUT_PAD0_SHIFT          (8)
#define LUT_INSTR0_SHIFT        (10)
#define LUT_OPRND1_SHIFT        (16)

#define LUT_CMD              (1)
#define LUT_ADDR             (2)
#define LUT_DUMMY            (3)
#define LUT_MODE             (4)
#define LUT_MODE2            (5)
#define LUT_MODE4            (6)
#define LUT_READ             (7)
#define LUT_WRITE            (8)

#define LUT_PAD_1            (0)
#define LUT_PAD_2            (1)
#define LUT_PAD_4            (2)

#define SEQ_ID_IP_MODE       (1) // LUT entry to use

#define IPCR_SEQID_SHIFT     (24)

/* Module Configuration */
#define MCR_CLR_RXF_MASK       BIT10 // Clear RX FIFO.
#define MCR_CLR_TXF_MASK       BIT11 // Clear TX FIFO/Buffer
#define MCR_MDIS_MASK          BIT14 // Module Disable
#define MCR_IDLE_SIGNAL_DRIVE  BIT16|BIT17|BIT18|BIT19
#define MCR_END_CFD_SHIFT      (2)
#define MCR_END_CFD_MASK       (3 << MCR_END_CFD_SHIFT)
#define MCR_END_CFD_32BIT_LE   (1 << MCR_END_CFD_SHIFT)
#define MCR_SWRSTHD_MASK       BIT1 // Software reset for AHB domain
#define MCR_SWRSTSD_MASK       BIT0 // Software reset for Serial Flash domain

// Sampling Register
#define SMPR_HSENA_MASK      BIT0 // Half Speed serial flash clock Enable
#define SMPR_HSPHS_MASK      BIT1 // Half Speed Phase selection for SDR instructions.
#define SMPR_FSPHS_MASK      BIT5 // Full Speed Phase selection for SDR instructions.
#define SMPR_FSDLY_MASK      BIT6 // Full Speed Delay selection for SDR instructions
#define SMPR_DDRSMP_MASK     BIT16|BIT17|BIT18 // DDR Sampling point

// RX Buffer Control Register
#define RBCT_RXBRD_USEIPS    BIT8 // RX Buffer Readout
                                  // 0 RX Buffer content is read using the AHB Bus registers QSPI_ARDB0 to QSPI_ARDB31.
                                  // 1 RX Buffer content is read using the IP Bus registers QSPI_RBDR0 to QSPI_RBDR31.

// RX Buffer Status Register
#define RBSR_RDBFL_SHIFT     (8)      // RX Buffer Fill Level, indicates how many entries of 4 bytes
#define RBSR_RDBFL_MASK      (0x3f00) // are still available in the RX Buffer

// Status Register Bits
#define SR_TXFULL            BIT27 // TX Buffer Full
#define SR_RXWE              BIT16 // RX Buffer Watermark Exceeded
#define SR_AHB_ACC           BIT2 // AHB Access
#define SR_IP_ACC            BIT1 // IP Access
#define SR_BUSY              BIT0 // Module Busy

// Flasg Register Bits
#define FR_TFF               BIT0  // IP Command Transaction Finished Flag.
#define FR_IPGEF             BIT4  // IP Command Trigger during AHB Grant Error Flag
#define FR_IPIEF             BIT6  // IP Command Trigger could not be executed Error Flag
#define FR_IPAEF             BIT7  // IP Command Trigger during AHB Access Error Flag.
#define FR_IUEF              BIT11 // IP Command Usage Error Flag
#define FR_ABOF              BIT12 // AHB Buffer Overflow Flag
#define FR_AIBSEF            BIT13 // AHB Illegal Burst Size Error Flag
#define FR_AITEF             BIT14 // AHB Illegal transaction error flag
#define FR_ABSEF             BIT15 // AHB Sequence Error Flag
#define FR_RBDF              BIT16 // RX Buffer Drain Flag
#define FR_RBOF              BIT17 // RX Buffer Overflow Flag
#define FR_ILLINE            BIT23 // Illegal Instruction Error Flag
#define FR_TBUF              BIT26 // TX Buffer Underrun Flag
#define FR_TBFF              BIT27 // TX Buffer Fill Flag:
#define FR_KFEF              BIT29 // Key Fetch Error Flag
#define FR_IAKFEF            BIT30 // Illegal Access during Key Fetch Error Flag:
#define FR_DLPFF             BIT31 // Data Learning Pattern Failure Flag

// TX Buffer Status Register
#define TBSR_TRBL_MASK       (0x1F00) // TX Buffer Fill Level. The TRBFL field contains the number of
#define TBSR_TRBL_SHIFT      (8)      // entries of 4 bytes each available in the TX Buffer

#define QSPI_SR_RETRY_COUNT  (20) // Retries to test for ongoing transaction completion.

///
/// Qspi Controller supports write only and write then read type of requests.
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
} QSPI_REQUEST;

///
/// Qspi Controller Registers
///
typedef struct {
  UINT32 Mcr;        // offset 0x00
  UINT32 Rsvd0[1];
  UINT32 Ipcr;       // offset 0x08
  UINT32 Flshcr;     // offset 0x0c
  UINT32 Buf0cr;     // offset 0x10
  UINT32 Buf1cr;     // offset 0x14
  UINT32 Buf2cr;     // offset 0x18
  UINT32 Buf3cr;     // offset 0x1c
  UINT32 Bfgencr;    // offset 0x20
  UINT32 Soccr;      // offset 0x24
  UINT32 Rsvd1[2];
  UINT32 Buf0ind;    // offset 0x30
  UINT32 Buf1ind;    // offset 0x34
  UINT32 Buf2ind;    // offset 0x38
  UINT32 Rsvd2[49];
  UINT32 Sfar;       // offset 0x100
  UINT32 Sfacr;      // offset 0x104
  UINT32 Smpr;       // offset 0x108
  UINT32 Rbsr;       // offset 0x10c
  UINT32 Rbct;       // offset 0x110
  UINT32 Rsvd3[15];
  UINT32 Tbsr;       // offset 0x150
  UINT32 Tbdr;       // offset 0x154
  UINT32 Rsvd4[1];
  UINT32 Sr;         // offset 0x15c
  UINT32 Fr;         // offset 0x160
  UINT32 Rser;       // offset 0x164
  UINT32 Spndst;     // offset 0x168
  UINT32 Sptrclr;    // offset 0x16c
  UINT32 Rsvd5[4];
  UINT32 Sfa1ad;     // offset 0x180
  UINT32 Sfa2ad;     // offset 0x184
  UINT32 Sfb1ad;     // offset 0x188
  UINT32 Sfb2ad;     // offset 0x18c
  UINT32 Rsvd6[28];
  UINT32 Rbdr[32];   // offset 0x200
  UINT32 Rsvd7[32];
  UINT32 Lutkey;     // offset 0x300
  UINT32 Lckcr;      // offset 0x304
  UINT32 Rsvd8[2];
  UINT32 Lut[64];    // offset 0x310
} QSPI_REGISTERS;

typedef enum {
  QSPI_CHIP_SELECT_0 = 0,
  QSPI_CHIP_SELECT_1,
  QSPI_CHIP_SELECT_2,
  QSPI_CHIP_SELECT_3,
  QSPI_CHIP_SELECT_MAX
} QSPI_CHIP_SELECT;

typedef struct {
  ///
  /// Handle on which EFI_SPI_HC_PROTOCOL and DEVICE_PATH_PROTOCOL are installed
  ///
  EFI_HANDLE                         Handle;
  ///
  /// Base address of QSPI controller registers
  ///
  QSPI_REGISTERS                     *Regs;
  ///
  /// First address of QuadSPI address space on system memory map.
  ///
  UINTN                              AmbaBase;
  ///
  /// address of current chip-select on system memory map.
  ///
  UINTN                              CurAmbaBase;
  ///
  /// Total number of Chip-selects attached to QSPI controller
  ///
  UINT32                             NumChipselect;
  ///
  /// EFI_SPI_HC_PROTOCOL to be installed on Handle
  ///
  EFI_SPI_HC_PROTOCOL                QspiHcProtocol;
  ///
  /// QSPI controller device path
  ///
  EFI_SPI_DEVICE_PATH                DevicePath;
  ///
  /// QSPI controller Register Read function
  ///
  UINT32                             (*QspiRead32) (IN  UINTN  Address);
  ///
  /// QSPI controller Register write function
  ///
  UINT32                             (*QspiWrite32) (IN  UINTN  Address, IN  UINT32  Value);
  ///
  /// List of QSPI controllers
  ///
  LIST_ENTRY                         Link;
} QSPI_MASTER;

/**
 This function parses the device tree and for each QSPI controller node found,
 allocates runtime memory for the internal data structure containing a handle and installs
 the SpiMaster Protocol and Device Path protocol on to that handle.

 @param[in]  Fdt                 Platform's device tree blob
 @param[out] QMasterList         Link List of QSPI_MASTER structures that are runtime
                                 each member corresponds to one QSPI controller.

 @retval EFI_UNSUPPORTED         The device tree node properties of QSPI controller are
                                 not supported by driver
 @retval EFI_INVALID_PARAMETER   No Parent node of QSPI node found in device tree.
 @retval EFI_OUT_OF_RESOURCES    No pool memory available
 @retval EFI_ALREADY_STARTED     The controller is busy with another transaction.
 @retval EFI_DEVICE_ERROR        There was an error during installing protocols.
 @retval EFI_SUCCESS             The protocols were installed successfully.
**/
EFI_STATUS
ParseDeviceTree (
  IN  VOID         *Fdt,
  OUT LIST_ENTRY   *QMasterList
  );

/**
 Configure sampling of incoming data

 @param[in] QMaster     Pointer to QSPI_MASTER structure of a QSPI controller
 @param[in] Clear       Clear Bits of SMPR
 @param[in] Set         Set Bits of SMPR
**/
VOID
QspiConfigureSampling (
  IN  QSPI_MASTER    *QMaster,
  IN  UINT32         Clear,
  IN  UINT32         Set
  );

/**
 Clears(puts) the QSPI controller from(into) Module Disabled state.

 The MDIS bit allows the clock to the non-memory mapped logic in the QuadSPI to be
 stopped, putting the QuadSPI in a software controlled power-saving state.

 @param[in]  QMaster     Pointer to QSPI_MASTER structure of a QSPI controller
 @param[in]  Enable      FALSE : Set Module Disable (MDS) bit of MCR Register
                          i.e. Disable QuadSPI clocks.
                         TRUE : clear MDS bit i.e. Enable QuadSPI clocks.
**/
VOID
EnableQspiModule (
  IN  QSPI_MASTER      *QMaster,
  IN  BOOLEAN          Enable
  );

/**
  Enable or Disable the QSPI controller software reset

  It is advisable to reset both the serial flash domain and AHB domain at the same time.
  Resetting only one domain might lead to side effects.

  The software resets need the clock to be running to propagate to the design. The
  MCR[MDIS] should therefore be set to 0 when the software reset bits are asserted. Also,
  before they can be de-asserted again (by setting MCR[SWRSTHD] and MCR[SWRSTSD] to 0),
  it is recommended to set the MCR[MDIS] bit to 1.
  Once the software resets have been de-asserted, the normal operation can be started by
  setting the MCR[MDIS] bit to 0.

  @param[in]   QMaster          QSPI_MASTER structure of a QSPI controller
  @param[in]   ResetEn          TRUE: Enable QSPI controller software reset
                                FALSE: Disable QSPI controller software reset
**/
VOID
QspiSwReset (
  IN  QSPI_MASTER       *QMaster,
  IN  BOOLEAN           ResetEn
  );

/**
  This function performs the WRITE_THEN_READ operation on SPI device.

  The QuadSPI module has two different methods for reading from the external device:
  IP commands and AHB commands.
  Each of these methods has its own internal buffers.

  Before calling this function ensure that a correct read sequence in programmed in the LUT in
  accordance with the device connected on board. and the valid address, number of data bytes,
  and buffer to read data into is provided to this function. The LUT entry to use, should also
  be provided to this function.

  As programmed in LUT entry, first some control information is sent to SPI device
  (like command, address etc) and then the device output data which is read into internal
  buffers. Hence the operation is WRITE_THEN_READ. Then the data is copied from internal
  buffers to buffer provided in QSPI Request structure.

  @param[in]   QMaster          QSPI_MASTER structure of a QSPI controller
  @param[in]   Request          QSPI Request structure which provides the LUT entry to use,
                                Address to read from, number of data bytes to read and buffer
                                in which data is to be read.

  @retval EFI_ALREADY_STARTED   The controller is busy with another transaction.
  @retval EFI_DEVICE_ERROR      There was an SPI error during the transaction.
  @retval EFI_SUCCESS           The transaction completed successfully.
**/
EFI_STATUS
ReadTransaction (
  IN  QSPI_MASTER      *QMaster,
  IN  QSPI_REQUEST     *Request
  );

/**
  This function performs the WRITE_ONLY operation on SPI device.

  The QuadSPI module has only one method for writing to the external device:
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

  @param[in]   QMaster          QSPI_MASTER structure of a QSPI controller
  @param[in]   Request          QSPI Request structure which provides the LUT entry to use,
                                Address to write to, number of data bytes to write and buffer
                                from which data is to be written.

  @retval EFI_ALREADY_STARTED   The controller is busy with another transaction.
  @retval EFI_DEVICE_ERROR      There was an SPI error during the transaction.
  @retval EFI_SUCCESS           The transaction completed successfully.
**/
EFI_STATUS
WriteTransaction (
  IN  QSPI_MASTER      *QMaster,
  IN  QSPI_REQUEST     *Request
  );

/**
  This function converts incoming SPI request to QSPI request
  Qspi Controller supports write only and write then read type of requests.
  Incoming SPI request packet is parsed and the type of request is determined.
  if the request is supported by QSPI controller, QSPI request data structure is
  filled with values needed to complete the request.

  Additionally the LUT table entries are also programmed, which will be required
  to complete the request. Which LUT entry is used, is specified by Request parameter.

  @param[in]   QMaster                QSPI_MASTER structure of a QSPI controller
  @param[in]   RequestPacket          Incoming SPI request packet
  @param[in, out]  Request            QSPI Request generated after parsing RequestPacket
                                      on Input this provides the LUT table entry to fill.

  @retval EFI_INVALID_PARAMETER       The parameters specified in RequestPacket are not
                                      Valid or the input parameters to function are null
  @retval EFI_UNSUPPORTED             The incoming request packet is not supported by QSPI
                                      controller. EFI_SPI_CONTROLLER_CAPABILITIES field of
-                                     EFI_SPI_HC_PROTOCOL can be checked to debug this.
  @retval EFI_SUCCESS                 The Incoming RequestPacket is supported and specified
                                      Lut entry has been filled and QSPI Request structure is
                                      filled with values needed to complete request packet.
**/
EFI_STATUS
ParseRequest(
  IN      QSPI_MASTER               *QMaster,
  IN      EFI_SPI_REQUEST_PACKET    *RequestPacket,
  IN OUT  QSPI_REQUEST              *Request;
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
QspiTransaction(
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
QspiSetupPeripheral (
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
QspiChipSelect (
  IN CONST EFI_SPI_HC_PROTOCOL  *This,
  IN CONST EFI_SPI_PERIPHERAL   *SpiPeripheral,
  IN BOOLEAN                    PinValue
  );

#endif //__QSPI_DXE_H__
