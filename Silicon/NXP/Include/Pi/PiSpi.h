/** @file
  Include file matches things in PI.

  Copyright (c) 2017, Intel Corporation. All rights reserved.<BR>
  Copyright 2017-2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD
  License which accompanies this distribution. The full text of the license may
  be found at http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

  @par Revision Reference:
    This Protocol was introduced in revised UEFI PI Specification 1.6.

**/

#ifndef __PI_SPI_H__
#define __PI_SPI_H__

///
/// Define the SPI flags
///

/// The SPI peripheral/controller supports only half duplex transactions
#define SPI_HALF_DUPLEX                           BIT0
/// The SPI peripheral/controller supports write only transactions.
#define SPI_SUPPORTS_WRITE_ONLY_OPERATIONS        BIT1
/// The SPI peripheral/controller supports Read only transactions.
#define SPI_SUPPORTS_READ_ONLY_OPERATIONS         BIT2
/// The SPI peripheral/controller supports Double Transfer Rate (DTR).
/// DTR : Transfer may be input or output on both the
/// rising and falling edges of the clock.
#define SPI_SUPPORTS_DTR_OPERATIONS               BIT3
/// The SPI peripheral/controller supports a 2-bit data bus
#define SPI_SUPPORTS_2_BIT_DATA_BUS_WIDTH         BIT4
/// The SPI peripheral/controller supports a 4-bit data bus
#define SPI_SUPPORTS_4_BIT_DATA_BUS_WIDTH         BIT5
/// The SPI peripheral/controller supports a 8-bit data bus
#define SPI_SUPPORTS_8_BIT_DATA_BUS_WIDTH         BIT6
/// Transfer size includes the opcode byte
#define SPI_TRANSFER_SIZE_INCLUDES_OPCODE         BIT7
/// Transfer size includes the 4 address bytes
#define SPI_TRANSFER_SIZE_INCLUDES_ADDRESS        BIT8

///
/// SPI Frame Size supported Mask
///
#define  SPI_FRAME_MASK(FrameSize)               (1U << FrameSize)

///
/// Calculate the Clock cycles from number of bytes and BusWidth
///
#define SPI_BYTES_TO_CYCLES(Bytes, BusWidth)     ( ( (Bytes << sizeof (UINT8)) + BusWidth - 1) / BusWidth)
///
/// Calculate the number of bytes from Clock cycles and BusWidth
///
#define SPI_CYCLES_TO_BYTES(Cycles, BusWidth)    ( (Cycles * BusWidth) >> sizeof (UINT8))

///
/// SPI Device Path can be used to describe the device path of both SPI controller
/// and SPI Peripheral.
///
typedef struct {
  ///
  /// Vendor device path specifying Vendor GUID for SPI Host controller or SPI Peripheral.
  ///
  VENDOR_DEVICE_PATH            Vendor;
  ///
  /// Controller device path to distinguish between two instances of SPI controller or SPI Peripheral.
  ///
  CONTROLLER_DEVICE_PATH        Controller;
  ///
  /// Signify the end of Device Path.
  ///
  EFI_DEVICE_PATH_PROTOCOL      End;
} EFI_SPI_DEVICE_PATH;

///
/// Note: The revised UEFI PI 1.6 specification does not specify values for the
///       members below. The order matches the specification.
///
typedef enum {
  ///
  /// Data flowing from the host to the SPI peripheral
  /// or Data flowing from the SPI peripheral to the host
  ///
  SPI_TRANSACTION_DATA = 0,

  ///
  /// Command to send to SPI Peripheral
  ///
  SPI_TRANSACTION_COMMAND,

  ///
  /// Offset in SPI Peripheral from/to which data is to be read/written
  ///
  SPI_TRANSACTION_ADDRESS,

  ///
  /// Optional control bits that follow the address bits.
  /// These bits are driven by the controller if they are specified.
  ///
  /// NOTE This field should be counted in clocks not number of bits received by the
  /// serial flash. The SPI master drives the bus during "mode bits" cycles;
  /// Example: If 8 mode bits are needed with a quad input address phase command, then
  /// length for this type of transaction would be 2.
  ///
  SPI_TRANSACTION_MODE,

  ///
  /// Clock cycles during which no data is transferred to or from a memory.
  /// the master tri-states the bus during "dummy" cycles.
  /// No data (WriteBuffer = NULL and ReadBuffer = NULL) can be specified
  /// in SPI Transaction. if these buffer are specified, then SPI HOST controller must ignore
  /// these buffers.
  ///
  SPI_TRANSACTION_DUMMY,

  ///
  /// Maximum element that should be present in any enum.
  ///
  SPI_TRANSACTION_MAX

} EFI_SPI_TRANSACTION_TYPE;

///
/// SPI Peripheral transaction bus width
///
/// The SPI_TRANSFER_LINE describes the number of data lines to use for
/// a transaction.
///
typedef enum {
  /// One line transfer
  SPI_TRANSACTION_BUS_WIDTH_1 = 1,

  /// Two line transfer
  SPI_TRANSACTION_BUS_WIDTH_2 = 2,

  /// Four line transfer
  SPI_TRANSACTION_BUS_WIDTH_4 = 4,

  /// Eight line transfer
  SPI_TRANSACTION_BUS_WIDTH_8 = 8,

  /// Maximum element that should be present in any enum.
  SPI_TRANSACTION_BUS_WIDTH
} EFI_SPI_TRANSACTION_BUS_WIDTH;

///
/// The EFI_SPI_BUS_TRANSACTION data structure contains the description of the
/// SPI transaction to perform on the host controller.
///
typedef struct _EFI_SPI_BUS_TRANSACTION {
  ///
  /// Type of transaction specified by one of the EFI_SPI_TRANSACTION_TYPE
  /// values.
  ///
  EFI_SPI_TRANSACTION_TYPE TransactionType;

  ///
  /// TRUE if the transaction is being debugged. Debugging may be turned on for
  /// a single SPI transaction. Only this transaction will display debugging
  /// messages. All other transactions with this value set to FALSE will not
  /// display any debugging messages.
  ///
  BOOLEAN                  DebugTransaction;

  ///
  /// SPI bus width in bits: 1, 2, 4, 8
  ///
  UINT32                   BusWidth;

  ///
  /// Transfer may be input or output on both the
  /// rising and falling edges of the clock.
  ///
  BOOLEAN                  DoubleTransferRate;

  ///
  /// Frame size in bits, range: 1 - 32
  ///
  UINT32                   FrameSize;

  /// For TransactionType command/address/data, number of bytes needed to complete the
  /// transaction. (Read or write or both).
  /// e.g. for 3 byte address the Length is 3.
  ///
  /// For TransactionType dummy/mode, the clock cycles needed to complete the
  /// transaction.
  /// e.g. if 1 byte (8 bits) of mode is to be transferred on 2 data bus.
  /// then clock cycles needed are 8/2 = 4. so Length is 4.
  ///
  UINT32                   Length;

  ///
  /// Buffer containing data to send to the SPI peripheral
  /// Frame sizes 1 - 8 bits: UINT8 (one byte) per frame
  /// Frame sizes 7 - 16 bits : UINT16 (two bytes) per frame
  /// Can be NULL if the SPI TransactionType is SPI_TRANSACTION_DUMMY.
  ///
  UINT8                    *WriteBuffer;

  ///
  /// Buffer to receive the data from the SPI peripheral
  /// * Frame sizes 1 - 8 bits: UINT8 (one byte) per frame
  /// * Frame sizes 7 - 16 bits : UINT16 (two bytes) per frame
  /// * Frame sizes 17 - 32 bits : UINT32 (four bytes) per frame
  /// Can be NULL if the SPI TransactionType is SPI_TRANSACTION_DUMMY
  ///
  UINT8                    *ReadBuffer;
} EFI_SPI_BUS_TRANSACTION;

///
/// SPI device request
///
/// The EFI_SPI_REQUEST_PACKET describes a single SPI Operation.
/// which consists of one or more SPI transactions
///
typedef struct {
  ///
  /// Number of elements in the Transaction array
  ///
  UINTN                       TransactionCount;

  ///
  /// Description of the SPI TransactionTransaction
  ///
  EFI_SPI_BUS_TRANSACTION     Transaction [1];
} EFI_SPI_REQUEST_PACKET;

#endif  //  __PI_SPI_H__
