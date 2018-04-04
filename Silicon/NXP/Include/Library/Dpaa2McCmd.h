/** Dpaa2McCmd.h
   DPAA2 Management Complex (MC) command interface

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef _DPAA2_MC_CMD_H
#define _DPAA2_MC_CMD_H

#include <Bitops.h>
#include <Library/IoLib.h>
#include <Uefi.h>

#define MC_CMD_NUM_OF_PARAMS  7

/**
 * Layout of an MC command
 */
typedef struct mc_command {
  UINT64 header;
  UINT64 params[MC_CMD_NUM_OF_PARAMS];
} DPAA2_MC_COMMAND;

typedef enum _MC_CMD_STATUS {
  MC_CMD_STATUS_OK = 0x0, /*!< Completed successfully */
  MC_CMD_STATUS_READY = 0x1, /*!< Ready to be processed */
  MC_CMD_STATUS_AUTH_ERR = 0x3, /*!< Authentication error */
  MC_CMD_STATUS_NO_PRIVILEGE = 0x4, /*!< No privilege */
  MC_CMD_STATUS_DMA_ERR = 0x5, /*!< DMA or I/O error */
  MC_CMD_STATUS_CONFIG_ERR = 0x6, /*!< Configuration error */
  MC_CMD_STATUS_TIMEOUT = 0x7, /*!< Operation timed out */
  MC_CMD_STATUS_NO_RESOURCE = 0x8, /*!< No resources */
  MC_CMD_STATUS_NO_MEMORY = 0x9, /*!< No memory available */
  MC_CMD_STATUS_BUSY = 0xA, /*!< Device is busy */
  MC_CMD_STATUS_UNSUPPORTED_OP = 0xB, /*!< Unsupported operation */
  MC_CMD_STATUS_INVALID_STATE = 0xC /*!< Invalid state */
} MC_CMD_STATUS;

/*
 * Typedefs for integer types used in MC Flibs
 */
typedef UINT8 uint8_t;
typedef UINT16 uint16_t;
typedef UINT32 uint32_t;
typedef UINT64 uint64_t;

/*
 * MC command flags
 */

/* High priority flag */
#define MC_CMD_FLAG_PRI    0x00008000
/* No flags */
#define MC_CMD_NO_FLAGS    0x00000000
/* Command completion flag */
#define MC_CMD_FLAG_INTR_DIS  0x01000000


#define MC_CMD_HDR_CMDID_O      52  /* Command ID field offset */
#define MC_CMD_HDR_CMDID_S      12  /* Command ID field size */
#define MC_CMD_HDR_STATUS_O     16  /* Status field offset */
#define MC_CMD_HDR_TOKEN_O      38  /* Token field offset */
#define MC_CMD_HDR_TOKEN_S      10  /* Token field size */
#define MC_CMD_HDR_STATUS_S     8   /* Status field size*/
#define MC_CMD_HDR_FLAGS_O      0   /* Flags field offset */
#define MC_CMD_HDR_FLAGS_S      32  /* Flags field size*/
#define MC_CMD_HDR_FLAGS_MASK   0xFF00FF00 /* Command flags mask */

#define MC_CMD_HDR_READ_STATUS(_Hdr) \
        GET_BIT_FIELD64 (_Hdr, MC_CMD_HDR_STATUS_O, MC_CMD_HDR_STATUS_S)

#define MC_CMD_HDR_READ_TOKEN(_Hdr) \
        GET_BIT_FIELD64 (_Hdr, MC_CMD_HDR_TOKEN_O, MC_CMD_HDR_TOKEN_S)

#define MC_CMD_HDR_READ_CMDID(_Hdr) \
        GET_BIT_FIELD64 (_Hdr, MC_CMD_HDR_CMDID_O, MC_CMD_HDR_CMDID_S)

#define MC_PREP_OP(_Ext, _Param, _Offset, _Width, _Type, _Arg) \
  SET_BIT_FIELD64 ((_Ext)[_Param], _Offset, _Width, _Arg)

#define MC_EXT_OP(_Ext, _Param, _Offset, _Width, _Type, _Arg) \
  ((_Arg) = (_Type) GET_BIT_FIELD64 ((_Ext)[_Param], _Offset, _Width))

#define MC_CMD_OP(_Cmd, _Param, _Offset, _Width, _Type, _Arg) \
  SET_BIT_FIELD64 ((_Cmd).params[_Param], _Offset, _Width, _Arg)

#define MC_RSP_OP(_Cmd, _Param, _Offset, _Width, _Type, _Arg) \
  ((_Arg) = (_Type) GET_BIT_FIELD64 ((_Cmd).params[_Param], _Offset, _Width))

/**
   Encodes the header of an MC command

   @param[in] CmdId     Command ID
   @param[in] CmdFlags  Command flags
   @param[in] Token     MC object token
 **/
STATIC inline
UINT64
mc_encode_cmd_header(UINT16 CmdId,
         UINT32 CmdFlags,
         UINT16 Token)
{
  UINT64 Header = 0x0;

  SET_BIT_FIELD64 (Header, MC_CMD_HDR_CMDID_O, MC_CMD_HDR_CMDID_S, CmdId);
  SET_BIT_FIELD64 (Header, MC_CMD_HDR_FLAGS_O, MC_CMD_HDR_FLAGS_S,
                  CmdFlags & MC_CMD_HDR_FLAGS_MASK);
  SET_BIT_FIELD64 (Header, MC_CMD_HDR_TOKEN_O, MC_CMD_HDR_TOKEN_S, Token);
  SET_BIT_FIELD64 (Header, MC_CMD_HDR_STATUS_O, MC_CMD_HDR_STATUS_S,
                  MC_CMD_STATUS_READY);

  return Header;
}

#endif /* _DPAA2_MC_CMD_H */
