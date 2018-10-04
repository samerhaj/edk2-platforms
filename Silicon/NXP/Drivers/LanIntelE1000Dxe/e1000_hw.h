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

#ifndef _E1000_HW_H_
#define _E1000_HW_H_

#include "e1000_osdep.h"
#include "e1000_regs.h"
#include "e1000_defines.h"

struct e1000_hw;

#ifndef NO_82571_SUPPORT
#define E1000_DEV_ID_82571EB_COPPER           0x105E
#define E1000_DEV_ID_82571EB_FIBER            0x105F
#define E1000_DEV_ID_82571EB_SERDES           0x1060
#define E1000_DEV_ID_82571EB_QUAD_COPPER      0x10A4
#define E1000_DEV_ID_82571PT_QUAD_COPPER      0x10D5
#define E1000_DEV_ID_82571EB_QUAD_FIBER       0x10A5
#define E1000_DEV_ID_82571EB_QUAD_COPPER_LP   0x10BC
#define E1000_DEV_ID_82572EI_COPPER           0x107D
#define E1000_DEV_ID_82572EI_FIBER            0x107E
#define E1000_DEV_ID_82572EI_SERDES           0x107F
#define E1000_DEV_ID_82572EI                  0x10B9
#define E1000_DEV_ID_82573E                   0x108B
#define E1000_DEV_ID_82573E_IAMT              0x108C
#define E1000_DEV_ID_82573L                   0x109A
#endif /* NO_82571_SUPPORT */
#ifndef NO_82574_SUPPORT
#define E1000_DEV_ID_82574L                   0x10D3
#define E1000_DEV_ID_82574LA                  0x10F6
#define E1000_DEV_ID_82583V                   0x150C
#endif /* NO_82574_SUPPORT */
#ifndef NO_80003ES2LAN_SUPPORT
#define E1000_DEV_ID_80003ES2LAN_COPPER_DPT   0x1096
#define E1000_DEV_ID_80003ES2LAN_SERDES_DPT   0x1098
#define E1000_DEV_ID_80003ES2LAN_COPPER_SPT   0x10BA
#define E1000_DEV_ID_80003ES2LAN_SERDES_SPT   0x10BB
#endif /* NO_80003ES2LAN_SUPPORT */
#ifndef NO_ICH8LAN_SUPPORT
#define E1000_DEV_ID_ICH8_IGP_M_AMT           0x1049
#define E1000_DEV_ID_ICH8_IGP_AMT             0x104A
#define E1000_DEV_ID_ICH8_IGP_C               0x104B
#define E1000_DEV_ID_ICH8_IFE                 0x104C
#define E1000_DEV_ID_ICH8_IFE_GT              0x10C4
#define E1000_DEV_ID_ICH8_IFE_G               0x10C5
#define E1000_DEV_ID_ICH8_IGP_M               0x104D
#define E1000_DEV_ID_ICH9_IGP_M               0x10BF
#define E1000_DEV_ID_ICH9_IGP_M_AMT           0x10F5
#define E1000_DEV_ID_ICH9_IGP_M_V             0x10CB
#define E1000_DEV_ID_ICH9_IGP_AMT             0x10BD
#define E1000_DEV_ID_ICH9_BM                  0x10E5
#define E1000_DEV_ID_ICH9_IGP_C               0x294C
#define E1000_DEV_ID_ICH9_IFE                 0x10C0
#define E1000_DEV_ID_ICH9_IFE_GT              0x10C3
#define E1000_DEV_ID_ICH9_IFE_G               0x10C2
#define E1000_DEV_ID_ICH10_R_BM_LM            0x10CC
#define E1000_DEV_ID_ICH10_R_BM_LF            0x10CD
#define E1000_DEV_ID_ICH10_R_BM_V             0x10CE
#define E1000_DEV_ID_PCH_M_HV_LM              0x10EA
#define E1000_DEV_ID_PCH_M_HV_LC              0x10EB
#define E1000_DEV_ID_PCH_D_HV_DM              0x10EF
#define E1000_DEV_ID_PCH_D_HV_DC              0x10F0
#define E1000_DEV_ID_PCH_LV_LM                0x1506
#define E1000_DEV_ID_PCH_LV_V                 0x1519
#define E1000_DEV_ID_PCH2_LV_LM               0x1502
#define E1000_DEV_ID_PCH2_LV_V                0x1503
#endif /* NO_ICH8LAN_SUPPORT */
#ifndef NO_82575_SUPPORT
#ifndef NO_82576_SUPPORT
#define E1000_DEV_ID_82576                    0x10C9
#define E1000_DEV_ID_82576_FIBER              0x10E6
#define E1000_DEV_ID_82576_SERDES             0x10E7
#define E1000_DEV_ID_82576_QUAD_COPPER        0x10E8
#define E1000_DEV_ID_82576_QUAD_COPPER_ET2    0x1526
#define E1000_DEV_ID_82576_NS                 0x150A
#define E1000_DEV_ID_82576_NS_SERDES          0x1518
#define E1000_DEV_ID_82576_SERDES_QUAD        0x150D
#endif /* NO_82576_SUPPORT */
#define E1000_DEV_ID_82575EB_COPPER           0x10A7
#define E1000_DEV_ID_82575EB_FIBER_SERDES     0x10A9
#define E1000_DEV_ID_82575GB_QUAD_COPPER      0x10D6
#endif /* NO_82575_SUPPORT */
#ifndef NO_82580_SUPPORT
#define E1000_DEV_ID_82580_COPPER             0x150E
#define E1000_DEV_ID_82580_FIBER              0x150F
#define E1000_DEV_ID_82580_SERDES             0x1510
#define E1000_DEV_ID_82580_SGMII              0x1511
#define E1000_DEV_ID_82580_COPPER_DUAL        0x1516
#define E1000_DEV_ID_82580_QUAD_FIBER         0x1527
#endif /* NO_82580_SUPPORT */
#define E1000_REVISION_0 0
#define E1000_REVISION_1 1
#define E1000_REVISION_2 2
#define E1000_REVISION_3 3
#define E1000_REVISION_4 4

#define E1000_FUNC_0     0
#define E1000_FUNC_1     1
#ifndef NO_82580_SUPPORT
#define E1000_FUNC_2     2
#define E1000_FUNC_3     3
#endif /* Barton Hills HW */

#define E1000_ALT_MAC_ADDRESS_OFFSET_LAN0   0
#define E1000_ALT_MAC_ADDRESS_OFFSET_LAN1   3
#ifndef NO_82580_SUPPORT
#define E1000_ALT_MAC_ADDRESS_OFFSET_LAN2   6
#define E1000_ALT_MAC_ADDRESS_OFFSET_LAN3   9
#endif /* Barton Hills HW */

enum e1000_mac_type {
	e1000_undefined = 0,
#ifndef NO_82571_SUPPORT
	e1000_82571,
	e1000_82572,
	e1000_82573,
#ifndef NO_82574_SUPPORT
	e1000_82574,
	e1000_82583,
#endif /* NO_82574_SUPPORT */
#endif
#ifndef NO_80003ES2LAN_SUPPORT
	e1000_80003es2lan,
#endif
#ifndef NO_ICH8LAN_SUPPORT
	e1000_ich8lan,
	e1000_ich9lan,
	e1000_pchlan,
	e1000_pch2lan,
#endif
#ifndef NO_82575_SUPPORT
	e1000_82575,
#ifndef NO_82576_SUPPORT
	e1000_82576,
#ifndef NO_82580_SUPPORT
	e1000_82580,
#endif
#endif /* NO_82576_SUPPORT */
#endif /* NO_82575_SUPPORT */
	e1000_num_macs  /* List is 1-based, so subtract 1 for TRUE count. */
};

enum e1000_media_type {
	e1000_media_type_unknown = 0,
	e1000_media_type_copper = 1,
	e1000_media_type_fiber = 2,
	e1000_media_type_internal_serdes = 3,
	e1000_num_media_types
};

enum e1000_nvm_type {
	e1000_nvm_unknown = 0,
	e1000_nvm_none,
	e1000_nvm_eeprom_spi,
#ifndef NO_MICROWIRE_SUPPORT
	e1000_nvm_eeprom_microwire,
#endif
	e1000_nvm_flash_hw,
	e1000_nvm_flash_sw
};

enum e1000_nvm_override {
	e1000_nvm_override_none = 0,
	e1000_nvm_override_spi_small,
	e1000_nvm_override_spi_large,
#ifndef NO_MICROWIRE_SUPPORT
	e1000_nvm_override_microwire_small,
	e1000_nvm_override_microwire_large
#endif
};

enum e1000_phy_type {
	e1000_phy_unknown = 0,
	e1000_phy_none,
	e1000_phy_m88,
	e1000_phy_igp,
	e1000_phy_igp_2,
	e1000_phy_gg82563,
	e1000_phy_igp_3,
	e1000_phy_ife,
	e1000_phy_bm,
	e1000_phy_82578,
	e1000_phy_82577,
	e1000_phy_82579,
#ifndef NO_82580_SUPPORT
	e1000_phy_82580,
#endif /* NO_82580_SUPPORT */
#ifndef NO_82576_SUPPORT
	e1000_phy_vf,
#endif
};

enum e1000_bus_type {
	e1000_bus_type_unknown = 0,
	e1000_bus_type_pci,
	e1000_bus_type_pcix,
	e1000_bus_type_pci_express,
	e1000_bus_type_reserved
};

enum e1000_bus_speed {
	e1000_bus_speed_unknown = 0,
	e1000_bus_speed_33,
	e1000_bus_speed_66,
	e1000_bus_speed_100,
	e1000_bus_speed_120,
	e1000_bus_speed_133,
	e1000_bus_speed_2500,
	e1000_bus_speed_5000,
	e1000_bus_speed_reserved
};

enum e1000_bus_width {
	e1000_bus_width_unknown = 0,
	e1000_bus_width_pcie_x1,
	e1000_bus_width_pcie_x2,
	e1000_bus_width_pcie_x4 = 4,
	e1000_bus_width_pcie_x8 = 8,
	e1000_bus_width_32,
	e1000_bus_width_64,
	e1000_bus_width_reserved
};

enum e1000_1000t_rx_status {
	e1000_1000t_rx_status_not_ok = 0,
	e1000_1000t_rx_status_ok,
	e1000_1000t_rx_status_undefined = 0xFF
};

enum e1000_rev_polarity {
	e1000_rev_polarity_normal = 0,
	e1000_rev_polarity_reversed,
	e1000_rev_polarity_undefined = 0xFF
};

enum e1000_fc_mode {
	e1000_fc_none = 0,
	e1000_fc_rx_pause,
	e1000_fc_tx_pause,
	e1000_fc_full,
	e1000_fc_default = 0xFF
};

enum e1000_ms_type {
	e1000_ms_hw_default = 0,
	e1000_ms_force_master,
	e1000_ms_force_slave,
	e1000_ms_auto
};

enum e1000_smart_speed {
	e1000_smart_speed_default = 0,
	e1000_smart_speed_on,
	e1000_smart_speed_off
};

enum e1000_serdes_link_state {
	e1000_serdes_link_down = 0,
	e1000_serdes_link_autoneg_progress,
	e1000_serdes_link_autoneg_complete,
	e1000_serdes_link_forced_up
};

/* Receive Descriptor */
struct e1000_rx_desc {
	UINT64 buffer_addr; /* Address of the descriptor's data buffer */
	UINT16 length;      /* Length of data DMAed into data buffer */
	UINT16 csum;        /* Packet checksum */
	UINT8  status;         /* Descriptor status */
	UINT8  errors;         /* Descriptor Errors */
	UINT16 special;
};

/* Receive Descriptor - Extended */
union e1000_rx_desc_extended {
	struct {
		UINT64 buffer_addr;
		UINT64 reserved;
	} read;
	struct {
		struct {
			UINT32 mrq;           /* Multiple Rx Queues */
			union {
				UINT32 rss;         /* RSS Hash */
				struct {
					UINT16 ip_id;  /* IP id */
					UINT16 csum;   /* Packet Checksum */
				} csum_ip;
			} hi_dword;
		} lower;
		struct {
			UINT32 status_error;  /* ext status/error */
			UINT16 length;
			UINT16 vlan;          /* VLAN tag */
		} upper;
	} wb;  /* writeback */
};

#define MAX_PS_BUFFERS 4
/* Receive Descriptor - Packet Split */
union e1000_rx_desc_packet_split {
	struct {
		/* one buffer for protocol header(s), three data buffers */
		UINT64 buffer_addr[MAX_PS_BUFFERS];
	} read;
	struct {
		struct {
			UINT32 mrq;           /* Multiple Rx Queues */
			union {
				UINT32 rss;           /* RSS Hash */
				struct {
					UINT16 ip_id;    /* IP id */
					UINT16 csum;     /* Packet Checksum */
				} csum_ip;
			} hi_dword;
		} lower;
		struct {
			UINT32 status_error;  /* ext status/error */
			UINT16 length0;       /* length of buffer 0 */
			UINT16 vlan;          /* VLAN tag */
		} middle;
		struct {
			UINT16 header_status;
			UINT16 length[3];     /* length of buffers 1-3 */
		} upper;
		UINT64 reserved;
	} wb; /* writeback */
};

/* Transmit Descriptor */
struct e1000_tx_desc {
	UINT64 buffer_addr;   /* Address of the descriptor's data buffer */
	union {
		UINT32 data;
		struct {
			UINT16 length;    /* Data buffer length */
			UINT8 cso;           /* Checksum offset */
			UINT8 cmd;           /* Descriptor control */
		} flags;
	} lower;
	union {
		UINT32 data;
		struct {
			UINT8 status;        /* Descriptor status */
			UINT8 css;           /* Checksum start */
			UINT16 special;
		} fields;
	} upper;
};

/* Offload Context Descriptor */
struct e1000_context_desc {
	union {
		UINT32 ip_config;
		struct {
			UINT8 ipcss;         /* IP checksum start */
			UINT8 ipcso;         /* IP checksum offset */
			UINT16 ipcse;     /* IP checksum end */
		} ip_fields;
	} lower_setup;
	union {
		UINT32 tcp_config;
		struct {
			UINT8 tucss;         /* TCP checksum start */
			UINT8 tucso;         /* TCP checksum offset */
			UINT16 tucse;     /* TCP checksum end */
		} tcp_fields;
	} upper_setup;
	UINT32 cmd_and_length;
	union {
		UINT32 data;
		struct {
			UINT8 status;        /* Descriptor status */
			UINT8 hdr_len;       /* Header length */
			UINT16 mss;       /* Maximum segment size */
		} fields;
	} tcp_seg_setup;
};

/* Offload data descriptor */
struct e1000_data_desc {
	UINT64 buffer_addr;   /* Address of the descriptor's buffer address */
	union {
		UINT32 data;
		struct {
			UINT16 length;    /* Data buffer length */
			UINT8 typ_len_ext;
			UINT8 cmd;
		} flags;
	} lower;
	union {
		UINT32 data;
		struct {
			UINT8 status;        /* Descriptor status */
			UINT8 popts;         /* Packet Options */
			UINT16 special;
		} fields;
	} upper;
};

/* Statistics counters collected by the MAC */
struct e1000_hw_stats {
	UINT64 crcerrs;
	UINT64 algnerrc;
	UINT64 symerrs;
	UINT64 rxerrc;
	UINT64 mpc;
	UINT64 scc;
	UINT64 ecol;
	UINT64 mcc;
	UINT64 latecol;
	UINT64 colc;
	UINT64 dc;
	UINT64 tncrs;
	UINT64 sec;
	UINT64 cexterr;
	UINT64 rlec;
	UINT64 xonrxc;
	UINT64 xontxc;
	UINT64 xoffrxc;
	UINT64 xofftxc;
	UINT64 fcruc;
	UINT64 prc64;
	UINT64 prc127;
	UINT64 prc255;
	UINT64 prc511;
	UINT64 prc1023;
	UINT64 prc1522;
	UINT64 gprc;
	UINT64 bprc;
	UINT64 mprc;
	UINT64 gptc;
	UINT64 gorc;
	UINT64 gotc;
	UINT64 rnbc;
	UINT64 ruc;
	UINT64 rfc;
	UINT64 roc;
	UINT64 rjc;
	UINT64 mgprc;
	UINT64 mgpdc;
	UINT64 mgptc;
	UINT64 tor;
	UINT64 tot;
	UINT64 tpr;
	UINT64 tpt;
	UINT64 ptc64;
	UINT64 ptc127;
	UINT64 ptc255;
	UINT64 ptc511;
	UINT64 ptc1023;
	UINT64 ptc1522;
	UINT64 mptc;
	UINT64 bptc;
	UINT64 tsctc;
	UINT64 tsctfc;
	UINT64 iac;
	UINT64 icrxptc;
	UINT64 icrxatc;
	UINT64 ictxptc;
	UINT64 ictxatc;
	UINT64 ictxqec;
	UINT64 ictxqmtc;
	UINT64 icrxdmtc;
	UINT64 icrxoc;
	UINT64 cbtmpc;
	UINT64 htdpmc;
	UINT64 cbrdpc;
	UINT64 cbrmpc;
	UINT64 rpthc;
	UINT64 hgptc;
	UINT64 htcbdpc;
	UINT64 hgorc;
	UINT64 hgotc;
	UINT64 lenerrs;
	UINT64 scvpc;
	UINT64 hrmpc;
	UINT64 doosync;
};

#ifndef NO_82576_SUPPORT
#endif

struct e1000_phy_stats {
	UINT32 idle_errors;
	UINT32 receive_errors;
};

struct e1000_host_mng_dhcp_cookie {
	UINT32 signature;
	UINT8  status;
	UINT8  reserved0;
	UINT16 vlan_id;
	UINT32 reserved1;
	UINT16 reserved2;
	UINT8  reserved3;
	UINT8  checksum;
};

/* Host Interface "Rev 1" */
struct e1000_host_command_header {
	UINT8 command_id;
	UINT8 command_length;
	UINT8 command_options;
	UINT8 checksum;
};

#define E1000_HI_MAX_DATA_LENGTH     252
struct e1000_host_command_info {
	struct e1000_host_command_header command_header;
	UINT8 command_data[E1000_HI_MAX_DATA_LENGTH];
};

/* Host Interface "Rev 2" */
struct e1000_host_mng_command_header {
	UINT8  command_id;
	UINT8  checksum;
	UINT16 reserved1;
	UINT16 reserved2;
	UINT16 command_length;
};

#define E1000_HI_MAX_MNG_DATA_LENGTH 0x6F8
struct e1000_host_mng_command_info {
	struct e1000_host_mng_command_header command_header;
	UINT8 command_data[E1000_HI_MAX_MNG_DATA_LENGTH];
};

#include "e1000_mac.h"
#include "e1000_phy.h"
#include "e1000_nvm.h"
#include "e1000_manage.h"

struct e1000_mac_operations {
	/* Function pointers for the MAC. */
	INT32  (*init_params)(struct e1000_hw *);
	INT32  (*id_led_init)(struct e1000_hw *);
	INT32  (*blink_led)(struct e1000_hw *);
	INT32  (*check_for_link)(struct e1000_hw *);
	BOOLEAN (*check_mng_mode)(struct e1000_hw *hw);
	INT32  (*cleanup_led)(struct e1000_hw *);
	void (*clear_hw_cntrs)(struct e1000_hw *);
	void (*clear_vfta)(struct e1000_hw *);
	INT32  (*get_bus_info)(struct e1000_hw *);
	void (*set_lan_id)(struct e1000_hw *);
	INT32  (*get_link_up_info)(struct e1000_hw *, UINT16 *, UINT16 *);
	INT32  (*led_on)(struct e1000_hw *);
	INT32  (*led_off)(struct e1000_hw *);
	void (*update_mc_addr_list)(struct e1000_hw *, UINT8 *, UINT32);
	INT32  (*reset_hw)(struct e1000_hw *);
	INT32  (*init_hw)(struct e1000_hw *);
#ifndef NO_82576_SUPPORT
	void (*shutdown_serdes)(struct e1000_hw *);
	void (*power_up_serdes)(struct e1000_hw *);
#endif /* NO_82576_SUPPORT */
	INT32  (*setup_link)(struct e1000_hw *);
	INT32  (*setup_physical_interface)(struct e1000_hw *);
	INT32  (*setup_led)(struct e1000_hw *);
	void (*write_vfta)(struct e1000_hw *, UINT32, UINT32);
	void (*config_collision_dist)(struct e1000_hw *);
	void (*rar_set)(struct e1000_hw *, UINT8*, UINT32);
	INT32  (*read_mac_addr)(struct e1000_hw *);
	INT32  (*validate_mdi_setting)(struct e1000_hw *);
	INT32  (*mng_host_if_write)(struct e1000_hw *, UINT8*, UINT16, UINT16, UINT8*);
	INT32  (*mng_write_cmd_header)(struct e1000_hw *hw,
                      struct e1000_host_mng_command_header*);
	INT32  (*mng_enable_host_if)(struct e1000_hw *);
	INT32  (*wait_autoneg)(struct e1000_hw *);
};

struct e1000_phy_operations {
	INT32  (*init_params)(struct e1000_hw *);
	INT32  (*acquire)(struct e1000_hw *);
#ifndef NO_80003ES2LAN_SUPPORT
	INT32  (*cfg_on_link_up)(struct e1000_hw *);
#endif
	INT32  (*check_polarity)(struct e1000_hw *);
	INT32  (*check_reset_block)(struct e1000_hw *);
	INT32  (*commit)(struct e1000_hw *);
	INT32  (*force_speed_duplex)(struct e1000_hw *);
	INT32  (*get_cfg_done)(struct e1000_hw *hw);
	INT32  (*get_cable_length)(struct e1000_hw *);
	INT32  (*get_info)(struct e1000_hw *);
	INT32  (*read_reg)(struct e1000_hw *, UINT32, UINT16 *);
	INT32  (*read_reg_locked)(struct e1000_hw *, UINT32, UINT16 *);
	void (*release)(struct e1000_hw *);
	INT32  (*reset)(struct e1000_hw *);
	INT32  (*set_d0_lplu_state)(struct e1000_hw *, BOOLEAN);
	INT32  (*set_d3_lplu_state)(struct e1000_hw *, BOOLEAN);
	INT32  (*write_reg)(struct e1000_hw *, UINT32, UINT16);
	INT32  (*write_reg_locked)(struct e1000_hw *, UINT32, UINT16);
	void (*power_up)(struct e1000_hw *);
	void (*power_down)(struct e1000_hw *);
};

struct e1000_nvm_operations {
	INT32  (*init_params)(struct e1000_hw *);
	INT32  (*acquire)(struct e1000_hw *);
	INT32  (*read)(struct e1000_hw *, UINT16, UINT16, UINT16 *);
	void (*release)(struct e1000_hw *);
	void (*reload)(struct e1000_hw *);
	INT32  (*update)(struct e1000_hw *);
	INT32  (*valid_led_default)(struct e1000_hw *, UINT16 *);
	INT32  (*validate)(struct e1000_hw *);
	INT32  (*write)(struct e1000_hw *, UINT16, UINT16, UINT16 *);
};

struct e1000_mac_info {
	struct e1000_mac_operations ops;
	UINT8 addr[6];
	UINT8 perm_addr[6];

	enum e1000_mac_type type;

	UINT32 collision_delta;
	UINT32 ledctl_default;
	UINT32 ledctl_mode1;
	UINT32 ledctl_mode2;
	UINT32 mc_filter_type;
	UINT32 tx_packet_delta;
	UINT32 txcw;

	UINT16 current_ifs_val;
	UINT16 ifs_max_val;
	UINT16 ifs_min_val;
	UINT16 ifs_ratio;
	UINT16 ifs_step_size;
	UINT16 mta_reg_count;
#ifndef NO_82576_SUPPORT
	UINT16 uta_reg_count;
#endif

	/* Maximum size of the MTA register table in all supported adapters */
	#define MAX_MTA_REG 128
	UINT32 mta_shadow[MAX_MTA_REG];
	UINT16 rar_entry_count;

	UINT8  forced_speed_duplex;

	BOOLEAN adaptive_ifs;
	BOOLEAN has_fwsm;
	BOOLEAN arc_subsystem_valid;
#ifndef NO_ASF_FIRMWARE_CHECK
	BOOLEAN asf_firmware_present;
#endif
	BOOLEAN autoneg;
	BOOLEAN autoneg_failed;
	BOOLEAN get_link_status;
	BOOLEAN in_ifs_mode;
	enum e1000_serdes_link_state serdes_link_state;
	BOOLEAN serdes_has_link;
	BOOLEAN tx_pkt_filtering;
};

struct e1000_phy_info {
	struct e1000_phy_operations ops;
	enum e1000_phy_type type;

	enum e1000_1000t_rx_status local_rx;
	enum e1000_1000t_rx_status remote_rx;
	enum e1000_ms_type ms_type;
	enum e1000_ms_type original_ms_type;
	enum e1000_rev_polarity cable_polarity;
	enum e1000_smart_speed smart_speed;

	UINT32 addr;
	UINT32 id;
	UINT32 reset_delay_us; /* in usec */
	UINT32 revision;

	enum e1000_media_type media_type;

	UINT16 autoneg_advertised;
	UINT16 autoneg_mask;
	UINT16 cable_length;
	UINT16 max_cable_length;
	UINT16 min_cable_length;

	UINT8 mdix;

	BOOLEAN disable_polarity_correction;
	BOOLEAN is_mdix;
	BOOLEAN polarity_correction;
#ifndef NO_PHY_RESET_DISABLE
	BOOLEAN reset_disable;
#endif
	BOOLEAN speed_downgraded;
	BOOLEAN autoneg_wait_to_complete;
};

struct e1000_nvm_info {
	struct e1000_nvm_operations ops;
	enum e1000_nvm_type type;
	enum e1000_nvm_override override;

	UINT32 flash_bank_size;
	UINT32 flash_base_addr;

	UINT16 word_size;
	UINT16 delay_usec;
	UINT16 address_bits;
	UINT16 opcode_bits;
	UINT16 page_size;
};

struct e1000_bus_info {
	enum e1000_bus_type type;
	enum e1000_bus_speed speed;
	enum e1000_bus_width width;

	UINT16 func;
	UINT16 pci_cmd_word;
};

struct e1000_fc_info {
	UINT32 high_water;          /* Flow control high-water mark */
	UINT32 low_water;           /* Flow control low-water mark */
	UINT16 pause_time;          /* Flow control pause timer */
	UINT16 refresh_time;        /* Flow control refresh timer */
	BOOLEAN send_xon;           /* Flow control send XON */
	BOOLEAN strict_ieee;        /* Strict IEEE mode */
	enum e1000_fc_mode current_mode; /* FC mode in effect */
	enum e1000_fc_mode requested_mode; /* FC mode requested by caller */
};

#ifndef NO_82571_SUPPORT
struct e1000_dev_spec_82571 {
	BOOLEAN laa_is_present;
	UINT32 smb_counter;
	E1000_MUTEX swflag_mutex;
};

#endif /* NO_82571_SUPPORT */
#ifndef NO_80003ES2LAN_SUPPORT
struct e1000_dev_spec_80003es2lan {
	BOOLEAN  mdic_wa_enable;
};

#endif /* NO_80003ES2LAN_SUPPORT */
#ifndef NO_ICH8LAN_SUPPORT
struct e1000_shadow_ram {
	UINT16  value;
	BOOLEAN modified;
};

#define E1000_SHADOW_RAM_WORDS		2048

struct e1000_dev_spec_ich8lan {
	BOOLEAN kmrn_lock_loss_workaround_enabled;
	struct e1000_shadow_ram shadow_ram[E1000_SHADOW_RAM_WORDS];
	E1000_MUTEX nvm_mutex;
	E1000_MUTEX swflag_mutex;
	BOOLEAN nvm_k1_enabled;
	BOOLEAN eee_disable;
};

#endif /* NO_ICH8LAN_SUPPORT */
#ifndef NO_82575_SUPPORT
struct e1000_dev_spec_82575 {
	BOOLEAN sgmii_active;
	BOOLEAN global_device_reset;
};

#endif /* NO_82575_SUPPORT */
#ifndef NO_82576_SUPPORT
struct e1000_dev_spec_vf {
	UINT32	vf_number;
	UINT32	v2p_mailbox;
};

#endif /* NO_82576_SUPPORT */
struct e1000_hw {
	void *back;

	UINT8 *hw_addr;
	UINT8 *flash_address;
	unsigned long io_base;

	struct e1000_mac_info  mac;
	struct e1000_fc_info   fc;
	struct e1000_phy_info  phy;
	struct e1000_nvm_info  nvm;
	struct e1000_bus_info  bus;
	struct e1000_host_mng_dhcp_cookie mng_cookie;

	union {
#ifndef NO_82571_SUPPORT
		struct e1000_dev_spec_82571	_82571;
#endif
#ifndef NO_80003ES2LAN_SUPPORT
		struct e1000_dev_spec_80003es2lan _80003es2lan;
#endif
#ifndef NO_ICH8LAN_SUPPORT
		struct e1000_dev_spec_ich8lan	ich8lan;
#endif
#ifndef NO_82575_SUPPORT
		struct e1000_dev_spec_82575	_82575;
#endif
#ifndef NO_82576_SUPPORT
		struct e1000_dev_spec_vf	vf;
#endif
	} dev_spec;

	UINT16 device_id;
	UINT16 subsystem_vendor_id;
	UINT16 subsystem_device_id;
	UINT16 vendor_id;

	UINT8  revision_id;
};

#ifndef NO_82571_SUPPORT
#include "e1000_82571.h"
#endif
#ifndef NO_80003ES2LAN_SUPPORT
#include "e1000_80003es2lan.h"
#endif
#ifndef NO_ICH8LAN_SUPPORT
#include "e1000_ich8lan.h"
#endif
#ifndef NO_82575_SUPPORT
#include "e1000_82575.h"
#endif

/* These functions must be implemented by drivers */
INT32  e1000_read_pcie_cap_reg(struct e1000_hw *hw, UINT32 reg, UINT16 *value);
#ifndef NO_PCIE_SUPPORT
INT32  e1000_write_pcie_cap_reg(struct e1000_hw *hw, UINT32 reg, UINT16 *value);
#endif
#if !defined(NO_PCI_SUPPORT) || !defined(NO_PCIX_SUPPORT)
void e1000_read_pci_cfg(struct e1000_hw *hw, UINT32 reg, UINT16 *value);
#endif
#ifndef NO_PCIX_SUPPORT
void e1000_write_pci_cfg(struct e1000_hw *hw, UINT32 reg, UINT16 *value);
#endif

#endif
