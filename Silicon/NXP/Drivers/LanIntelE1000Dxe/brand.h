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
typedef struct {
    UINT16 vendor_id;
    UINT16 subvendor_id;
    UINT16 device_id;
    UINT16 subsystem_id;
    CHAR16 *brand_string;
} BRAND_STRUCT;

BRAND_STRUCT branding_table[] =
{
#ifndef NO_BRANDING_SUPPORT

#ifndef NO_82571_SUPPORT
    {0x8086, 0x0000, 0x105E, 0x0000, L"Intel(R) PRO/1000 PT Dual Port Network Connection"},
    {0x8086, 0x8086, 0x105E, 0x005E, L"Intel(R) PRO/1000 PT Dual Port Server Connection"},
    {0x8086, 0x8086, 0x105E, 0x115E, L"Intel(R) PRO/1000 PT Dual Port Server Adapter"},
    {0x8086, 0x8086, 0x105E, 0x125E, L"Intel(R) PRO/1000 PT Dual Port Server Adapter"},
    {0x8086, 0x8086, 0x105E, 0x135E, L"Intel(R) PRO/1000 PT Dual Port Server Adapter"},
    {0x8086, 0x103C, 0x105E, 0x704E, L"Intel(R) PRO/1000 PT Dual Port Server Adapter"},
    {0x8086, 0x103C, 0x105E, 0x7044, L"HP NC360T PCIe DP Gigabit Server Adapter"},
    {0x8086, 0x0000, 0x105F, 0x0000, L"Intel(R) PRO/1000 PF Dual Port Server Adapter"},
    {0x8086, 0x103C, 0x105F, 0x704F, L"Intel(R) PRO/1000 PF Dual Port Server Adapter"},
    {0x8086, 0x0000, 0x1060, 0x0000, L"Intel(R) PRO/1000 PB Dual Port Server Connection"},
    {0x8086, 0x0000, 0x10A4, 0x0000, L"Intel(R) PRO/1000 PT Quad Port Server Adapter"},
    {0x8086, 0x108E, 0x10D5, 0xF1BC, L"Intel(R) Gigabit PT Quad Port Server ExpressModule"},
    {0x8086, 0x0000, 0x10A5, 0x0000, L"Intel(R) PRO/1000 PF Quad Port Server Adapter"},
    {0x8086, 0x0000, 0x10BC, 0x0000, L"Intel(R) PRO/1000 PT Quad Port LP server Adapter"},
    {0x8086, 0x103C, 0x10BC, 0x704B, L"HP NC364T PCIe Quad Port Gigabit Server Adapter"},
    {0x8086, 0x8086, 0x107D, 0x1082, L"Intel(R) PRO/1000 PT Server Adapter"},
    {0x8086, 0x8086, 0x107D, 0x1092, L"Intel(R) PRO/1000 PT Server Adapter"},
    {0x8086, 0x8086, 0x107D, 0x1084, L"Intel(R) PRO/1000 PT Server Adapter"},
    {0x8086, 0x0000, 0x107D, 0x0000, L"Intel(R) PRO/1000 PT Network Connection"},
    {0x8086, 0x0000, 0x107E, 0x0000, L"Intel(R) PRO/1000 PF Network Connection"},
    {0x8086, 0x8086, 0x107E, 0x1084, L"Intel(R) PRO/1000 PF Server Adapter"},
    {0x8086, 0x8086, 0x107E, 0x1094, L"Intel(R) PRO/1000 PF Server Adapter"},
    {0x8086, 0x8086, 0x107E, 0x1085, L"Intel(R) PRO/1000 PF Server Adapter"},
    {0x8086, 0x0000, 0x107F, 0x0000, L"Intel(R) PRO/1000 PB Server Connection"},
    {0x8086, 0x0000, 0x10B9, 0x0000, L"Intel(R) PRO/1000 PT Desktop Adapter"},
    {0x8086, 0x103C, 0x10B9, 0x704A, L"HP NC110T PCIe Gigabit Server Adapter"},
    {0x8086, 0x0000, 0x108B, 0x0000, L"Intel(R) PRO/1000 PM Network Connection"},
    {0x8086, 0x0000, 0x108C, 0x0000, L"Intel(R) PRO/1000 PM Network Connection"},
    {0x8086, 0x0000, 0x109A, 0x0000, L"Intel(R) PRO/1000 PL Network Connection"},
#ifndef NO_82574_SUPPORT
    {0x8086, 0x0000, 0x10D3, 0x0000, L"Intel(R) 82574L Gigabit Network Connection"},
    {0x8086, 0x8086, 0x10D3, 0xA01F, L"Intel(R) Gigabit CT Desktop Adapter"},
    {0x8086, 0x8086, 0x10D3, 0x0001, L"Intel(R) Gigabit CT2 Desktop Adapter"},
    {0x8086, 0x103C, 0x10D3, 0x1785, L"HP NC112i 1-port Ethernet Server Adapter"},
    {0x8086, 0x0000, 0x10F6, 0x0000, L"Intel(R) 82574L Gigabit Network Connection"},
    {0x8086, 0x0000, 0x150C, 0x0000, L"Intel(R) 82583V Gigabit Network Connection"},
#endif
#endif /* NO_82571_SUPPORT */

#ifndef NO_80003ES2LAN_SUPPORT      
    {0x8086, 0x0000, 0x1096, 0x0000, L"Intel(R) PRO/1000 EB Network Connection "},
    {0x8086, 0x10F1, 0x1096, 0x2692, L"Intel(R) PRO/1000 EB1 Network Connection "},
    {0x8086, 0x1734, 0x1096, 0x10A8, L"Intel(R) PRO/1000 EB1 Network Connection "},
    {0x8086, 0x0000, 0x1098, 0x0000, L"Intel(R) PRO/1000 EB Backplane Connection "},
    {0x8086, 0x0000, 0x10BA, 0x0000, L"Intel(R) PRO/1000 EB1 Network Connection "},
    {0x8086, 0x0000, 0x10BB, 0x0000, L"Intel(R) PRO/1000 EB1 Backplane Connection "},
#endif /* NO_80003ES2LAN_SUPPORT */

#ifndef NO_ICH8LAN_SUPPORT
    {0x8086, 0x1179, 0x1049, 0x0001, L"82566MM Network Connection"},
    {0x8086, 0x0000, 0x1049, 0x0000, L"Intel(R) 82566MM Gigabit Network Connection"},
    {0x8086, 0x0000, 0x104A, 0x0000, L"Intel(R) 82566DM Gigabit Network Connection"},
    {0x8086, 0x17AA, 0x104A, 0x1012, L"Intel(R) 82566DM Gigabit Platform LAN Connect"},
    {0x8086, 0x17AA, 0x104A, 0x100F, L"Intel(R) 82566DM Gigabit Platform LAN Connect"},
    {0x8086, 0x0000, 0x104B, 0x0000, L"Intel(R) 82566DC Gigabit Network Connection"},
    {0x8086, 0x1179, 0x104C, 0x0001, L"82562V Network Connection"},
    {0x8086, 0x1179, 0x104D, 0x0001, L"82566MC Network Connection"},
    {0x8086, 0x0000, 0x104D, 0x0000, L"Intel(R) 82566MC Gigabit Network Connection"},
    {0x8086, 0x0000, 0x10BF, 0x0000, L"Intel(R) 82567LF Gigabit Network Connection"},
    {0x8086, 0x0000, 0x10F5, 0x0000, L"Intel(R) 82567LM Gigabit Network Connection"},
    {0x8086, 0x0000, 0x10CB, 0x0000, L"Intel(R) 82567V Gigabit Network Connection"},
    {0x8086, 0x0000, 0x10BD, 0x0000, L"Intel(R)82566DM-2 Gigabit Network Connection"},
    {0x8086, 0x0000, 0x10E5, 0x0000, L"Intel(R) 82567LM-4 Gigabit Network Connection"},
    {0x8086, 0x0000, 0x294C, 0x0000, L"Intel(R) 82566DC-2 Gigabit Network Connection"},
    {0x8086, 0x0000, 0x10CC, 0x0000, L"Intel(R) 82567LM-2 Gigabit Network Connection"},
    {0x8086, 0x0000, 0x10CD, 0x0000, L"Intel(R) 82567LF-2 Gigabit Network Connection"},
    {0x8086, 0x0000, 0x10CE, 0x0000, L"Intel(R) 82567V-2 Gigabit Network Connection"},
    {0x8086, 0x0000, 0x10EA, 0x0000, L"Intel(R) 82577LM Gigabit Network Connection"},
    {0x8086, 0x0000, 0x10EB, 0x0000, L"Intel(R) 82577LC Gigabit Network Connection"},
    {0x8086, 0x0000, 0x10EF, 0x0000, L"Intel(R) 82578DM Gigabit Network Connection"},
    {0x8086, 0x0000, 0x10F0, 0x0000, L"Intel(R) 82578DC Gigabit Network Connection"},
    {0x8086, 0x0000, 0x1502, 0x0000, L"Intel(R) 82579LM Gigabit Network Connection"},
    {0x8086, 0x0000, 0x1503, 0x0000, L"Intel(R) 82579V Gigabit Network Connection"},
#endif /* NO_ICH8LAN_SUPPORT */

#ifndef NO_82575_SUPPORT
#ifndef NO_82576_SUPPORT
    {0x8086, 0x0000, 0x10C9, 0x0000, L"Intel(R) 82576 Gigabit Dual Port Network Connection"},
    {0x8086, 0x8086, 0x10C9, 0x0000, L"Intel(R) Gigabit ET Dual Port Server Adapter"},
    {0x8086, 0x0000, 0x10E6, 0x0000, L"Intel(R) 82576 Gigabit Dual Port Network Connection"},
    {0x8086, 0x8086, 0x10E6, 0x0000, L"Intel(R) Gigabit EF Dual Port Server Adapter"},
    {0x8086, 0x0000, 0x10E7, 0x0000, L"Intel(R) 82576 Gigabit Dual Port Server Network Connection"},
    {0x8086, 0x8086, 0x10E8, 0xA02B, L"Intel(R) Gigabit ET Quad Port Server Adapter"},
    {0x8086, 0x8086, 0x10E8, 0xA02C, L"Intel(R) Gigabit ET Quad Port Server Adapter"},
    {0x8086, 0x8086, 0x1526, 0xA05C, L"Intel(R) Gigabit ET2 Quad Port Server Adapter"},
    {0x8086, 0x8086, 0x1526, 0xA06C, L"Intel(R) Gigabit ET2 Quad Port Server Adapter"},
    {0x8086, 0x0000, 0x150A, 0x0000, L"Intel(R) 82576NS Gigabit Ethernet Controller"},
    {0x8086, 0x0000, 0x1518, 0x0000, L"Intel(R) 82576NS SerDes Gigabit Ethernet Controller"},
    {0x8086, 0x0000, 0x150D, 0xA10C, L"Intel(R) Gigabit ET Quad Port Mezzanine Card"},
#endif /* NO_82576_SUPPORT */
    {0x8086, 0x0000, 0x10A7, 0x0000, L"Intel(R) 82575EB Gigabit Network Connection"},
    {0x8086, 0x0000, 0x10A9, 0x0000, L"Intel(R) 82575EB Gigabit Backplane Connection"},
    {0x8086, 0x0000, 0x10D6, 0x0000, L"Intel(R) Gigabit VT Quad Port Server Adapter"},
#endif /* NO_82575_SUPPORT */

    {0x8086, 0x0000, 0x150E, 0x0000, L"Intel(R) 82580 Gigabit Network Connection"},
    {0x8086, 0x103C, 0x150E, 0x1780, L"HP NC365T PCIe Quad Port Gigabit Server Adapter"},
    {0x8086, 0x0000, 0x150F, 0x0000, L"Intel(R) 82580 Gigabit Fiber Network Connection"},
    {0x8086, 0x0000, 0x1510, 0x0000, L"Intel(R) 82580 Gigabit Backplane Connection"},
    {0x8086, 0x0000, 0x1511, 0x0000, L"Intel(R) 82580 Gigabit SFP Connection"},
    {0x8086, 0x8086, 0x150E, 0x12A1, L"Intel(R) Ethernet Server Adapter I340-T4"},
    {0x8086, 0x8086, 0x150E, 0x12A2, L"Intel(R) Ethernet Server Adapter I340-T4"},
    {0x8086, 0x8086, 0x1516, 0x12B1, L"Intel(R) Ethernet Server Adapter I340-T2"},
    {0x8086, 0x8086, 0x1516, 0x12B2, L"Intel(R) Ethernet Server Adapter I340-T2"},
    {0x8086, 0x8086, 0x1527, 0x0001, L"Intel(R) Ethernet Server Adapter I340-F4"},
    {0x8086, 0x8086, 0x1527, 0x0002, L"Intel(R) Ethernet Server Adapter I340-F4"},


#else /* N0_BRANDING_SUPPORT */
    {0x8086, 0x8086, 0x0000, 0x0000, L"Intel(R) PRO/1000 Network Connection"},
#endif /* N0_BRANDING_SUPPORT */
    {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, L" "}
};

