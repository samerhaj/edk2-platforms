#  @file
#
#  Copyright 2018 NXP
#
#  This program and the accompanying materials
#  are licensed and made available under the terms and conditions of the BSD License
#  which accompanies this distribution. The full text of the license may be found at
#  http://opensource.org/licenses/bsd-license.php
#
#  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
#  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
#
#

################################################################################
#
# Pcd Section - list of all EDK II PCD Entries defined by this Chassis
#
################################################################################

[PcdsFixedAtBuild.common]
 #Stream IDs on NXP Chassis-3 (for example ls2080a, ls1088a, ls2088a)
 #devices are not hardwired and are programmed by sw. There are a limited
 #number of stream IDs available, and the partitioning of them is scenario
 #dependent. This header defines the partitioning between legacy,
 #PCI, and DPAA2 devices.
 #
 #This partitioning can be customized in this file depending
 #on the specific hardware config:
 #
 # -non-PCI legacy, platform devices (USB, SD/MMC, SATA, DMA)
 #    -all legacy devices get a unique stream ID assigned and programmed in
 #     their AMQR registers by u-boot
 #
 # -PCIe
 #    -there is a range of stream IDs set aside for PCI in this
 #     file.  U-boot will scan the PCI bus and for each device discovered:
 #        -allocate a streamID
 #        -set a PEXn LUT table entry mapping 'requester ID' to 'stream ID'
 #        -set a msi-map entry in the PEXn controller node in the
 #         device tree (see Documentation/devicetree/bindings/pci/pci-msi.txt
 #         for more info on the msi-map definition)
 #        -set a iommu-map entry in the PEXn controller node in the
 #         device tree (see Documentation/devicetree/bindings/pci/pci-iommu.txt
 #         for more info on the iommu-map definition)
 #
 # -DPAA2
 #    -u-boot will allocate a range of stream IDs to be used by the Management
 #     Complex for containers and will set these values in the MC DPC image.
 #    -the MC is responsible for allocating and setting up 'isolation context
 #     IDs (ICIDs) based on the allocated stream IDs for all DPAA2 devices.
 #
 #On Chasis-3 SoCs stream IDs are programmed in AMQ registers (32-bits) for
 #each of the different bus masters.  The relationship between
 #the AMQ registers and stream IDs is defined in the table below:
 #         AMQ bit    streamID bit
 #     ---------------------------
 #          PL[18]         9        // privilege bit
 #         BMT[17]         8        // bypass translation
 #          VA[16]         7        // reserved
 #            [15]         -        // unused
 #        ICID[14:7]       -        // unused
 #        ICID[6:0]        6-0      // isolation context id
 #    ----------------------------
 #
  # PCI - programmed in PEXn_LUT
  gNxpQoriqLsTokenSpaceGuid.PcdPcieStreamIdStart|7
  gNxpQoriqLsTokenSpaceGuid.PcdPcieStreamIdEnd|22
  # DPAA2 - set in MC DPC and alloced by MC
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2StreamIdStart|23
  gNxpQoriqLsTokenSpaceGuid.PcdDpaa2StreamIdEnd|63
