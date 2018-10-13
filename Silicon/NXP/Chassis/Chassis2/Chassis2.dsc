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
 # Stream IDs on Chassis-2 (for example ls1043a, ls1046a, ls1012) devices
 # are not hardwired and are programmed by sw.  There are a limited number
 # of stream IDs available, and the partitioning of them is scenario
 # dependent. This header defines the partitioning between legacy, PCI,
 # and DPAA1 devices.
 #
 # This partitioning can be customized in this file depending
 # on the specific hardware config:
 #
 #  -non-PCI legacy, platform devices (USB, SDHC, SATA, DMA, QE etc)
 #     -all legacy devices get a unique stream ID assigned and programmed in
 #      their AMQR registers by u-boot
 #
 #  -PCIe
 #     -there is a range of stream IDs set aside for PCI in this
 #      file.  U-boot will scan the PCI bus and for each device discovered:
 #         -allocate a streamID
 #         -set a PEXn LUT table entry mapping 'requester ID' to 'stream ID'
 #         -set a msi-map entry in the PEXn controller node in the
 #          device tree (see Documentation/devicetree/bindings/pci/pci-msi.txt
 #          for more info on the msi-map definition)
 #         -set a iommu-map entry in the PEXn controller node in the
 #          device tree (see Documentation/devicetree/bindings/pci/pci-iommu.txt
 #          for more info on the iommu-map definition)
 #
 #  -DPAA1
 #     - Stream ids for DPAA1 use are reserved for future usecase.
 #
  # PCI - programmed in PEXn_LUT
  gNxpQoriqLsTokenSpaceGuid.PcdPcieStreamIdStart|11
  gNxpQoriqLsTokenSpaceGuid.PcdPcieStreamIdEnd|26
