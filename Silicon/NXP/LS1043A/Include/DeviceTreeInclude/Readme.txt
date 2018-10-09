These dts/dtsi files have been copied from https://bitbucket.sw.nxp.com/projects/DASH/repos/dash-linux/browse/arch/arm64/boot/dts/freescale

branch : next
Linux Version : 4.18.0-rc1

Following changes have been done in these files
1. Tab has been converted to two spaces.
2. The file end of line has been changed to Windows
3. #include <dt-bindings/interrupt-controller/arm-gic.h> and #include <dt-bindings/thermal/thermal.h>
   these header files have been removed and the values of macros defined in these files has been used directly
4. The include files' location has been changed.
   The common dts/dtsi files are in Silicon/NXP/Include/DeviceTreeInclude/
   SOC specific dts/dtsi files are in Silicon/NXP/<SOC>/Include/DeviceTreeInclude/
   Platform specific dts/dtsi files are in Platform/NXP/<Platform>/DeviceTree/

   where <SOC> can be LS1043A, LS1046A, LS2088A etc
   and <Platform> can be LS1043aRdbPkg, LS1046aRdbPkg, LS2088aRdbPkg etc

   Their path in dts/dtsi files have been fixed accordingly
5. The enable-method = "psci" has been added to all cpu nodes. no fixups are done by UEFI regarding this
6. The psci node has been added. no fixups are done by UEFI regarding this
7. Job ring 3 has been deleted from crypto node
8. "big-endian" property has been added to crypto, qman and bman nodes
9. The spi peripherals' maximum clock frequency has been increased to 50Mhz.
10. uefi-runtime property added to the qspi controller node and chip select 0 flash.
11. The devices that are controlled by serdes protocol like mac devices and pcie and sata device have been
    disabled using status = "disabled"; property in main SOC dtsi file and have been enabled in board dts
    file.
12. The interrupt-map in pcie nodes in dtsi file has been changed as per silicon rev 1.0
13. The changes required for Silicon revision 1.1 have been put in fsl-ls1043a-rdb-v1.1-overlay.dts file
