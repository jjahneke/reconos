The PetaLinux project "RECONOS" was created with the following steps under v2020.1:

1. Create template project:
`petalinux-create --type project --template zynqMP --name RECONOS`

2. Initialize project with exemplary ReconOS hardware platform (Vivado-generated .xsa file):
`petalinux-config --get-hw-description <PATH-TO-XSA Directory>`

3. Configure system-level options:
`petalinux-config`
  * Set DTG->Machine_name to ZCU102 board
  * Enable kernel & U-Boot autoconfig
  * Manual kernel boot args: `earlycon clk_ignore_unused root=/dev/nfs nfsroot=<IP>:<NFS-PATH>,tcp,nfsvers=3 ip=dhcp rw`
  * Enable FPGA-Manager
  * Image packing: Switch to NFS with corresponding path & ip, disable tftpboot copy

4. Create new kernel module:
`petalinux-create -t modules --name reconosmod64 --enable`
  * Copy or link to driver sourcecode and makefile
  * Adjust recipe file (.bb) to include all files

5. Configure device-tree
  * ReconOS PL devices are automatically inferred and referenced in device-tree overlay (pl.dtbo) but we do not use these overlays
  * Adjust system-user.dtsi file in device-tree recipe to include ReconOS devices

6. Configure kernel:
`petalinux-config -c kernel`
  * Keep default settings

7. Configure rootfs:
`petalinux-config -c rootfs`
  * Keep default settings

8. Build:
`petalinux-build` or `petalinux-build -c reconosmod64` to build just the driver

9. Package:
`. ./petalinux_package.sh`
  * This script calls `petalinux-package` with special parameters to set required APU configuration registers

10. Deploy
  * Copy to SD card: BOOT.BIN, boot.scr, image.ub
  * Extract rootfs to NFS directory, avoid permission errors with `chmod 777 -R *` or similar

11. Cleaning:
`petalinux-build -f -x mrproper` or `petalinux-build -c reconosmod64 -x clean` to clean just the driver
