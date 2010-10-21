HOW TO BUILD KERNEL 2.6.32 FOR SHW-M130K

1. Visit http://www.codesourcery.com/, download and install Sourcery G++ Lite 2009q3-67 toolchain for ARM EABI.

2. Extract kernel source and move into the top directory.

3. Open 'Makefile' in the top directory and modify the value of following CROSS_COMPILE to a proper one regarding the path where toolchain is installed.
    CROSS_COMPILE ?= /opt/toolchains/arm-2009q3/bin/arm-none-linux-gnueabi-

4. Execute 'make aries_android_rfs_defconfig'.

5. Execute 'make' or 'make -j<n>' where '<n>' is the number of multiple jobs to be invoked simultaneously.

6. If the kernel is built successfully, you will find following files from the top directory:
	arch/arm/boot/zImage
	net/netfilter/xt_TCPMSS.ko
	drivers/onedram_svn/modemctl/modemctl.ko
	drivers/onedram_svn/onedram/onedram.ko
	drivers/onedram_svn/svnet/svnet.ko
	drivers/bluetooth/bthid/bthid.ko
	drivers/net/wireless/bcm4329/dhd.ko