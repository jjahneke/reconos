SUMMARY = "Recipe for  build an external reconosmod64 Linux kernel module"
SECTION = "PETALINUX/modules"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

SRC_URI = "file://Makefile \
           file://reconos.c \
	   file://COPYING \
	   file://include/reconosio.h \
	   file://reconos.h \
	   file://proc_control.h \
	   file://proc_control.c \
	   file://osif_intc.h \
	   file://osif_intc.c \
          "

S = "${WORKDIR}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.
