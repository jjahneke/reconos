#!/bin/bash

# This is the complete petalinux-package command 
# Includes the CCI enablement via APU configuration registers before boot
# This relies on a Bootgen init attribute and the file "regs.init"

# Run "petalinux-build" first

petalinux-package --boot --u-boot --force --bif-attribute init --bif-attribute-value regs.init
