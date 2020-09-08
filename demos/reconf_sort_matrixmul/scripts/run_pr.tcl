###############################################################
###  Minimum settings required to run PR flow:
###  1. Specify flow steps
###  2. Define target board
###  3. Identify source directories
###  4. Define static module
###  5. Define RPs, and their RM variants
###############################################################
####flow control (1 = run step , 0 = skip step)
set run.topSynth       0 ;#synthesize static
set run.rmSynth        1 ;#synthesize RM variants
set run.prImpl         1 ;#implement each static + RM configuration
set run.prVerify       1 ;#verify RMs are compatible with static
set run.writeBitstream 1 ;#generate full and partial bitstreams

###############################################################
### Define target demo board
### Valid values are kcu105, vcu108, kcu116 and vcu118
### Select one only
###############################################################
set xboard        "zcu102"

###############################################################
###  Run Settings
###############################################################
####Input Directories
#set srcDir     "./Sources"
set srcDir     "."

set prjDir     "$srcDir/prj"

#not used, paths in advanced_settings point directly to checkpoint and constraint files!
#set rtlDir     "$srcDir/hdl"
#set xdcDir     "$srcDir/xdc"
#set coreDir    "$srcDir/cores"
#set netlistDir "$srcDir/netlist"

####Output Directories
set synthDir  "./Synth"
set implDir   "./Implement"
set dcpDir    "./Checkpoint"
set bitDir    "./Bitstreams"

###############################################################
### Static Module Definition
###############################################################
set top "design_1_wrapper"

###############################################################
### RP & RM Definitions (Repeat for each RP)
### 1. Define Reconfigurable Partition (RP) name
### 2. Define corresponoding top level cell (added this to work with ReconOS-generated static design)
### 3. Associate Reconfigurable Modules (RMs) to the RP
###############################################################
set rp1 "design_1_slot_0_0"
set rp1_inst "design_1_i/slot_0"
set rm_variants($rp1) "sortdemo_hwt0 matrixmul_hwt0"
set rp2 "design_1_slot_1_0"
set rp2_inst "design_1_i/slot_1"
set rm_variants($rp2) "sortdemo_hwt1 matrixmul_hwt1"

########################################################################
### RM Configurations (Valid combinations of RM variants)
### 1. Define initial configuration: rm_config(initial)
### 2. Define additional configurations: rm_config(xyz)
########################################################################
set module1_variant1 "sortdemo_hwt0"
set module2_variant1 "sortdemo_hwt1"
set rm_config(initial)   "$rp1 $rp1_inst $module1_variant1 $rp2 $rp2_inst $module2_variant1"
set module1_variant2 "matrixmul_hwt0"
set module2_variant2 "matrixmul_hwt1"
set rm_config(reconfig1) "$rp1 $rp1_inst $module1_variant2 $rp2 $rp2_inst $module2_variant2"

########################################################################
### Task / flow portion
########################################################################
# Build the designs
source ./scripts/advanced_settings.tcl
source $tclDir/run.tcl

exit ;#uncomment if running in batch mode
