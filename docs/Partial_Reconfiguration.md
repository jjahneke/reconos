## Introduction
The following is a brief documentation of the new partial reconfiguration toolflow that succeeds the previous implementation of the "reconf_sort_matrixmul" demo (https://github.com/ReconOS/reconos/commit/1ad5b84993847fe78a41dc435811c49fd80253fa).

Due to Vivado limitations, the new flow is still unable to operate in project mode or interactively via the Vivado GUI. It is based on a newer version of the same non-project script-based toolflow as the previous implementation, but offers a deeper integration into the RDK. This eliminates almost all inconvenient manual design steps.

## Xilinx Resources
* XAPP1231 - Partial Reconfiguration of a Hardware Accelerator with Vivado Design Suite

  Original PR tutorial for the Zynq-7000, accompanied by a Xilinx Wiki page that is still being updated.

* UG909 - Vivado User Guide: Dynamic Function eXchange

  PR functionality was renamed to DFX without much change to the underlying scripts. This guide gives an overview.

* UG947 - Vivado Tutorial: Dynamic Function eXchange

  The actual scripts used by ReconOS are taken from lab 2 of this tutorial.

## Inner Workings
According to project settings (build.cfg), the RDK generates files from the following templates to prepare the PR toolflow:

* templates/ref_linux_zcu102_0_timer_vivado/pr_flow/run_pr.tcl

    Main script of the toolflow. Defines settings, reconfigurable partitions (RP), reconfigurable modules (RM) and configurations that shall be implemented. The rest of the scripts, namely "advanced_settings.tcl" and "Tcl_HD/*", are not touched by the RDK preprocessor.

* templates/ref_linux_zcu102_0_timer_vivado/pr_flow/pblocks.xdc

    Defines Pblocks for all slots.

* templates/thread_prj

    Generates a .prj file for each possible HWT/slot combination in "build.hw/pcores/prj/". These files define the relevant sourcefiles for the script-based toolflow. The static design is synthesized from the usual ReconOS block design (BD) and requires no .prj definition.

* templates/thread_rt_reconf

    This contains the sources of an empty dummy thread named "reconf". We need this for two reasons. Firstly, some module needs to be placed in the hardware slots for synthesis of the static design around them. We could use a black box for this purpose (as was done manually in the previous demo), but this is not easily automated and incorporated into the scripted toolflow. It is also not recommended by Xilinx, as the static design could not be optimized based on some exemplary reconfigurable logic inside the slots. While this dummy thread does nothing besides reacting to an exit signal, it is still better than a complete black box or grey box. Xilinx recommends to use the most resource-demanding or difficult to synthesize reconfigurable module for static design synthesis. Therefore, ReconOS users can overwrite this template dummy thread in the project directory (like any other template) as desired. One current limitation: There is only a single dummy thread with this exact name supported. It must fit into every slot.

    Secondly, there are some complications with the "link_design" command used during the toolflow, due to the way we handle the static design/black box situation. This results in unexpected behavior for the first configuration that is to be implemented. Instead of linking the respective HWT sources, the dummy module used for static synthesis seems to take priority and is inserted instead. Hence, we define a dummy configuration using the "reconf" thread first, so that all user-defined HWTs implement properly.

## Setting up PR for a ReconOS Project

### In Project Settings (build.cfg)

* Set `PartialReconfiguration = True`.
* Define pblock regions for each slot. These can easily be drawn in the Vivado GUI to retrieve the clock regions or tile coordinates, for example:
    * `CLOCKREGION_X2Y2:CLOCKREGION_X2Y4`
    * or `SLICE_X56Y0:SLICE_X94Y59 DSP48E2_X12Y0:DSP48E2_X17Y23 RAMB18_X7Y0:RAMB18_X12Y23 RAMB36_X7Y0:RAMB36_X12Y11`
* For groups of slots:
  * Define `Region_<ID>` for each ID in the defined range.
  * Make sure the IDs don't overlap, e.g. slotgroup with ID=0 and ID range (0:3) is followed by slot group with ID=4 and ID range (0:1).
* Define threads normally. VHDL and HLS are supported as hardware source and threads can also have software implementations.

### In HWTs
* For VHDL sources: Set top level entity name of all HWTs to `rt_reconf`.

### In Application
* Create, suspend, terminate HWTs normally.
* If the same resources are used by multiple threads, it is not required to terminate and re-create the appropriate delegate threads when a hardware slot is reconfigured. It is sufficient to create the dummy HWT "reconf" and suspend/resume it accordingly. This is also done in the current "reconf_sort_matrixmul" demo implementation.

## Advanced Usage

### Clocks
There is no problem with using multiple clocks for different groups of slots. However, keep in mind that the current ReconOS implementation uses 1 MMCM per clock, which are rather scarce for UltraScale+ devices (only 1 per clock management tile). In order to avoid unnecessary blockage of these resources, it is recommended to define pblock regions with more specific tile coordinates instead of clock region coordinates. In the future, it might be possible to change this ReconOS behavior, since a single MMCM can produce multiple clocks. PLL sites might also be used.
  
### Resource Groups
There is no problem with using different resource groups for each ReconOS thread. In this case, the corresponding delegate thread must be instantiated during runtime and "reconf" cannot be used.

Keep in mind that resource groups are associated with threads and not with slots. The OSIF's init data feature can be used to achieve communication with a specific instance of a thread that is running in multiple slots.
