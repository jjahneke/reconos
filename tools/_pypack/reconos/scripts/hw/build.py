import reconos.utils.shutil2 as shutil2
import reconos.utils.template as template

import logging
import argparse
import subprocess

log = logging.getLogger(__name__)

def get_cmd(prj):
	return "build_hw"

def get_call(prj):
	return build_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("build_hw", description="""
		Builds the hardware project and generates a bitstream to
		be downloaded to the FPGA.
		""")
	parser.add_argument("hwdir", help="alternative export directory", nargs="?")
	return parser

def build_cmd(args):
	build(args.prj, args.hwdir)

def build(prj, hwdir):
	if prj.impinfo.xil[0] == "ise":
		_build_ise(prj, hwdir)
	elif prj.impinfo.xil[0] == "vivado":
		_build_vivado(prj, hwdir)
	else:
		log.error("Tool not supported")

def _build_ise(prj, hwdir):
	hwdir = hwdir if hwdir is not None else prj.basedir + ".hw"

	try:
		shutil2.chdir(hwdir)
	except:
		log.error("hardware directory '" + hwdir + "' not found")
		return
	
	subprocess.call("""
	  source /opt/Xilinx/""" + prj.impinfo.xil[1] + """/ISE_DS/settings64.sh;
	  cd """ + hwdir + """ &&
	  echo -e "run hwclean\nrun bits\nexit\n" | xps -nw system""",
	  shell=True, executable="/bin/bash")

	print()

	with open(shutil2.join(hwdir, "implementation", "system.bin"), "rb") as b:
		with open(shutil2.join(hwdir, "implementation", "system.bin.rev"), "wb") as r:
			while True:
				word = b.read(4)
				if not word:
					break
				r.write(word[::-1])

	shutil2.chdir(prj.dir)
	
def _build_vivado(prj, hwdir):
	hwdir = hwdir if hwdir is not None else prj.basedir + ".hw"

	try:
		shutil2.chdir(hwdir)
	except:
		log.error("hardware directory '" + hwdir + "' not found")
		return
	
	subprocess.call("""
					source /opt/Xilinx/Vivado/{0}/settings64.sh;
					vivado -mode batch -notrace -nojournal -nolog -source build.tcl;""".format(prj.impinfo.xil[1]),
					shell=True, executable="/bin/bash")
	print()

	shutil2.chdir(prj.dir)
