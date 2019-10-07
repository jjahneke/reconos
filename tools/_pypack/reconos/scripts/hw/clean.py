import reconos.utils.shutil2 as shutil2

import logging
import argparse
import subprocess

log = logging.getLogger(__name__)

def get_cmd(prj):
	return "clean_hw"

def get_call(prj):
	return clean_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("clean_hw", description="""
		Cleans the hardware project.
		""")
	parser.add_argument("-r", "--remove", help="remove entire hardware directory", action="store_true")
	return parser

def clean_cmd(args):
	clean(args)
	
def clean(args):
	if args.prj.impinfo.xil[0] == "vivado":
		clean_vivado(args)
	else:
		log.error("Tool not supported")

def clean_vivado(args):
	prj = args.prj
	hwdir = prj.basedir + ".hw"

	if args.remove:
		shutil2.rmtree(hwdir)
	else:
		try:
			shutil2.chdir(hwdir)
		except:
			log.error("hardware directory '" + hwdir + "' not found")
			return
		
		subprocess.call("""
				source {1}/Vivado/{0}/settings64.sh;
				vivado -mode batch -notrace -nojournal -nolog -source clean.tcl;""".format(prj.impinfo.xil[1], prj.impinfo.xil_path),
				shell=True, executable="/bin/bash")

		print()
		shutil2.chdir(prj.dir)