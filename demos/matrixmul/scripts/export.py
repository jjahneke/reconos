import reconos.utils.shutil2 as shutil2
import reconos.utils.template as template

import logging
import argparse

import tempfile
import subprocess

log = logging.getLogger(__name__)

def get_cmd(prj):
	return "export_hw"

def get_call(prj):
	return export_hw_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("export_hw", description="""
		Exports the hardware project and generates all necessary files.
		""")
	parser.add_argument("-l", "--link", help="link sources instead of copying", action="store_true")
	parser.add_argument("-t", "--thread", help="export only single thread")
	parser.add_argument("hwdir", help="alternative export directory", nargs="?")
	return parser

def get_dict(prj):
	dictionary = {}
	dictionary["NUM_SLOTS"] = len(prj.slots)
	dictionary["NUM_CLOCKS"] = len(prj.clocks)
	dictionary["SYSCLK"] = prj.clock.id
	dictionary["SYSRST"] = "SYSRESET"
	dictionary["TOOL"]   = prj.impinfo.xil[0]
	dictionary["SLOTS"] = []
	for s in prj.slots:
		if s.threads:
			d = {}
			d["HwtCoreName"] = s.threads[0].get_corename()
			d["HwtCoreVersion"] = s.threads[0].get_coreversion()
			d["Id"] = s.id
			d["Clk"] = s.clock.id
			d["Async"] = "sync" if s.clock == prj.clock else "async"
			dictionary["SLOTS"].append(d)
	dictionary["CLOCKS"] = []
	for c in prj.clocks:
		d = {}
		d["Id"] = c.id
		d["ReqFreqKHz"] = c.freq / 1000
		param = c.get_pllparam(800000000, 1600000000, 100000000)
		d["ActFreqKHz"] = param[2] / 1000
		d["M"] = param[0]
		d["O"] = param[1]

		dictionary["CLOCKS"].append(d)
	
	return dictionary
	
def export_hw_cmd(args):
	if args.thread is None:
		export_hw(args, args.hwdir, args.link)
	else:
		export_hw_thread(args, args.hwdir, args.link, args.thread)

def export_hw(args, hwdir, link):
	if args.prj.impinfo.xil[0] == "ise":
		export_hw_ise(args, hwdir, link)
	elif args.prj.impinfo.xil[0] == "vivado":
		export_hw_vivado(args, hwdir, link)
	else:
		log.error("Xilinx tool not supported")

def export_hw_thread(args, hwdir, link, thread):
	if (args.prj.impinfo.xil[0] == "ise") or (args.prj.impinfo.xil[0] == "vivado"):
		export_hw_thread_ise_vivado(args, hwdir, link, thread)
	else:
		log.error("Xilinx tool not supported")

def export_hw_ise(args, hwdir, link):
	''' 
	Generates the project directory for an ISE/XPS project.
	
	It first compiles the configuration dictionary and then processes the templates
	according to the configuration dictionary.
	
	hwdir gives the name of the project directory
	link boolean; if true files will be linked instead of copied
	'''
	prj = args.prj
	hwdir = hwdir if hwdir is not None else prj.basedir + ".hw"

	log.info("Export hardware to directory '" + hwdir + "'")

	dictionary = get_dict(prj)

	log.info("Generating export files ...")
	tmpl = "ref_" + prj.impinfo.os + "_" + "_".join(prj.impinfo.board) + "_" + prj.impinfo.design + "_" + prj.impinfo.xil[1]
	prj.apply_template(tmpl, dictionary, hwdir, link)

	log.info("Generating threads ...")
	for t in prj.threads:
		export_hw_thread(args, shutil2.join(hwdir, "pcores"), link, t.name)

def export_hw_thread_ise_vivado(args, hwdir, link, thread):
	''' 
	Generates sources for one hardware thread for ReconOS in an ISE/XPS or Vivado project.
	
	It checks whether vhdl or hls sources shall be used and generates the hardware thread
	from the source templates. 
	
	hwdir gives the name of the project directory
	link boolean; if true files will be linked instead of copied
	thread is the name of the hardware thread to generate
	'''
	prj = args.prj
	hwdir = hwdir if hwdir is not None else prj.basedir + ".hw" + "." + thread.lower()

	log.info("Exporting thread " + thread + " to directory '" + hwdir + "'")

	threads = [_ for _ in prj.threads if _.name == thread]
	if (len(threads) == 1):
		thread = threads[0]

		if thread.hwsource is None:
			log.info("No hardware source specified")
	else:
		log.error("Thread '" + thread  + "' not found")
		return

	if thread.hwsource == "vhdl":
		dictionary = {}
		dictionary["NAME"] = thread.name.lower();
		srcs = shutil2.join(prj.dir, "src", "rt_" + thread.name.lower(), thread.hwsource)
		dictionary["SOURCES"] = [srcs]
		incls = shutil2.listfiles(srcs, True)
		dictionary["INCLUDES"] = [{"File": shutil2.trimext(_)} for _ in incls]
		dictionary["RESOURCES"] = []
		for i, r in enumerate(thread.resources):
			d = {}
			d["NameUpper"] = (r.group + "_" + r.name).upper()
			d["NameLower"] = (r.group + "_" + r.name).lower()
			d["LocalId"] = i
			d["HexLocalId"] =  "%08x" % i
			dictionary["RESOURCES"].append(d)

		log.info("Generating export files ...")
		prj.apply_template("thread_vhdl_pcore", dictionary, hwdir, link)

	elif thread.hwsource == "hls":
		tmp = tempfile.TemporaryDirectory()

		dictionary = {}
		dictionary["PART"] = prj.impinfo.part
		dictionary["NAME"] = thread.name.lower()
		dictionary["CLKPRD"] = min([_.clock.get_periodns() for _ in thread.slots])
		srcs = shutil2.join(prj.dir, "src", "rt_" + thread.name.lower(), thread.hwsource)
		dictionary["SOURCES"] = [srcs]
		files = shutil2.listfiles(srcs, True)
		dictionary["FILES"] = [{"File": _} for _ in files]
		dictionary["RESOURCES"] = []
		for i, r in enumerate(thread.resources):
			d = {}
			d["NameUpper"] = (r.group + "_" + r.name).upper()
			d["NameLower"] = (r.group + "_" + r.name).lower()
			d["LocalId"] = i
			d["HexLocalId"] =  "%08x" % i
			d["Type"] = r.type
			d["TypeUpper"] = r.type.upper()
			dictionary["RESOURCES"].append(d)

		log.info("Generating temporary HLS project in " + tmp.name + " ...")
		prj.apply_template("thread_hls_build", dictionary, tmp.name)

		log.info("Starting Vivado HLS ...")
		if "bbd" in thread.hwoptions:
			if "vivado" in thread.hwoptions:
				subprocess.call("""
				  source /opt/Xilinx/Vivado/{1}/settings64.sh;
				  cd {0};
				  vivado_hls -f script_csynth.tcl;
				  vivado -mode batch -notrace -nojournal -nolog -source script_vivado_edn.tcl;""".format(tmp.name, prj.impinfo.hls[1]),
				  shell=True)

				dictionary = {}
				dictionary["NAME"] = thread.name.lower()
				srcs = shutil2.join(tmp.name, "rt_imp.edn")
				dictionary["SOURCES"] = [srcs]
				incls = ["rt_imp.edn"]
				dictionary["INCLUDES"] = [{"File": _} for _ in incls]
			else:
				log.error("No bbd tool found")
				return

			log.info("Generating export files ...")
			prj.apply_template("thread_hls_pcore_bbd", dictionary, hwdir)

		else:
			subprocess.call("""
			  source /opt/Xilinx/Vivado/{1}/settings64.sh;
			  cd {0};
			  vivado_hls -f script_csynth.tcl;""".format(tmp.name, prj.impinfo.hls[1]),
			  shell=True)

			dictionary = {}
			dictionary["NAME"] = thread.name.lower()
			srcs = shutil2.join(tmp.name, "hls", "sol", "syn", "vhdl")
			dictionary["SOURCES"] = [srcs]
			incls = shutil2.listfiles(srcs, True)
			dictionary["INCLUDES"] = [{"File": shutil2.trimext(_)} for _ in incls]

			log.info("Generating export files ...")
			prj.apply_template("thread_hls_pcore_vhdl", dictionary, hwdir)

		shutil2.rmtree("/tmp/test")
		shutil2.mkdir("/tmp/test")
		shutil2.copytree(tmp.name, "/tmp/test")
		
def export_hw_vivado(args, hwdir, link):
	''' 
	Generates a TCL script for generation of a Vivado based project.
	
	It first compiles the configuration dictionary and then processes the templates
	according to the configuration dictionary.
	
	hwdir gives the name of the project directory
	link boolean; if true files will be linked instead of copied
	'''
	
	print("export_hw_vivado")
	prj = args.prj
	hwdir = hwdir if hwdir is not None else prj.basedir + ".hw"

	log.info("Export hardware to directory '" + hwdir + "'")

	dictionary = get_dict(prj)

	log.info("Generating export files ...")
	
	
	tmpl = "ref_" + prj.impinfo.os + "_" + "_".join(prj.impinfo.board) + "_" + prj.impinfo.design + "_" + prj.impinfo.xil[0] + "_" + prj.impinfo.xil[1]
	print("Using template directory " + tmpl)
	# TODO: No error message when template directory is not found!
	prj.apply_template(tmpl, dictionary, hwdir, link)

	log.info("Generating threads ...")
	for t in prj.threads:
		export_hw_thread(args, shutil2.join(hwdir, "pcores"), link, t.name)
		
	print("Calling TCL script to generate Vivado IP Repository")
	subprocess.call("""
					source /opt/Xilinx/SDSoC/{1}/settings64.sh;
					cd {0};
					vivado -mode batch -notrace -nojournal -nolog -source create_ip_library.tcl;""".format(hwdir, prj.impinfo.xil[1]),
					shell=True)
	
	print("Calling TCL script to generate ReconOS in Vivado IP Integrator")
	subprocess.call("""
					source /opt/Xilinx/SDSoC/{1}/settings64.sh;
					cd {0};
					vivado -mode batch -source system.tcl -tclargs -proj_name myReconOS -proj_path . -hwts rt_matrixmul,rt_matrixmul;""".format(hwdir, prj.impinfo.xil[1]),
					# vivado -mode batch -notrace -nojournal -nolog -source system.tcl -tclargs -proj_name myReconOS -proj_path . -hwts rt_matrixmul,rt_matrixmul;""".format(hwdir, prj.impinfo.xil[1]),
					shell=True)

