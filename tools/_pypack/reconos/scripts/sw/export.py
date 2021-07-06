import re
import reconos.utils.shutil2 as shutil2
import reconos.utils.template as template

import logging
import argparse

log = logging.getLogger(__name__)

def get_cmd(prj):
	return "export_sw"

def get_call(prj):
	return export_sw_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("export_hw", description="""
		Exports the software project and generates all necessary files.
		""")
	parser.add_argument("-l", "--link", help="link sources instead of copying", default=False, action="store_true")
	parser.add_argument("-t", "--thread", help="export only single thread")
	parser.add_argument("swdir", help="alternative export directory", nargs="?")
	return parser

def export_sw_cmd(args):
	if (args.thread is None):
		export_sw(args, args.swdir, args.link)
	else:
		export_sw_thread(args, args.swdir, args.link, args.thread)

def export_sw(args, swdir, link):
	prj = args.prj
	swdir = swdir if swdir is not None else prj.basedir + ".sw"

	log.info("Export software to project directory '" + prj.dir + "'")
	
	dictionary = {}
	dictionary["NAME"] = prj.name.lower()
	dictionary["CFLAGS"] = prj.impinfo.cflags
	dictionary["LDFLAGS"] = prj.impinfo.ldflags
	dictionary["THREADS"] = []
	for t in prj.threads:
		d = {}
		d["Name"] = t.name.lower()
		d["Slots"] = ",".join([str(_.id) for _ in t.slots])
		d["SlotCount"] = len(t.slots)
		d["Resources"] = ",".join(["&" + (_.group + "_" + _.name).lower() + "_res" for _ in t.resources])
		d["ResourceCount"] = len(t.resources)
		d["HasHw"] = t.hwsource is not None
		d["HasSw"] = t.swsource is not None
		dictionary["THREADS"].append(d)
	dictionary["RESOURCES"] = []
	for r in prj.resources:
		d = {}
		d["Id"] = r.id
		d["NameUpper"] = (r.group + "_" + r.name).upper()
		d["NameLower"] = (r.group + "_" + r.name).lower()
		d["Type"] = r.type
		d["TypeUpper"] = r.type.upper()
		d["Args"] = ", ".join(r.args)
		d["Id"] = r.id
		dictionary["RESOURCES"].append(d)
	dictionary["CLOCKS"] = []
	for c in prj.clocks:
		d = {}
		d["NameLower"] = c.name.lower()
		d["Id"] = c.id
		param = c.get_pllparam(800000000, 1600000000, 100000000)
		d["M"] = param[0]
		d["O"] = param[1]
		dictionary["CLOCKS"].append(d)

	srcs = shutil2.join(prj.dir, "src", "application")
	dictionary["SOURCES"] = [srcs]

	log.info("Generating export files ...")
	templ = "app_" + prj.impinfo.os
	prj.apply_template(templ, dictionary, swdir, link)

	log.info("Generating threads ...")
	for t in prj.threads:
		export_sw_thread(args, swdir, link, t.name)

	list_files = shutil2.listfiles(swdir, True, "c[cp]*$")
	list_rcns = list(filter(re.compile("^lib/").match, list_files))
	list_app = list(filter(re.compile("^application/").match, list_files))

	dictionary = {}
	dictionary["OS"] = prj.impinfo.os.lower()
	dictionary["BOARD"] = "_".join(prj.impinfo.board)
	dictionary["REPO_REL"] = shutil2.relpath(prj.impinfo.repo, swdir)
	if prj.name.lower() != 'os2':
		dictionary["OBJS_RCNS"] = [{"Source": shutil2.trimext(_) + ".o"} for _ in list_files]
		dictionary["OBJS_APP"] = []
	else:
		dictionary["OBJS_RCNS"] = [{"Source": shutil2.trimext(_) + ".o"} for _ in list_rcns]
		dictionary["OBJS_APP"] = [{"Source": shutil2.trimext(_) + ".o"} for _ in list_app]

	template.preproc(shutil2.join(swdir, "Makefile"), dictionary, "overwrite", force=True)

def export_sw_thread(args, swdir, link, thread):
	prj = args.prj
	swdir = swdir if swdir is not None else prj.basedir + ".sw" + "." + thread.lower()

	log.info("Exporting thread " + thread + " to directory '" + swdir + "'")

	threads = [_ for _ in prj.threads if _.name == thread]
	if (len(threads) == 1):
		thread = threads[0]

		if thread.swsource is None:
			log.info("No software source specified")
			return
	else:
		log.info("Thread '" + thread  + "' not found")
		return

	dictionary = {}
	dictionary["NAME"] = thread.name.lower()
	dictionary["RESOURCES"] = []
	for i,r in enumerate(thread.resources):
		d = {}
		d["NameUpper"] = (r.group + "_" + r.name).upper()
		d["NameLower"] = (r.group + "_" + r.name).lower()
		d["Id"] = r.id
		d["HexId"] = "%08x" % r.id
		d["LocalId"] = i
		d["HexLocalId"] = "%08x" % i
		d["Type"] = r.type
		d["TypeUpper"] = r.type.upper()
		dictionary["RESOURCES"].append(d)
	dictionary["SOURCES"] = [shutil2.join(prj.dir, "src", "rt_" + thread.name.lower(), thread.swsource)]

	log.info("Generating export files ...")
	if thread.swsource == "c":
		prj.apply_template("thread_c_plain", dictionary, swdir, link)
	elif thread.swsource == "cpp":
		prj.apply_template("thread_cpp_plain", dictionary, swdir, link)
	elif thread.swsource == "hls":
		prj.apply_template("thread_c_hls", dictionary, swdir, link)
