#                                                        ____  _____
#                            ________  _________  ____  / __ \/ ___/
#                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
#                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
#                         /_/   \___/\___/\____/_/ /_/\____//____/
# 
# ======================================================================
# 
#   project:      ReconOS - Toolchain
#   author:       Christoph Rüthing, University of Paderborn
#   description:  Representation of a ReconOS project.
# 
# ======================================================================

import reconos.utils.shutil2 as shutil2
import reconos.utils.template as template

import logging
import configparser
import re

log = logging.getLogger(__name__)

#
# Class representing a clock in the project.
#
class Clock:
	_id = 0

	def __init__(self, name, source, freq):
		self.id = Clock._id
		Clock._id += 1
		self.name = name
		self.source = source
		self.freq = freq

	def get_pllparam(self, vcomin, vcomax, infreq):
		dfmin = 1000000000
		for m in range(vcomin // infreq, vcomax // infreq + 1):
			for o in range(1, 101):
				if dfmin > abs(self.freq - infreq * m // o):
					dfmin = abs(self.freq - infreq * m // o)
					mopt = m
					oopt = o

		return (mopt, oopt, infreq * mopt // oopt)

	def get_periodns(self):
		return 1000000000 / self.freq

	def __str__(self):
		return "Clock '" + self.name + "'"

	def __repr__(self):
		return "'" + self.name + "' (" + str(self.id) + ")"

#
# Class representing a resource in the project.
#
class Resource:
	_id = 128

	def __init__(self, name, type_, mode, args, group):
		self.id = Resource._id
		Resource._id += 1
		self.name = name
		self.type = type_
		self.mode = mode
		self.args = args
		self.group = group

	def __str__(self):
		return "Resource '" + self.name + "'"

	def __repr__(self):
		return "'" + self.name + "' (" + str(self.id) + ")"

#
# Class representing a slot in the project.
#
class Slot:
	def __init__(self, name, id_, clock):
		self.name = name
		self.id = id_
		self.clock = clock
		self.threads = []

	def acc_resources(self):
		return set([r for t in self.threads for r in t.resources])

	def __str__(self):
		return "Slot '" + self.name + "' (" + str(self.id) + ")"

	def __repr__(self):
		return "'" + self.name + "' (" + str(self.id) + ")"

#
# Class representing a thread in the project.
#
class Thread:
	_id = 0

	def __init__(self, name, slots, hw, sw, res):
		self.id = Thread._id
		Thread._id += 1
		self.name = name
		self.slots = slots
		self.resources = res
		if hw is not None:
			hw = hw.split(",")
			self.hwsource = hw[0]
			self.hwoptions = hw[1:]
		else:
			self.hwsource = None
			self.hwoptions = []
		if sw is not None:
			sw = sw.split(",")
			self.swsource = sw[0]
			self.swoptions = sw[1:]
		else:
			self.swsource = None
			self.swoptions = []

	def get_corename(self):
		return "rt_" + self.name.lower()

	def get_coreversion(self):
		return "1.00.a"

	def get_swentry(self):
		if self.swsource is None or self.swsource == "":
			return "swt_idle"

		reg = r"(?P<name>[a-zA-Z0-9_]*)_(?P<vers>v[0-9]+_[0-9]{2}_[a-z])"
		return re.search(reg, self.swsource).group("name")

	def __str__(self):
		return "Thread '" + self.name + "' (" + str(self.id) + ")"

	def __repr__(self):
		return "'" + self.name + "' (" + str(self.id) + ")"

#
# Class representing implementation related information.
#
class ImpInfo:
	#
	# Initialization of a new ImpInfo
	#
	def __init__(self):
		self.repo = ""

		self.board = ""
		self.part = ""
		self.design = ""
		self.xil = ""

		self.os = ""
		self.cflags = ""
		self.ldflags = ""

#
# Class representing a project and providing different functionality
# for performing all relevant tasks.
#
class Project:

	#
	# Initialization of a new project.
	#
	def __init__(self, repo=None):
		self.clocks = []
		self.resources = []
		self.slots = []
		self.threads = []

		self.impinfo = ImpInfo()

		self.dir = ""
		self.name = ""
		self.clock = None

		if repo is not None and shutil2.isdir(repo):
			self.impinfo.repo = repo
		elif shutil2.environ("RECONOS"):
			self.impinfo.repo = shutil2.environ("RECONOS")
		else:
			log.error("ReconOS repository not found")

	def get_template(self, name):
		return shutil2.join(self.impinfo.repo, "templates", name)

	def apply_template(self, name, dictionary, output, link = False):
		shutil2.mkdir(output)
		shutil2.copytree(self.get_template(name), output, followlinks=True)
		template.generate(output, dictionary, "overwrite", link)


	def get_hwtref(self, thread):
		return shutil2.join(self.impinfo.repo, "templates", "export_hw", self.impinfo.xil[0], "thread_" + thread.hwsource)

	def get_swtref(self, thread):
		return shutil2.join(self.impinfo.repo, "templates", "export_sw", "thread_" + thread.swsource)


	#
	# Opens a project by parsing the project file.
	#
	#   filepath - path to the project file (*.cfg)
	#
	def open(self, filepath):
		self.clocks = []
		self.resources = []
		self.slots = []
		self.threads = []
		self.file = shutil2.abspath(filepath)
		self.dir = shutil2.dirname(self.file)
		self.basedir = shutil2.trimext(self.file)
		#self.hwdir = shutil2.trimext(self.file) + ".hw"
		#self.swdir = shutil2.trimext(self.file) + ".sw"

		cfg = configparser.RawConfigParser()
		cfg.optionxform = str
		ret = cfg.read(filepath)
		if not ret:
			log.error("Config file '" + filepath + "' not found")
			return

		self._parse_project(cfg)

	#
	# Internal method parsing the project from the project file.
	#
	#   cfg - configparser referencing the project file
	#
	def _parse_project(self, cfg):
		self.name = cfg.get("General", "Name")
		self.impinfo.board = re.split(r"[, ]+", cfg.get("General", "TargetBoard"))
		self.impinfo.part = cfg.get("General", "TargetPart")
		self.impinfo.design = cfg.get("General", "ReferenceDesign")
		self.impinfo.os = cfg.get("General", "TargetOS")
		self.impinfo.xil = cfg.get("General", "TargetXil").split(",")
		if cfg.has_option("General", "CFlags"):
			self.impinfo.cflags = cfg.get("General", "CFlags")
		else:
			self.impinfo.cflags = ""
		if cfg.has_option("General", "LdFlags"):
			self.impinfo.ldflags = cfg.get("General", "LdFlags")
		else:
			self.impinfo.ldflags = ""
		log.debug("Found project '" + str(self.name) + "' (" + str(self.impinfo.board) + "," + str(self.impinfo.os) + ")")

		self._parse_clocks(cfg)
		self._parse_resources(cfg)
		self._parse_slots(cfg)
		self._parse_threads(cfg)

		clock = [_ for _ in self.clocks if _.name == cfg.get("General", "SystemClock")]
		if not clock:
			log.error("Clock not found")
		self.clock = clock[0]

	#
	# Internal method parsing the clocks from the project file.
	#
	#   cfg - configparser referencing the project file
	#
	def _parse_clocks(self, cfg):
		for c in [_ for _ in cfg.sections() if _.startswith("Clock")]:
			match = re.search(r"^.*@(?P<name>.+)", c)
			if match is None:
				log.error("Clock must have a name")

			name = match.group("name")
			source = cfg.get(c, "ClockSource")
			freq = cfg.getint(c, "ClockFreq")

			log.debug("Found clock '" + str(name) + "' (" + str(source) + "," + str(freq / 1000000) + " MHz)")

			clock = Clock(name, source, freq)
			self.clocks.append(clock)

	#
	# Internal method parsing the resources from the project file.
	#
	#   cfg - configparser referencing the project file
	#
	def _parse_resources(self, cfg):
		for c in [_ for _ in cfg.sections() if _.startswith("ResourceGroup")]:
			match = re.search(r"^.*@(?P<name>.+)", c)
			if match is None:
				log.error("Resources must have a name")

			group = match.group("name")

			for r in cfg.options(c):
				match = re.split(r"[, ]+", cfg.get(c, r))

				log.debug("Found resource '" + str(r) + "' (" + str(match[0]) + "," + str(match[1:]) + "," + str(group) + ")")
			
				if "hw" in match:
					match.remove("hw")
					resource = Resource(r, match[0], "hw", match[1:], group)
				elif "sw" in match:
					match.remove("sw")
					resource = Resource(r, match[0], "sw", match[1:], group)
				else:
					resource = Resource(r, match[0], "sw", match[1:], group)
				self.resources.append(resource)

	#
	# Internal method parsing the slots from the project file.
	#
	#   cfg - configparser referencing the project file
	#
	def _parse_slots(self, cfg):
		for s in [_ for _ in cfg.sections() if _.startswith("HwSlot")]:
			match = re.search(r"^.*@(?P<name>.*)\((?P<start>[0-9]*):(?P<end>[0-9]*)\)", s)
			if match is None:
				r = range(0)
			else:
				r = range(int(match.group("start")), int(match.group("end")) + 1)

			name = match.group("name")
			id_ = cfg.getint(s, "Id")
			clock = [_ for _ in self.clocks if _.name == cfg.get(s, "Clock")]
			if not clock:
				log.error("Clock not found")

			for i in r:
				log.debug("Found slot '" + str(name) + "(" + str(i) + ")" + "' (" + str(id_) + "," + str(clock[0]) + ")")

				slot = Slot(name + "(" + str(i) + ")", id_ + i, clock[0])
				self.slots.append(slot)

	#
	# Internal method parsing the threads from the project file.
	#
	#   cfg - configparser referencing the project file
	#
	def _parse_threads(self, cfg):
		for t in [_ for _ in cfg.sections() if _.startswith("ReconosThread")]:
			match = re.search(r"^.*@(?P<name>.+)", t)
			if match is None:
				log.error("Thread must have a name")

			name = match.group("name")
			if cfg.has_option(t, "Slot"):
				slots = cfg.get(t, "Slot")
				slots = slots.replace("(", "\\(")
				slots = slots.replace(")", "\\)")
				slots = slots.replace(",", "|")
				slots = slots.replace("*", ".*")
				slots = [_ for _ in self.slots if re.match(slots, _.name) is not None]
			else:
				slots = []
			if cfg.has_option(t, "HwSource"):
				hw = cfg.get(t, "HwSource")
			else:
				hw = None
				log.info("No HwSource found")
			if cfg.has_option(t, "SwSource"):
				sw = cfg.get(t, "SwSource")
			else:
				sw = None
			if cfg.has_option(t, "ResourceGroup"):
				res = re.split(r"[, ]+", cfg.get(t, "ResourceGroup"))
				res = [_ for _ in self.resources if _.group in res]
				if not res:
					log.error("ResourceGroup not found")
			else:
				res = []

			log.debug("Found thread '" + str(name) + "' (" + str(slots) + "," + str(hw) + "," + str(sw) + "," + str(res) + ")")

			thread = Thread(name, slots, hw, sw, res)
			for s in slots: s.threads.append(thread)
			self.threads.append(thread)