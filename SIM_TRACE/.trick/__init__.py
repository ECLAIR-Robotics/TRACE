from pkgutil import extend_path
__path__ = extend_path(__path__, __name__)
import sys
import os
sys.path.append(os.getcwd() + "/trick.zip/trick")

import _sim_services
from sim_services import *

# create "all_cvars" to hold all global/static vars
all_cvars = new_cvar_list()
combine_cvars(all_cvars, cvar)
cvar = None

# /home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/S_source.hh
import _mc950ad8b3f635ee76e9b9a235a82fc32
combine_cvars(all_cvars, cvar)
cvar = None

# /home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball.h
import _m4603a1b6dbdc02b2b48dc52dfef9d9b0
combine_cvars(all_cvars, cvar)
cvar = None

# /home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball_numeric.h
import _m28df78b22963a8506682986bdd4e0292
combine_cvars(all_cvars, cvar)
cvar = None

# /home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/S_source.hh
from mc950ad8b3f635ee76e9b9a235a82fc32 import *
# /home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball.h
from m4603a1b6dbdc02b2b48dc52dfef9d9b0 import *
# /home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball_numeric.h
from m28df78b22963a8506682986bdd4e0292 import *

# S_source.hh
import _mc950ad8b3f635ee76e9b9a235a82fc32
from mc950ad8b3f635ee76e9b9a235a82fc32 import *

import _top
import top

import _swig_double
import swig_double

import _swig_int
import swig_int

import _swig_ref
import swig_ref

from shortcuts import *

from exception import *

cvar = all_cvars

