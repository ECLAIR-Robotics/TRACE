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

# /home/rizky-ubuntu/trick_sims/TRACE/SIM_TRACE/S_source.hh
import _m15a240a60fd7eda1c917086950bdde64
combine_cvars(all_cvars, cvar)
cvar = None

# /home/rizky-ubuntu/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball.h
import _m70449b68f80b7909e9d01cb4df0e4b49
combine_cvars(all_cvars, cvar)
cvar = None

# /home/rizky-ubuntu/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball_numeric.h
import _m2469d1330a0d8f9e9ef4d38c51b9c590
combine_cvars(all_cvars, cvar)
cvar = None

# /home/rizky-ubuntu/trick_sims/TRACE/SIM_TRACE/S_source.hh
from m15a240a60fd7eda1c917086950bdde64 import *
# /home/rizky-ubuntu/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball.h
from m70449b68f80b7909e9d01cb4df0e4b49 import *
# /home/rizky-ubuntu/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball_numeric.h
from m2469d1330a0d8f9e9ef4d38c51b9c590 import *

# S_source.hh
import _m15a240a60fd7eda1c917086950bdde64
from m15a240a60fd7eda1c917086950bdde64 import *

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

