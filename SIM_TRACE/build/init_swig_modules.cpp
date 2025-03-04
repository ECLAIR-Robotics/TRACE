#include <Python.h>
#if PY_VERSION_HEX >= 0x03000000
extern "C" {

PyObject * PyInit__m15a240a60fd7eda1c917086950bdde64(void) ; /* /home/rizky-ubuntu/trick_sims/TRACE/SIM_TRACE/S_source.hh */
PyObject * PyInit__m70449b68f80b7909e9d01cb4df0e4b49(void) ; /* /home/rizky-ubuntu/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball.h */
PyObject * PyInit__m2469d1330a0d8f9e9ef4d38c51b9c590(void) ; /* /home/rizky-ubuntu/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball_numeric.h */
PyObject * PyInit__sim_services(void) ;
PyObject * PyInit__top(void) ;
PyObject * PyInit__swig_double(void) ;
PyObject * PyInit__swig_int(void) ;
PyObject * PyInit__swig_ref(void) ;

void init_swig_modules(void) {

    PyImport_AppendInittab("_m70449b68f80b7909e9d01cb4df0e4b49", PyInit__m70449b68f80b7909e9d01cb4df0e4b49) ;
    PyImport_AppendInittab("_m2469d1330a0d8f9e9ef4d38c51b9c590", PyInit__m2469d1330a0d8f9e9ef4d38c51b9c590) ;
    PyImport_AppendInittab("_m15a240a60fd7eda1c917086950bdde64", PyInit__m15a240a60fd7eda1c917086950bdde64) ;
    PyImport_AppendInittab("_sim_services", PyInit__sim_services) ;
    PyImport_AppendInittab("_top", PyInit__top) ;
    PyImport_AppendInittab("_swig_double", PyInit__swig_double) ;
    PyImport_AppendInittab("_swig_int", PyInit__swig_int) ;
    PyImport_AppendInittab("_swig_ref", PyInit__swig_ref) ;
    return ;
}

}
#else
extern "C" {

void init_m15a240a60fd7eda1c917086950bdde64(void) ; /* /home/rizky-ubuntu/trick_sims/TRACE/SIM_TRACE/S_source.hh */
void init_m70449b68f80b7909e9d01cb4df0e4b49(void) ; /* /home/rizky-ubuntu/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball.h */
void init_m2469d1330a0d8f9e9ef4d38c51b9c590(void) ; /* /home/rizky-ubuntu/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball_numeric.h */
void init_sim_services(void) ;
void init_top(void) ;
void init_swig_double(void) ;
void init_swig_int(void) ;
void init_swig_ref(void) ;

void init_swig_modules(void) {

    init_m70449b68f80b7909e9d01cb4df0e4b49() ;
    init_m2469d1330a0d8f9e9ef4d38c51b9c590() ;
    init_m15a240a60fd7eda1c917086950bdde64() ;
    init_sim_services() ;
    init_top() ;
    init_swig_double() ;
    init_swig_int() ;
    init_swig_ref() ;
    return ;
}

}
#endif
