#include <Python.h>
#if PY_VERSION_HEX >= 0x03000000
extern "C" {

PyObject * PyInit__mc950ad8b3f635ee76e9b9a235a82fc32(void) ; /* /home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/S_source.hh */
PyObject * PyInit__m4603a1b6dbdc02b2b48dc52dfef9d9b0(void) ; /* /home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball.h */
PyObject * PyInit__m28df78b22963a8506682986bdd4e0292(void) ; /* /home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball_numeric.h */
PyObject * PyInit__sim_services(void) ;
PyObject * PyInit__top(void) ;
PyObject * PyInit__swig_double(void) ;
PyObject * PyInit__swig_int(void) ;
PyObject * PyInit__swig_ref(void) ;

void init_swig_modules(void) {

    PyImport_AppendInittab("_m4603a1b6dbdc02b2b48dc52dfef9d9b0", PyInit__m4603a1b6dbdc02b2b48dc52dfef9d9b0) ;
    PyImport_AppendInittab("_m28df78b22963a8506682986bdd4e0292", PyInit__m28df78b22963a8506682986bdd4e0292) ;
    PyImport_AppendInittab("_mc950ad8b3f635ee76e9b9a235a82fc32", PyInit__mc950ad8b3f635ee76e9b9a235a82fc32) ;
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

void init_mc950ad8b3f635ee76e9b9a235a82fc32(void) ; /* /home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/S_source.hh */
void init_m4603a1b6dbdc02b2b48dc52dfef9d9b0(void) ; /* /home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball.h */
void init_m28df78b22963a8506682986bdd4e0292(void) ; /* /home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball_numeric.h */
void init_sim_services(void) ;
void init_top(void) ;
void init_swig_double(void) ;
void init_swig_int(void) ;
void init_swig_ref(void) ;

void init_swig_modules(void) {

    init_m4603a1b6dbdc02b2b48dc52dfef9d9b0() ;
    init_m28df78b22963a8506682986bdd4e0292() ;
    init_mc950ad8b3f635ee76e9b9a235a82fc32() ;
    init_sim_services() ;
    init_top() ;
    init_swig_double() ;
    init_swig_int() ;
    init_swig_ref() ;
    return ;
}

}
#endif
