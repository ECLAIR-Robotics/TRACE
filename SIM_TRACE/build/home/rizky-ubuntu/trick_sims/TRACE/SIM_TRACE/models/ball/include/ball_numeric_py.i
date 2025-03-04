%module m2469d1330a0d8f9e9ef4d38c51b9c590

%include "trick/swig/trick_swig.i"


%insert("begin") %{
#include <Python.h>
#include <cstddef>
%}

%{
#include "/home/rizky-ubuntu/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball_numeric.h"
%}





#ifndef TRACE_NUMERIC_H
#define TRACE_NUMERIC_H

%import "build/home/rizky-ubuntu/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball_py.i"

#ifdef __cplusplus
extern "C" {
#endif
int robot_integ(BALL*) ;
int robot_deriv(BALL*) ;
#ifdef __cplusplus
}
#endif
#endif
