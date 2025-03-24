%module m28df78b22963a8506682986bdd4e0292

%include "trick/swig/trick_swig.i"


%insert("begin") %{
#include <Python.h>
#include <cstddef>
%}

%{
#include "/home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball_numeric.h"
%}





#ifndef TRACE_NUMERIC_H
#define TRACE_NUMERIC_H

%import "build/home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball_py.i"

#ifdef __cplusplus
extern "C" {
#endif
int robot_integ(BALL*) ;
int robot_deriv(BALL*) ;
#ifdef __cplusplus
}
#endif
#endif
