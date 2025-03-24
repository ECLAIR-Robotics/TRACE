%module m4603a1b6dbdc02b2b48dc52dfef9d9b0

%include "trick/swig/trick_swig.i"


%insert("begin") %{
#include <Python.h>
#include <cstddef>
%}

%{
#include "/home/arnavjoshi/trick_sims/TRACE/SIM_TRACE/models/ball/include/ball.h"
%}

%trick_swig_class_typemap(BALL, BALL)



#ifndef BALL_H
#define BALL_H

typedef struct {

    double vel0[2] ;    

    double pos0[2] ;    


    double integ[2] ;    

    double acc[2] ;     

    double vel[2] ;     

    double pos[2] ;     


    double theta[2];    

    double time;        


    double tiltrate[2]; 


    double error[2]; 

    double setpoint[2]; 


} BALL ;
#if SWIG_VERSION > 0x040000
%pythoncode %{
    if 'BALL' in globals():
        BALL.__setattr__ = _swig_setattr_nondynamic_instance_variable(object.__setattr__)
%}
#endif


#ifdef __cplusplus
extern "C" {
#endif
    int robot_default_data(BALL*) ;
    int robot_init(BALL*) ;
    int robot_shutdown(BALL*) ;
#ifdef __cplusplus
}
#endif

#endif
#ifdef TRICK_SWIG_DEFINED_BALL
%trick_cast_as(BALL, BALL)
#endif
