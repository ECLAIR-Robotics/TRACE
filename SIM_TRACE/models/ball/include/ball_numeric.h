/*************************************************************************
PURPOSE: (TRACE Numeric Model )
**************************************************************************/

#ifndef TRACE_NUMERIC_H
#define TRACE_NUMERIC_H

#include "ball.h"

#ifdef __cplusplus
extern "C" {
#endif
int robot_integ(BALL*) ;
int robot_deriv(BALL*) ;
#ifdef __cplusplus
}
#endif
#endif