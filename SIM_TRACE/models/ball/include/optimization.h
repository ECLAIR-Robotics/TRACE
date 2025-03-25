/******************************* TRICK HEADER ****************************
PURPOSE: Function prototypes for Monte Carlo optimization.
*************************************************************************/

#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include "../include/ball.h"

#ifdef __cplusplus
extern "C" {
#endif

int robot_slave_post(BALL *);
int robot_master_post(BALL *);

#ifdef __cplusplus
}
#endif
#endif