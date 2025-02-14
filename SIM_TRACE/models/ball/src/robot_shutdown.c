/*****************************************
 * PURPOSE: (PRINT THE FINAL ROBOT STATE)
*******************************************/

#include <stdio.h>
#include "../include/ball.h"
#include "trick/exec_proto.h"

int robot_shutdown(BALL* B){
    double t = exec_get_sim_time();
    printf( "========================================\n");
    printf( "      Robot State at Shutdown     \n");
    printf( "t = %g\n", t);
    printf( "pos = [%.9f, %.9f]\n", B->pos[0], B->pos[1]);
    printf( "vel = [%.9f, %.9f]\n", B->vel[0], B->vel[1]);
    printf( "========================================\n");
    return 0;
}