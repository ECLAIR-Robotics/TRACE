/*********************************************************************
  PURPOSE: ( Trick numeric )
*********************************************************************/
#include <stddef.h>
#include <stdio.h>
#include "trick/integrator_c_intf.h"
#include "../include/ball_numeric.h"

int robot_deriv(BALL* B){
    double MOTORSPEED =.1;
    B->tiltrate[0] = MOTORSPEED * B->motor[0];
    B->tiltrate[1] = MOTORSPEED * B->motor[1];
    return (0) ;    
}

int robot_integ(BALL* B){
    int ipass = 0;

    load_state(
        &B->pos[0],
        &B->pos[1],
        &B->vel[0],
        &B->vel[1],
        &B->acc[0],
        &B->acc[1],
        NULL
    );

    load_deriv(
        &B->vel[0],
        &B->vel[1],
        &B->acc[0],
        &B->acc[1],
        &B->tiltrate[0],
        &B->tiltrate[1],
        NULL
    );
    ipass = integrate();
    unload_state(
        &B->pos[0],
        &B->pos[1],
        &B->vel[0],
        &B->vel[1],
        &B->acc[0],
        &B->acc[1],
        NULL
    );
    puts("runinteg");
    return (ipass);
}
