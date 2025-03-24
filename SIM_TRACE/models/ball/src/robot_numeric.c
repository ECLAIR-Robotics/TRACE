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

    // slide in the rX / rY calculations here!

    B->prevX = B->errorX;
    B->prevY = B->errorY;

    B->errorX = B->targetX - B->pos[0];
    B->errorY = B->targetY - B->pos[1];

    B->integX = B->integX + B->errorX;
    B->integY = B->integY + B->errorY;

    B->derivX = B->errorX - B->prevX;
    B->derivY = B->errorY - B->prevY;

    double rX = B->errorX * B->kP + B->integX * B->kI + B->derivX * B->kD;
    double rY = B->errorY * B->kP + B->integY * B->kI + B->derivY * B->kD;

    if (rX > 30) {
        rX = 30;
    }
    else if (rX < -30) {
        rX = -30;
    }
    if (rY > 30) {
        rY = 30;
    }
    else if (rY < -30) {
        rY = -30;
    }

    B->acc[0] = rX;
    B->acc[1] = rY;

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
    return (ipass);
}
