/*********************************************************************
  PURPOSE: ( Trick numeric )
*********************************************************************/
#include <stddef.h>
#include <stdio.h>
#include "trick/integrator_c_intf.h"
#include "../include/ball_numeric.h"

int robot_deriv(BALL* B){
    //adding control loop in here to run in sim-time
    double Kp = 59.0/3;
    double Ki = 12.0/3;
    double Kd = 14.0/3;

    B->error[0] = B->setpoint[0] - B->pos[0];
    B->tiltrate[0] = Kp * B->error[0] + Ki * B->integ[0] - Kd * B->vel[0];

    B->error[1] = B->setpoint[1] - B->pos[1];
    B->tiltrate[1] = Kp * B->error[1] + Ki * B->integ[1] - Kd * B->vel[1];
    
    B->acc[0] = 9.81 * B->theta[0];
    B->acc[1] = 9.81 * B->theta[1];

    return (0) ;    
}

int robot_integ(BALL* B){
    int ipass = 0;

    load_state(
        &B->pos[0],
        &B->pos[1],
        &B->vel[0],
        &B->vel[1],
        &B->theta[0],
        &B->theta[1],
        &B->integ[0],
        &B->integ[1],
        NULL
    );

    load_deriv(
        &B->vel[0],
        &B->vel[1],
        &B->acc[0],
        &B->acc[1],
        &B->tiltrate[0],
        &B->tiltrate[1],
        &B->pos[0],
        &B->pos[1],
        NULL
    );
    ipass = integrate();
    unload_state(
        &B->pos[0],
        &B->pos[1],
        &B->vel[0],
        &B->vel[1],
        &B->theta[0],
        &B->theta[1],
        &B->integ[0],
        &B->integ[1],
        NULL
    );
    return (ipass);
}
