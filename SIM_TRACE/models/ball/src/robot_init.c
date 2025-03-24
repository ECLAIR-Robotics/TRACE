/******************************* TRICK HEADER ****************************
PURPOSE: (Set the initial data values)
*************************************************************************/

/* Model Include files */
#include <math.h>
#include "../include/ball.h"

/* default data job */
int robot_default_data( BALL* B ) {

    B->acc[0] = 0.0;
    B->acc[1] = 0;

    B->pos0[0] = 0.0 ;
    B->pos0[1] = 0.0 ;

    B->vel0[0] = 0;
    B->vel0[1] = 0;

    B->time = 0.0 ;
    B->setpoint[0] = 1;
    B->setpoint[1] = 1;

    return 0 ;
}

/* initialization job */
int robot_init(BALL* B) {

    B->vel[0] = B->vel0[0] ; 
    B->vel[1] = B->vel0[1] ; 


    B->tiltrate[0] = 0;
    B->tiltrate[1] = 0;

    B->pos[0] = B->pos0[0];
    B->pos[1] = B->pos0[1];
    
    B->error[0] = B->setpoint[0] - B->pos[0];
    B->error[1] = B->setpoint[1] - B->pos[1];

    B->integ[0] = 0;
    B->integ[1] = 0;

    B->theta[0] =0;
    B->theta[1] =0;
    return 0 ; 
}