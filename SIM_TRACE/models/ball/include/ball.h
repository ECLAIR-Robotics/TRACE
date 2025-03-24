/*************************************************************************
PURPOSE: (Represent the state and initial conditions of the ball and platform)
**************************************************************************/
#ifndef BALL_H
#define BALL_H

typedef struct {

    double vel0[2] ;    /* *i m Init velocity of ball */
    double pos0[2] ;    /* *i m Init position of ball */

    double integ[2] ;    /*m*s integral of position error*/
    double acc[2] ;     /* m/s2 xy-acceleration  */
    double vel[2] ;     /* m/s xy-velocity */
    double pos[2] ;     /* m xy-position */

    double theta[2];    /*rad angle of plate*/
    double time;        /* s Model time */

    double tiltrate[2]; /* m/s^3 derivative of acceleration, abstract approximation of tiltrate */

    double error[2]; /* m xy position error in previous timestep*/
    double setpoint[2]; /* m xy setpoint for position*/

} BALL ;

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