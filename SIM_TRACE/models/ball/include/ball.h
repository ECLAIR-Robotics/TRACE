/*************************************************************************
PURPOSE: (Represent the state and initial conditions of the ball and platform)
**************************************************************************/
#ifndef BALL_H
#define BALL_H

typedef struct {

    double vel0[2] ;    /* *i m Init velocity of ball */
    double pos0[2] ;    /* *i m Init position of ball */

    double acc[2] ;     /* m/s2 xy-acceleration  */
    double vel[2] ;     /* m/s xy-velocity */
    double pos[2] ;     /* m xy-position */

    double time;        /* s Model time */

    double tiltrate[2]; /* m/s^3 derivative of acceleration, abstract approximation of tiltrate */
    double motor[2]; /*are we moving or not*/

    double kP; /* -- coefficient of proportional component */
    double kI; /* -- coefficient of integral component */
    double kD; /* -- coefficient of derivative component */

    double targetX; /* -- target x coordinate */
    double targetY; /* -- target y coordinate */

    double errorX; /* -- error from target x */
    double errorY; /* -- error from target y */

    double prevX; /* -- previous x error */
    double prevY; /* -- previous y error */

    double integX; /* -- x integral component */
    double integY; /* -- y integral component */

    double derivX; /* -- x derivative component */
    double derivY; /* -- y derivative component */

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