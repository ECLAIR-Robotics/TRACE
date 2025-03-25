/******************************* TRICK HEADER ****************************
PURPOSE: Monte Carlo optimization functions.
*************************************************************************/

#include "../include/optimization.h"
#include "../include/ball.h"
#include "sim_services/MonteCarlo/include/montecarlo_c_intf.h"

int robot_slave_post(BALL *B)
{
    mc_write((char*) C, sizeof(BALL));
    return 0;
}

int cannon_master_post(BALL *B)
{
    BALL run_ball;
    static double previous_approach_time = __DBL_MAX__;
    static double increment = 0.1; // Remember radians

    // Get the run's cannon back from slave.
    mc_read((char*) &run_ball, sizeof(BALL));

    // Optimization logic.
    if(run_ball.approach_time[0] > previous_approach_time)
    {
        // Cut the increment in half and reverse the direction.
        increment /= 2;
        increment *= -1;
    }
    
    B->init_angle += increment;
    previous_approach_time = run_ball.approach_time[0];
    return 0;
}