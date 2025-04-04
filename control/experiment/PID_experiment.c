#include "PID_experiment.h"

PID_status PID_experiment()
{
    //Get current angular velocity for z axis
    double angvel_x = 0, angvel_y = 0, angvel_z = 0;
    if(vi_get_angvel(&angvel_x, &angvel_y, &angvel_z) == GET_ANGVEL_FAILURE)
        return PID_EXPERIMENT_FAILURE;

    //Get the current time (Virtual Intellisat)
    int curr_millis = 0;
    if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
        return PID_EXPERIMENT_FAILURE;

    //Declare and initlialize PID controller
    PID_controller controller;
    double target = 0;
    PID_init(target, angvel_z, curr_millis, 1, 1, 1, &controller);
    
    //Run a while loop 
    while (abs(target - angvel_z) > 0.1)
    {
        //Get the current time (Virtual Intellisat)
        if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
            return PID_EXPERIMENT_FAILURE;

        //PLug it into the control function
        double throttle = PID_command(target, angvel_z, curr_millis, &controller);

        //Take output and plug it into HDD 
        if(vi_hdd_command(throttle) == HDD_COMMAND_FAILURE)
            return PID_EXPERIMENT_FAILURE;
    }

    return PID_EXPERIMENT_SUCCESS;
}

