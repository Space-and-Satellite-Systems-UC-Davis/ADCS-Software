#include "control/experiment/PID_experiment.h"
#include "virtual_intellisat.h"

#include <math.h>
#include <stdint.h>

//TODO: IMU alternation?
//      Negative PID output should activate VI_HDD2
#define IMU_CHOICE VI_IMU1
#define HDD_CHOICE VI_HDD1

#define P_GAIN 0.4
#define I_GAIN 0
#define D_GAIN 0.1

PID_status PID_experiment(double target, int infinite)
{
	//Verify experiment is running

    //Get current angular velocity for z axis
    double angvel_x = 0, angvel_y = 0, angvel_z = 0;

    double throttle = 0;

    if(vi_get_angvel(IMU_CHOICE, &angvel_x, &angvel_y, &angvel_z) == GET_ANGVEL_FAILURE)
        return PID_EXPERIMENT_FAILURE;

    //Get the current time (Virtual Intellisat)
    uint64_t curr_millis = 0;
    if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
        return PID_EXPERIMENT_FAILURE;

    //Declare and initlialize PID controller
    PID_controller controller;
    PID_init(target, angvel_z, curr_millis, P_GAIN, I_GAIN, D_GAIN, &controller);
    
    //Run a while loop 
    while (infinite || (fabs(target - angvel_z) > 0.1))
    {
    	if(vi_get_angvel(IMU_CHOICE, &angvel_x, &angvel_y, &angvel_z) == GET_ANGVEL_FAILURE)
    	    	return PID_EXPERIMENT_FAILURE;

        //Get the current time (Virtual Intellisat)
        if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
            return PID_EXPERIMENT_FAILURE;

        //PLug it into the control function
        throttle += PID_command(target, angvel_z, curr_millis, &controller);

        //Clamp the output
        if (throttle > 2.5){
			throttle = 2.5;
		} else if (throttle < -2.5){
			throttle = -2.5;
		}

        //Take output and plug it into HDD 
        if(vi_hdd_command(HDD_CHOICE, throttle) == HDD_COMMAND_FAILURE)
            return PID_EXPERIMENT_FAILURE;

    }


    return PID_EXPERIMENT_SUCCESS;
}

