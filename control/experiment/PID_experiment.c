#include "control/experiment/PID_experiment.h"
#include "virtual_intellisat.h"

#include <math.h>
#include <stdint.h>

//TODO: IMU alternation?
//      Negative PID output should activate VI_HDD2
#define IMU_CHOICE VI_IMU1

#define P_GAIN 0.4
#define I_GAIN 0
#define D_GAIN 0.1

PID_status PID_experiment()
{
	//Verify experiment is running

    //Get current angular velocity for z axis
    double angvel_x = 0, angvel_y = 0, angvel_z = 0;

    double throttle = 0;

    vi_HDD hdd_choice = VI_HDD1;

    if(vi_get_angvel(IMU_CHOICE, &angvel_x, &angvel_y, &angvel_z) == GET_ANGVEL_FAILURE)
        return PID_EXPERIMENT_FAILURE;

    //Get the current time (Virtual Intellisat)
    uint64_t curr_millis = 0;
    if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
        return PID_EXPERIMENT_FAILURE;

    //Declare and initlialize PID controller
    PID_controller controller;
    double target = 0;
    PID_init(target, angvel_z, curr_millis, P_GAIN, I_GAIN, D_GAIN, &controller);
    
    //Run a while loop 
    while (fabs(target - angvel_z) > 0.1)
    {
    	if(vi_get_angvel(IMU_CHOICE, &angvel_x, &angvel_y, &angvel_z) == GET_ANGVEL_FAILURE)
    	    	return PID_EXPERIMENT_FAILURE;

        //Get the current time (Virtual Intellisat)
        if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
            return PID_EXPERIMENT_FAILURE;

        //PLug it into the control function
        throttle += PID_command(target, angvel_z, curr_millis, &controller);

        //Clamp the output
        if (throttle >= 0) {
            hdd_choice = VI_HDD1;
            if (throttle > 2.5){
			    throttle = 2.5;
            }
		} else {
            hdd_choice = VI_HDD2;
            if (throttle < -2.5){
                throttle = -2.5;
            }
        }


        //Take output and plug it into HDD 
        if(vi_hdd_command(hdd_choice, fabs(throttle)) == HDD_COMMAND_FAILURE)
            return PID_EXPERIMENT_FAILURE;

    }


    return PID_EXPERIMENT_SUCCESS;
}

