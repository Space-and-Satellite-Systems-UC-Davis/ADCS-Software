#include "control/experiment/PID_experiment.h"
#include "virtual_intellisat.h"

#include <math.h>

//TODO: IMU, HDD alternation?
#define IMU_CHOICE VI_IMU1
#define HDD_CHOICE VI_HDD1


PID_status PID_experiment()
{
	//Verify experiment is running

    //Get current angular velocity for z axis
    double angvel_x = 0, angvel_y = 0, angvel_z = 0;

    if(vi_get_angvel(IMU_CHOICE, &angvel_x, &angvel_y, &angvel_z) == GET_ANGVEL_FAILURE)
        	    	return PID_EXPERIMENT_FAILURE;


    vi_print("%f \r \n", angvel_z);



    //Get the current time (Virtual Intellisat)
    int curr_millis = 0;
    if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
        return PID_EXPERIMENT_FAILURE;

    //Declare and initlialize PID controller
    PID_controller controller;
    double target = 0;
    PID_init(target, angvel_z, curr_millis, 1, 1, 1, &controller);
    
    //Run a while loop 
    while (fabs(target - angvel_z) > 1)
    {
    	if(vi_get_angvel(IMU_CHOICE, &angvel_x, &angvel_y, &angvel_z) == GET_ANGVEL_FAILURE)
    	    	return PID_EXPERIMENT_FAILURE;

    	vi_print("target: %f \r \n",target);
    	vi_print("angvel: %f \r \n",angvel_z);
        //Get the current time (Virtual Intellisat)
        if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
            return PID_EXPERIMENT_FAILURE;

        //PLug it into the control function
        double throttle = PID_command(target, angvel_z, curr_millis, &controller);

        //Take output and plug it into HDD 
        if(vi_hdd_command(HDD_CHOICE, throttle) == HDD_COMMAND_FAILURE)
            return PID_EXPERIMENT_FAILURE;
    }


    return PID_EXPERIMENT_SUCCESS;
}

