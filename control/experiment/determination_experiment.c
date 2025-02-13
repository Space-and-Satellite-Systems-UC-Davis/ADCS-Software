#include "determination_experiment.h"
#include "determination.h"
#include "vector.h"

determination_exp_status determination_experiment()
{
    mat3 prevAttitude;
    mat3 currAttitude;
    int year, month, day, hour, minute, second;
    vec3 mag;
    // have to calculate/get measured_sun (from where?)
    vec3 sun;

    vi_get_epoch(&year, &month, &day, &hour, &minute, &second);
    vi_get_mag(&mag.x, &mag.y, &mag.z);
    
    // default values for now, waiting for sun sensors to implement get_sun
    sun.x = 0;
    sun.y = 0;
    sun.z = 0;

    determination(year, month, day, hour, minute, second, mag, sun, &currAttitude);

    //Get current angular velocity for z axis
    double angvel_x = 0, angvel_y = 0, angvel_z = 0;
    if(vi_get_angvel(&angvel_x, &angvel_y, &angvel_z) == GET_ANGVEL_FAILURE)
        return DETERMINATION_EXPERIMENT_FAILURE;

    //Get the current time (Virtual Intellisat)
    int curr_millis = 0;
    if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
        return DETERMINATION_EXPERIMENT_FAILURE;

    //Declare and initlialize PID controller
    PID_controller controller;
    double target = 0;
    PID_init(target, angvel_z, curr_millis, 1, 1, 1, &controller);
    
    //Run a while loop 
    while (abs(target - angvel_z) > 0.1)
    {
        //Get the current time (Virtual Intellisat)
        if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
            return DETERMINATION_EXPERIMENT_FAILURE;

        //PLug it into the control function
        double throttle = PID_command(target, state, curr_millis, &controller);
        //Take output and plug it into HDD 
        if(vi_hdd_command(throttle) == HDD_COMMAND_FAILURE)
            return DETERMINATION_EXPERIMENT_FAILURE;
    }

    return DETERMINATION_EXPERIMENT_SUCCESS;
}

