#include "control/experiment/determination_experiment.h"
#include "determination/determination.h"
#include "adcs_math/vector.h"
#include "adcs_math/sensors.h"

//TODO: MAG, HDD alternation?
#define MAG_CHOICE MAG1
#define HDD_CHOICE HDD1


determination_exp_status determination_experiment()
{
    mat3 prevAttitude;
    mat3 currAttitude;
    int year, month, day, hour, minute, second;
    vec3 mag;
    vec3 sun;

    vi_get_epoch(&year, &month, &day, &hour, &minute, &second);
    vi_get_mag(MAG_CHOICE, &mag.x, &mag.y, &mag.z);
    
    //TODO: default values for now, waiting for sun sensors to implement get_sun
    sun.x = 0;
    sun.y = 0;
    sun.z = 0;

    determination(year, month, day, hour, minute, second, mag, sun, &prevAttitude);

    //TODO: generalizing initial angular velocity as 0; might have to fix
    int angvel_z = 0;

    //Get the current time (Virtual Intellisat)
    int prev_millis = 0;
    int curr_millis = 0;
    if(vi_get_curr_millis(&prev_millis) == GET_CURR_MILLIS_FAILURE)
        return DETERMINATION_EXPERIMENT_FAILURE;

    //Declare and initlialize PID controller
    PID_controller controller;
    double target = 0;
    PID_init(target, angvel_z, prev_millis, 1, 1, 1, &controller);
    
    //Run a while loop 
    while (abs(target - angvel_z) > 0.1)
    {
        vi_delay_ms(100);
        vi_get_epoch(&year, &month, &day, &hour, &minute, &second);
        vi_get_mag(MAG_CHOICE, &mag.x, &mag.y, &mag.z);
        // default values for now, waiting for sun sensors to implement get_sun
        sun.x = 0;
        sun.y = 0;
        sun.z = 0;

        determination(year, month, day, hour, minute, second, mag, sun, &currAttitude);
        //Get the current time (Virtual Intellisat)
        if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
            return DETERMINATION_EXPERIMENT_FAILURE;
        
        int delta_t = get_delta_t(curr_millis, prev_millis);
        
        mat3 derivative;
        mat_sub(currAttitude, prevAttitude, &derivative);
        mat_scalar(1.0/delta_t, derivative, &derivative);
        double zrotation = derivative.y1;

        //PLug it into the control function
        double throttle = PID_command(target, zrotation, curr_millis, &controller);
        //Take output and plug it into HDD 
        if(vi_hdd_command(HDD_CHOICE, throttle) == HDD_COMMAND_FAILURE)
            return DETERMINATION_EXPERIMENT_FAILURE;
        prevAttitude = currAttitude;
        prev_millis = curr_millis;
    }

    return DETERMINATION_EXPERIMENT_SUCCESS;
}

