#include "control/experiment/PID_experiment.h"
#include "virtual_intellisat.h"

#include <math.h>
#include <stdint.h>
#include "print_scan.h"
#include "../hdd/hdd_init.h"
#include "../system_config/Timers/timers.h"

//TODO: IMU alternation?
//      Negative PID output should activate VI_HDD2
#define IMU_CHOICE VI_IMU1

#define P_GAIN 1000.0
#define I_GAIN 0.0
#define D_GAIN 0.0

PID_status PID_experiment()
{
	//Verify experiment is running

    //Get current angular velocity for z axis
    double angvel_x = 0, angvel_y = 0, angvel_z = 0;

    double throttle = 0;
    double throttleBound = MID_DUTY * 1000;  // let throttle represent three duty decimal places

    vi_HDD hdd_choice = VI_HDD1;

    printMsg("Trying to throttle HDD\r\n");
    pwm_setDutyCycle(0, 100);
    delay_ms(5000);

    // set back to middle
    pwm_setDutyCycle(0, MID_DUTY);
    pwm_setDutyCycle(1, MID_DUTY);

    if(vi_get_angvel(IMU_CHOICE, &angvel_x, &angvel_y, &angvel_z) == GET_ANGVEL_FAILURE)
        return PID_EXPERIMENT_FAILURE;

    //Get the current time (Virtual Intellisat)
    uint64_t curr_millis = 0;
    if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
        return PID_EXPERIMENT_FAILURE;

    //Declare and initialize PID controller
    PID_controller controller;
    double target = 0;
    PID_init(target, angvel_z, curr_millis, P_GAIN, I_GAIN, D_GAIN, &controller);
    
    //Run a while loop 
    led_d2(1);
    led_d3(1);
    printMsg("Got through PID experiment initialization; beginning loop. \r\n");
    uint64_t timeLastOn = curr_millis;
    uint64_t timeLastPrint = curr_millis;
    while (1)
    {
    	//Get the current time (Virtual Intellisat)
    	if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
    	return PID_EXPERIMENT_FAILURE;

    	// if it hasn't been a second, don't print
    	int8_t doPrint = 0;
    	if (curr_millis - timeLastPrint >= 3000) {
    		doPrint = 1;
    		timeLastPrint = curr_millis;
    		printMsg("\r\ncurr Time: %u, timeLastOn: %u \r\n", (uint32_t) curr_millis, (uint32_t) timeLastOn);
    	}

    	// get the velocity data
    	if(vi_get_angvel(IMU_CHOICE, &angvel_x, &angvel_y, &angvel_z) == GET_ANGVEL_FAILURE)
    	    	    	return PID_EXPERIMENT_FAILURE;

    	// provide a status of the velocity data
    	if (doPrint) {
    	    printMsg("Found angvel_z of %f \r\n", angvel_z);
    	}

    	// if the velocity is too low to respond to
    	if (fabs(target - angvel_z) <= 1.0) {
    		if (doPrint) { printMsg("Did not detect significant angular velocity.\r\n"); }

    		// after a minute, stop the PID experiment
    		if (curr_millis - timeLastOn > 60000) {
    			return PID_EXPERIMENT_SUCCESS;
    		}

    		// if it hasn't been a minute, keep the test going
    		continue;
    	}

    	timeLastOn = curr_millis;

    	if (doPrint) { printMsg("Starting with throttle %f.\r\n", throttle); }

        //Plug it into the control function
        throttle += PID_command(target, angvel_z, curr_millis, &controller);

        // bound the resulting throttle and scale down
        if (throttle > 0 && throttle > throttleBound) { throttle = throttleBound; }
        else if (throttle < 0 && throttle < -throttleBound) { throttle = -throttleBound; }
        throttle /= 1000;  // scale down

        if (doPrint) { printMsg("Updated throttle to %f.\r\n", throttle); }

        // we define HDD1 to be in the same orientation
        // (face up in the same z direction)
        // as the board, with HDD2 in reverse
        uint8_t duty1 = pwm_getDutyCycle(PWM0);
        uint8_t duty2 = pwm_getDutyCycle(PWM1);
        if (throttle > 0 || (throttle < 0 && duty1 > MID_DUTY)) {
            hdd_choice = VI_HDD1;
		} else {
            hdd_choice = VI_HDD2;
        }

        if (doPrint) { printMsg("HDD1 duty: %u, HDD2 duty: %u\r\n", duty1, duty2); }

        //Take output and plug it into HDD 
        if(vi_hdd_command(hdd_choice, throttle, doPrint) == HDD_COMMAND_FAILURE)
            return PID_EXPERIMENT_FAILURE;

        if (doPrint) { printMsg("Got result duty of: %d for HDD %d\r\n", pwm_getDutyCycle(hdd_choice == VI_HDD1 ? PWM0 : PWM1), hdd_choice + 1); }
    }


    return PID_EXPERIMENT_SUCCESS;
}

