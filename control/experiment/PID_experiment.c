#include "control/experiment/PID_experiment.h"
#include "virtual_intellisat.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "print_scan.h"
#include "../hdd/hdd_init.h"
#include "../system_config/Timers/timers.h"

//TODO: IMU alternation?
//      Negative PID output should activate VI_HDD2
#define IMU_CHOICE VI_IMU1
//#define PRINT 0  // if print is defined characteristics will be printed that many ms
//#define PROFILE false  // if profile is defined loop time will be printed

double P_GAIN = 0.2;
double I_GAIN = 0.0;
double D_GAIN = 0.55;

void readPIDUART(PID_controller *pid, double *P, double *I, double *D, double angvel_z, int doPrint);

PID_status PID_experiment()
{
	//Verify experiment is running
	printMsg("Beginning PID Exp with variables P: %f, I: %f, D: %f\r\n", P_GAIN, I_GAIN, D_GAIN);

    //Get current angular velocity for z axis
    double angvel_x = 0, angvel_y = 0, angvel_z = 0;

    float throttle = 0;

    vi_HDD hdd_choice = VI_HDD1;

    printMsg("Trying to throttle first HDD\r\n");
    pwm_setDutyCycle(PWM0, SLIP_DUTY);
    delay_ms(5000);

    // set back to middle
    pwm_setDutyCycle(PWM0, MID_DUTY);

    printMsg("Now throttling second HDD\r\n");
    pwm_setDutyCycle(PWM1, SLIP_DUTY);
    delay_ms(5000);

    pwm_setDutyCycle(PWM1, MID_DUTY);

    if(vi_get_angvel_z(IMU_CHOICE, &angvel_z) == GET_ANGVEL_FAILURE)
        return PID_EXPERIMENT_FAILURE;

    //Get the current time (Virtual Intellisat)
    uint64_t curr_millis = 0;
    if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
        return PID_EXPERIMENT_FAILURE;

    //Declare and initialize PID controller
    PID_controller controller;
    double target = 0;
    PID_init(target, angvel_z, curr_millis, P_GAIN, I_GAIN, D_GAIN, &controller);

    uint8_t duty1 = pwm_getDutyCycle(PWM0);
    uint8_t duty2 = pwm_getDutyCycle(PWM1);

    //Run a while loop
    led_d2(1);
    led_d3(1);
    printMsg("Got through PID experiment initialization; beginning loop. \r\n");
    uint64_t timeLastOn = curr_millis;
    uint64_t timeLastPrint = curr_millis;
    int8_t doPrint = 1;
    uint64_t lastCheckpoint = curr_millis;
    uint64_t nextCheckpoint = curr_millis;
    delay_ms(5);  // wait to start execution so that curr_millis start != curr_millis in loop

    // more efficient, less detailed loop ~125ms / loop
    while (1) {
    	delay_ms(1);
    	//Get the current time (Virtual Intellisat)
    	if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE) {
    		return PID_EXPERIMENT_FAILURE;
    	}

#ifdef PROFILE
    	vi_get_curr_millis(&nextCheckpoint);
    	printMsg("Get millis dt: %ums\r\n", (uint32_t) (nextCheckpoint - lastCheckpoint));
    	vi_get_curr_millis(&lastCheckpoint);

    	printMsg("loop dt: %ums\r\n", (uint32_t) (curr_millis - timeLastOn));
    	timeLastOn = curr_millis;
#endif

    	// get the velocity data
    	if(vi_get_angvel_z(IMU_CHOICE, &angvel_z) == GET_ANGVEL_FAILURE) {
    		return PID_EXPERIMENT_FAILURE;
    	}

#ifdef PRINT
    	doPrint = curr_millis - timeLastPrint >= PRINT;
    	if (doPrint) { timeLastPrint = curr_millis; }
#endif

#ifdef PROFILE
    	vi_get_curr_millis(&nextCheckpoint);
    	printMsg("Get angvel dt: %ums\r\n", (uint32_t) (nextCheckpoint - lastCheckpoint));
    	vi_get_curr_millis(&lastCheckpoint);
#endif

    	// if the velocity is too low to respond to
    	if (fabs(target - angvel_z) <= 0.1) {
    	    if (doPrint) { printMsg("\r\nDid not detect significant angular velocity.\r\n"); }

    	    // after a minute, stop the PID experiment
    	    if (curr_millis - timeLastOn > 60000) {
    	    	return PID_EXPERIMENT_SUCCESS;
    	    }

    	    // if it hasn't been a minute, keep the test going
    	    angvel_z = 0.0;
    	    //continue;
    	}

    	timeLastOn = curr_millis;

    	//Plug it into the control function
    	throttle = PID_command(doPrint, angvel_z, curr_millis, &controller);

#ifdef PROFILE
    	vi_get_curr_millis(&nextCheckpoint);
    	printMsg("PID dt: %ums\r\n", (uint32_t) (nextCheckpoint - lastCheckpoint));
    	vi_get_curr_millis(&lastCheckpoint);
#endif

    	// bound the resulting throttle
    	if (throttle > MID_DUTY) { throttle = MID_DUTY; }
    	else if (throttle < -MID_DUTY) { throttle = -MID_DUTY; }

    	// we define HDD1 to be in the same orientation
    	// (face up in the same z direction)
    	// as the board, with HDD2 in reverse
    	duty1 = pwm_getDutyCycle(PWM0);
    	duty2 = pwm_getDutyCycle(PWM1);
    	if ((throttle > 0 && duty2 <= MID_DUTY) || (throttle < 0 && duty1 > MID_DUTY)) {
    		hdd_choice = VI_HDD1;
    		led_d2(1);
    		led_d3(0);
    	} else {
    		hdd_choice = VI_HDD2;
    		led_d2(0);
    		led_d3(1);
    		throttle *= -1;  // invert because 2nd HDD is inverted
    	}

#ifdef PROFILE
    	vi_get_curr_millis(&nextCheckpoint);
    	printMsg("Handle PID result dt: %ums\r\n", (uint32_t) (nextCheckpoint - lastCheckpoint));
    	vi_get_curr_millis(&lastCheckpoint);
#endif

#ifdef PRINT
    	if (doPrint) {
    		printMsg("B4 DRIVE DUTIES @ <%ums> => duty1: %u, duty2: %u\r\n", (uint32_t) curr_millis, duty1, duty2);
    		printMsg("Commanding HDD #%u w/ throttle: %f\r\n", hdd_choice + 1, throttle);
    	}
#endif

    	//Take output and plug it into HDD
    	if(vi_hdd_command(hdd_choice, throttle, 0) == HDD_COMMAND_FAILURE) {
    		return PID_EXPERIMENT_FAILURE;
    	}

#ifdef PROFILE
    	vi_get_curr_millis(&nextCheckpoint);
    	printMsg("Command hdd time: %ums\r\n", (uint32_t) (nextCheckpoint - lastCheckpoint));
    	vi_get_curr_millis(&lastCheckpoint);
#endif
    }


    // original loop
    while (0)
    {
    	//Get the current time (Virtual Intellisat)
    	if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
    		return PID_EXPERIMENT_FAILURE;

    	vi_get_curr_millis(&nextCheckpoint);
    	//printMsg("Get millis dt: %u\r\n", (uint32_t) (nextCheckpoint - lastCheckpoint));
    	vi_get_curr_millis(&lastCheckpoint);

    	// if it hasn't been a bit, don't print
    	if (curr_millis - timeLastPrint >= 0) {
    		doPrint = 1;
    		//readPIDUART(&controller, &P_GAIN, &I_GAIN, &D_GAIN, angvel_z, doPrint);  // process UART
    		printMsg("\r\ncurr Time: %u, timeLastOn: %u \r\n", (uint32_t) curr_millis, (uint32_t) timeLastOn);
    	}

    	// get the velocity data
    	if(vi_get_angvel(IMU_CHOICE, &angvel_x, &angvel_y, &angvel_z) == GET_ANGVEL_FAILURE)
    	    	    	return PID_EXPERIMENT_FAILURE;

    	//if (doPrint) { printMsg("Controller parameters => ePrev: %f, eCum: %f, dt: %u\r\n", controller.e_prev, controller.e_cumulative, curr_millis - controller.t_prev); }

    	// provide a status of the velocity data
    	if (doPrint) {
    	    printMsg("Found angvel_z of %f \r\n", angvel_z);
    	}

    	vi_get_curr_millis(&nextCheckpoint);
    	if (doPrint) { printMsg("Get angvel dt: %u\r\n", (uint32_t) (nextCheckpoint - lastCheckpoint)); }
    	vi_get_curr_millis(&lastCheckpoint);

    	// if the velocity is too low to respond to
    	if (fabs(target - angvel_z) <= 0.1) {
    		if (doPrint) { printMsg("Did not detect significant angular velocity.\r\n"); }

    		// after a minute, stop the PID experiment
    		if (curr_millis - timeLastOn > 60000) {
    			return PID_EXPERIMENT_SUCCESS;
    		}

    		// if it hasn't been a minute, keep the test going
    		angvel_z = 0.0;
    		//continue;
    	}

    	vi_get_curr_millis(&nextCheckpoint);
    	if (doPrint) { printMsg("Fabs dt: %u\r\n", (uint32_t) (nextCheckpoint - lastCheckpoint)); }
    	vi_get_curr_millis(&lastCheckpoint);

    	printMsg("About to pid command with timeLastOn: %u, currMillis: %u\r\n", (uint32_t) timeLastOn, (uint32_t) curr_millis);
    	timeLastOn = curr_millis;

    	if (doPrint) { printMsg("Starting with throttle %f.\r\n", throttle); }

        //Plug it into the control function
        throttle += PID_command(target, -angvel_z, curr_millis, &controller);

        // bound the resulting throttle and scale down
        if (throttle > MID_DUTY) { throttle = MID_DUTY; }
        else if (throttle < MID_DUTY) { throttle = -MID_DUTY; }

        if (doPrint) { printMsg("Updated throttle to %f.\r\n", throttle); }

        vi_get_curr_millis(&nextCheckpoint);
        if (doPrint) { printMsg("Get throttle dt: %u\r\n", (uint32_t) (nextCheckpoint - lastCheckpoint)); }
        vi_get_curr_millis(&lastCheckpoint);

        // we define HDD1 to be in the same orientation
        // (face up in the same z direction)
        // as the board, with HDD2 in reverse
        uint8_t duty1 = pwm_getDutyCycle(PWM0);
        uint8_t duty2 = pwm_getDutyCycle(PWM1);
        if ((throttle > 0 && duty2 <= MID_DUTY) || (throttle < 0 && duty1 > MID_DUTY)) {
            hdd_choice = VI_HDD1;
            if (throttle < 0) { throttle = -(pwm_getDutyCycle(PWM0) - MID_DUTY); }
		} else {
            hdd_choice = VI_HDD2;
            if (throttle > 0) { throttle = -(pwm_getDutyCycle(PWM1) - MID_DUTY); }
            else { throttle = -throttle; }  // make throttle positive
        }

        if (doPrint) { printMsg("HDD1 duty: %u, HDD2 duty: %u\r\n", duty1, duty2); }

        vi_get_curr_millis(&nextCheckpoint);
        if (doPrint) { printMsg("Get HDD dt: %u\r\n", (uint32_t) (nextCheckpoint - lastCheckpoint)); }
        vi_get_curr_millis(&lastCheckpoint);

        //Take output and plug it into HDD
        if(vi_hdd_command(hdd_choice, throttle, doPrint) == HDD_COMMAND_FAILURE)
            return PID_EXPERIMENT_FAILURE;

        if (!doPrint) { continue; }

        if (doPrint) { printMsg("Got result duty of: %d for HDD %d\r\n", pwm_getDutyCycle(hdd_choice == VI_HDD1 ? PWM0 : PWM1), hdd_choice + 1); }

        // turn printing back off
        if (doPrint) {
        	timeLastPrint = curr_millis;
        	doPrint = 0;
        }

        vi_get_curr_millis(&nextCheckpoint);
        printMsg("Command HDD dt: %u\r\n", (uint32_t) (nextCheckpoint - lastCheckpoint));
        vi_get_curr_millis(&lastCheckpoint);
        printMsg("Finished loop in: %u\r\n", (uint32_t) (lastCheckpoint - curr_millis));
    }


    return PID_EXPERIMENT_SUCCESS;
}

// from from uart and respond accordingly
void readPIDUART(PID_controller *pid, double *P, double *I, double *D, double angvel_z, int doPrint) {
	const uint8_t rxMax = 64;
	uint8_t received[rxMax];
	int numReceived = readMsg(received, rxMax);

	// stop if no valid message was received
	if (numReceived == 0) {
		if (doPrint) { printMsg("Received no UART data\r\n"); }
		return;
	}

	printMsg("Received: ");
	printMsg(received);
	printMsg("\r\n");

	char cmdChar = received[0];
	if (cmdChar != 'R' && cmdChar != 'P' && cmdChar != 'I' && cmdChar != 'D') {
		if (doPrint) { printMsg("Received no valid PID command character at start <%c, %d>\r\n", received[0], received[0]); }
		return;
	}

	uint8_t rxIndex = 0;
	if (cmdChar == 'R') {
		delay_ms(3000);
		angvel_z = 0;
		rxIndex = numReceived;  // skip all other commands
	}

	// 'R' alone calls for a reset, with each term PID controlling the value for the terms
	if (cmdChar == 'P') {
		printMsg("Got Proportional command\r\n");
		// read in characters
		uint8_t termCharIndex = 0;
		char termVal[numReceived + 1];
		++rxIndex;  // we don't want to carry over command char
		while ((char) received[rxIndex] != ' ' && rxIndex < numReceived) {
			termVal[termCharIndex++] = received[rxIndex++];
		}

		termVal[termCharIndex] = '\0';  // terminate string
		double val = atof(termVal);  // update the value
		printMsg("Setting proportional to %f.\r\n", val);
		*P = val;
		++rxIndex;  // go to next potentially valid character
	}

	// read in integral value
	if (rxIndex < numReceived && received[rxIndex] == 'I') {
		// read in characters
		printMsg("Got Integral command\r\n");
		uint8_t termCharIndex = 0;
		char termVal[numReceived + 1];
		++rxIndex;
		while (received[rxIndex] != ' ' && rxIndex < numReceived) {
			termVal[termCharIndex++] = received[rxIndex++];
		}

		termVal[termCharIndex] = '\0';  // terminate string
		double val = atof(termVal);  // update the value
		printMsg("Setting integral to %f.\r\n", val);
		*I = val;
		++rxIndex;  // go to next potentially valid character
	}

	// read in derivative value
	if (rxIndex < numReceived && received[rxIndex] == 'D') {
		// read in characters
		printMsg("Got Derivative command\r\n");
		uint8_t termCharIndex = 0;
		char termVal[rxMax + 1];
		++rxIndex;
		while (received[rxIndex] != ' ' && rxIndex < numReceived) {
			termVal[termCharIndex++] = received[rxIndex++];
		}

		termVal[termCharIndex] = '\0';  // terminate string
		double val = atof(termVal);  // update the value
		printMsg("Setting derivative to %f.\r\n", val);
		*D = val;
	}

	// update pid controller
	uint64_t currMillis = 0;
	vi_get_curr_millis(&currMillis);
	printMsg("Resetting with PID values of P:%f, I:%f, and D%f.\r\n", *P, *I, *D);
	PID_init(0, angvel_z, currMillis, *P, *I, *D, pid);
}

