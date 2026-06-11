/*
 * hdd_training_test.c
 *
 *  Fill out the information below
 *  Created on: 2025-07-22
 *  Updated on: 2025-07-22
 *      Author: Nicholas Bianez
 *
 *
 *  Information:
 *  	This file is designate for personal testing
 *  	to the active HDD member with ID 1.
 */

#include "../hdd_drive.h"
#include "../hdd_init.h"
#include "adcs_math/sensors.h"

// #include <Timers/timers.h>
// #include <ADC/adc.h>
// #include <LED/led.h>
// #include <inttypes.h>
// #include <print_scan.h>

// runs the actual testing code
void testFunction_HDD1()
{
    vi_print("Starting HDD1 test function execution.\r\n");

    // init timer here does other initialization actions
    const vi_sensor targetPWM = makeSensor(HDD, ONE, PX);
    const vi_sensor secondPWM = makeSensor(HDD, TWO, NX);
    vi_print("Testing HDD%d\r\n", targetPWM);
    // led_d2(1);
    // led_d3(0);
    hdd_init(targetPWM);
    hdd_init(secondPWM);
    // pwm_initTimer(PWM0, PERIOD_uS); //This period time is in microseconds
    // vi_hdd_command(PWM0, 10);
    // pwm_timerOn(PWM0);

    const float DUTY_STEP = 0.5;

    // calibrate or arm
    if (0) {
        // calibration controls what PWM duty cycle the
        // ESC considers as the maximums and minimums
        vi_print("Calibrating max duty. \r\n");
        hdd_calibrate(targetPWM, 1);

        vi_hdd_command(targetPWM, MID_DUTY);

        vi_print("Calibrating min duty in 3 seconds. \r\n");
        hdd_calibrate(targetPWM, 0);

        vi_print("Calibration completed; 3 seconds until calibration concludes "
                 "\r\n.");
        vi_delay_ms(3000);
        vi_print("Continuing. \r\n");
    }

    // hdd_calibrate(targetPWM, 1);
    hdd_arm(targetPWM);
    // hdd_arm(secondPWM);

    vi_print("Testing max duty \r\n");
    vi_hdd_command(targetPWM, MAX_DUTY);
    vi_delay_ms(30000);
    return;

    vi_print("Finished arming; throttling \r\n");
    vi_hdd_command(targetPWM, 90);
    vi_hdd_command(secondPWM, MAX_DUTY);
    vi_delay_ms(10000);

    vi_hdd_command(secondPWM, 75);
    vi_print("Testing slipping.\r\n");
    vi_hdd_command(targetPWM, SLIP_DUTY);
    vi_delay_ms(5000);
    vi_hdd_command(targetPWM, MID_DUTY);
    vi_delay_ms(5000);

    // led_d2(0);
    // led_d3(1);
    vi_print("Testing driving.\r\n");
    hddDrive(targetPWM, 95, 1);
    uint8_t duty = vi_hdd_status(targetPWM);
    vi_print("Set duty to %u and got duty %u\r\n", 95, duty);
    vi_delay_ms(3000);

    // led_d2(1);
    // led_d3(1);
    vi_print("Starting Execution. \r\n");
    vi_hdd_command(targetPWM, 100);
    // ramp(DRIVE_DUTY, MIN_DUTY, MAX_DUTY);
    vi_delay_ms(1000);

    vi_print("Testing minimum duty \r\n");
    vi_hdd_command(targetPWM, MIN_DUTY);
    vi_delay_ms(5000);

    vi_print("Testing mid duty \r\n");
    vi_hdd_command(targetPWM, MID_DUTY);
    vi_delay_ms(5000);

    vi_print("Testing max duty \r\n");
    vi_hdd_command(targetPWM, MAX_DUTY);
    vi_delay_ms(5000);

    // led_d2(0);
    // led_d3(1);
    // debug loop
    while (0) {
        // rapidly increase until max is hit
        vi_print("Preparing to start trial. \r\n");
        float currDuty = MIN_DUTY;
        while (currDuty < MID_DUTY) {
            vi_hdd_command(targetPWM, currDuty);
            vi_print("Duty: %f \r\n", currDuty);
            vi_delay_ms(500);
            currDuty += DUTY_STEP;
        }

        vi_print("Waiting to up duty cycle. \r\n");
        vi_delay_ms(5000);
        while (currDuty <= MAX_DUTY) {
            vi_hdd_command(targetPWM, currDuty);
            vi_print("Duty: %f \r\n", currDuty);
            vi_delay_ms(500);
            currDuty += DUTY_STEP;
        }

        vi_delay_ms(5000);

        break;
    }

    // led_d2(0);
    // led_d3(0);

    vi_print("Ending program. \r\n");
    // pwm_timerOff(targetPWM);
}
