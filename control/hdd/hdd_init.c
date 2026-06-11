/*
 * hdd_init.c
 *
 *  Created on: Apr 2, 2025
 *      Author: nmain
 */

#include "control/hdd/hdd_init.h"

// #include <ADC/adc.h>
// #include <LED/led.h>
// #include <Timers/timers.h>
// #include <inttypes.h>
// #include <print_scan.h>
#include <virtual_intellisat.h>

#define RAW_MID_DUTY 50

double duty_to_throttle(rawDutyType duty)
{
    return (duty - RAW_MID_DUTY) / ((double)RAW_MID_DUTY);
}

const dutyType MAX_START_DUTY =
    duty_to_throttle(100); // previous max duty to trigger calibration
const dutyType SLIP_DUTY =
    duty_to_throttle(75); // last valid duty that will not cause any slipping
const dutyType MAX_DUTY =
    duty_to_throttle(100); // targeted current max duty (should be no higher
                           // than 10 for 2ms pulses)
const dutyType MID_DUTY = duty_to_throttle(50);
const dutyType MIN_DUTY = duty_to_throttle(
    0); // targeted current min duty (should be no lower than 5 for 1ms pulses)
const uint32_t SLIP_TIME_MS =
    750; // safe amount of time to wait to go from mid to slip duty
const uint32_t JUMP_TIME_MS =
    250; // safe amount of time to wait to go from >mid to <= slip duty
const int PERIOD_uS = 2000; // period is microseconds (5% duty -> min (1ms
                            // pulse), 10% duty -> max (2ms pulse))

void hdd_init(const vi_sensor channel)
{
    // pwm_initTimer(channel.choice - 1, PERIOD_uS);
    vi_hdd_command(channel, MID_DUTY)
    // pwm_timerOn(channel.choice - 1);
}

void hdd_calibrate(const vi_sensor channel, const int CAL_MAX)
{
    vi_hdd_command(channel, MAX_START_DUTY); // trigger calibration
    vi_print("Min duty: %u, Max duty: %u, Max start duty: %u \r\n", MIN_DUTY,
             MAX_DUTY, MAX_START_DUTY);
    vi_delay_ms(1000);

    if (CAL_MAX) {
        vi_print("Feeding Maximum duty. \r\n");
        vi_hdd_command(channel, MAX_DUTY);
        vi_delay_ms(5000);
    } else {
        vi_print("Feeding minimum duty. \r\n");
        vi_hdd_command(channel, MIN_DUTY);
        vi_delay_ms(5000);
    }
}

void hdd_arm(const vi_sensor channel)
{
    const float DUTY_STEP = (MAX_DUTY - MID_DUTY) / 8;
    float currDuty = MID_DUTY;

    vi_print("Arming with the following parameters: \r\n");
    vi_print("Min duty: %f, Max duty: %f, Zero duty: %f, Duty step: %f \r\n",
             MIN_DUTY, MAX_DUTY, MID_DUTY, DUTY_STEP);

    // The ESC needs to detect some signal, then to drop to minimum
    // the zero/mid duty is just half way between max and minimum,
    // so it should count as some signal. From here, we ramp down to 0
    vi_print("Ramping down. \r\n");
    while (currDuty >= MIN_DUTY) {
        vi_hdd_command(channel, currDuty);
        vi_print("Current duty: %f \r\n", currDuty);
        vi_delay_ms(250);
        currDuty -= DUTY_STEP;
    }

    vi_print("Return to baseline. \r\n");
    vi_print("Setting minimum duty \r\n");
    vi_hdd_command(channel, MIN_DUTY);
    vi_delay_ms(2000);

    vi_print("Setting mid duty \r\n");
    vi_hdd_command(channel, MID_DUTY);
    vi_delay_ms(2000);

    // pwm_setDutyCycle(channel, MID_DUTY);
    // vi_delay_ms(5000);
}
