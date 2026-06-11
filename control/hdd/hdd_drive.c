/*
 * hdd_drive.c
 *
 *  Created on: Apr 2, 2025
 *      Author: nmain
 */

// #include "../system_config/Timers/timers.h"
#include "control/hdd/hdd_drive.h"

void hddRamp(const vi_sensor channel, const dutyType TARGET_DUTY,
             int8_t doPrint)
{
    if (doPrint) {
        vi_print("Ramping to %u. \r\n", TARGET_DUTY);
    }
    const uint8_t DUTY_STEP = 1;
    const int vi_delay_ms = 50; // time between ramps

    // start at the middle duty + some base speed, then slowly increase
    uint8_t currDuty = vi_hdd_status(channel) + DUTY_STEP;
    if (doPrint) {
        vi_print("Taking first duty step to %u. \r\n", currDuty);
    }
    while (currDuty <= TARGET_DUTY && currDuty <= MAX_DUTY) {
        // set the duty
        vi_hdd_command(channel, currDuty);
        // if (doPrint) { vi_print("Duty cycle: %u \r\n", currDuty); }

        // exit if we just set pwm to the target, otherwise increment
        if (currDuty == TARGET_DUTY) {
            break;
        }
        currDuty += DUTY_STEP;

        // make sure a loop with currDuty == TARGET_DUTY always happens
        if (currDuty > TARGET_DUTY) {
            currDuty = TARGET_DUTY;
        }
        vi_delay_ms(vi_delay_ms);
    }

    if (doPrint) {
        vi_print("Ramped to %u. \r\n", TARGET_DUTY);
    }
}

void hddDrive(const vi_sensor channel, const dutyType TARGET_DUTY,
              int8_t doPrint)
{
    if (TARGET_DUTY <= SLIP_DUTY) {
        if (doPrint) {
            vi_print("Drove to %u without slip handling.\r\n", TARGET_DUTY);
        }
        vi_hdd_command(channel, TARGET_DUTY);
        return;
    }

    // if we would slip going straight to target, ramp instead
    if (doPrint) {
        vi_print("Driving to %u with slip handling.\r\n", TARGET_DUTY);
    }
    vi_hdd_command(channel, SLIP_DUTY);
    rawDutyType currDuty = vi_hdd_status(channel);

    // wait for speed up, then begin ramp
    uint32_t delayTimeMs = currDuty <= MID_DUTY ? SLIP_TIME_MS : JUMP_TIME_MS;
    if (doPrint) {
        vi_print("Delaying for %u with currDuty %u\r\n", delayTimeMs, currDuty);
    }
    vi_delay_ms(delayTimeMs);
    // pwm_setDutyCycle(pwm, TARGET_DUTY);  // try to make next complete jump
    hddRamp(pwm, TARGET_DUTY, doPrint);
}

void hddDriveNB(const vi_sensor channel, const dutyType TARGET_DUTY)
{
    // limit the targeted duty if we would slip trying to immediately jump
    if (TARGET_DUTY > SLIP_DUTY && vi_hdd_status(channel) < SLIP_DUTY) {
        vi_hdd_command(channel, SLIP_DUTY);
        vi_delay_ms(50);
        hddRamp(channel, TARGET_DUTY, 0);
        return;
    }

    vi_hdd_command(channel, TARGET_DUTY);
}
