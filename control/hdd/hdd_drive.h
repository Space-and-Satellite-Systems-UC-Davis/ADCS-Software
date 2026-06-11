/*
 * Any functions or code relevant to directly driving the
 * rotation of the hdd
 */

#pragma once

#include "hdd_init.h"
#include "virtual_intellisat.h"

// default ramp
void hddRamp(const vi_sensor channel, const dutyType TARGET_DUTY,
             int8_t doPrint);

// will drive to the targeted duty without slipping; will ramp as needed
void hddDrive(const vi_sensor channel, const dutyType TARGET_DUTY,
              int8_t doPrint);

// tries to avoid slipping while not blocking program execution
void hddDriveNB(const vi_sensor channel, const dutyType TARGET_DUTY);
