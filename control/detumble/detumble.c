//
//  detumble.c
//  detumbling
//
//  Created by Chandler Shellabarger on 9/21/24.
//

#define LIMIT 100

#include "control/detumble/detumble.h"
#include "adcs_math/calibration.h"
#include "adcs_math/sensors.h"
#include "adcs_math/vector.h"
#include "control/bdot/bdot_control.h"
#include "control/detumble/detumble_util.h"
#include "virtual_intellisat.h"

#include <math.h>

detumble_status detumble(vec3 needle, bool isTesting, uint64_t maxTime,
                         uint64_t minTime)
{
    // Magnotometer & IMU readings
    vec3 mag_curr = (vec3){ 0, 0, 0 }, mag_prev = (vec3){ 0, 0, 0 };
    vec3 imu_curr = (vec3){ 0, 0, 0 }, imu_prev = (vec3){ 0, 0, 0 };

    // Time varibles
    uint64_t startTime = 0, curr_millis = 0, prev_millis = 0;
    uint64_t delta_t = 0, timeElapsed = 0;

    // Extra varibles
    vec3 mdm;            // Magnetic Dipole Moment
    bool keepDetumbling; // Keep loop running?
    int generation = vi_get_detumbling_generation();

    // Declare varibles for sensor alternation
    vi_sensor magnotometer;
    magnotometer.component = MAG;
    magnotometer.choice =
        sensor_pair_choice(magnotometer, generation) == 1 ? ONE : TWO;

    vi_sensor imu;
    imu.component = IMU;
    imu.choice = sensor_pair_choice(imu, generation) == 1 ? ONE : TWO;

    // Get startTime
    if (vi_get_curr_millis(&curr_millis))
        return DETUMBLING_FAILURE_CURR_MILLIS;
    startTime = curr_millis;

    // Note: May be do something to account for integer overflow
    do {

        // Perform delay for the coil magnetic field decay
        if (vi_control_coil(0, 0, 0))
            return DETUMBLING_FAILURE_CONTROL_COILS;
        if (detumbleDelay(mdm))
            return DETUMBLING_FAILURE_DELAY_MS;

        // Get MAG readings
        mag_prev = mag_curr;
        if (getMag(magnotometer, mag_prev, &mag_curr))
            return DETUMBLING_FAILURE_MAGNOTOMETER;

        // Compute the magetic dipole moment: M = -k(bDot - n)
        mdm = computeMDM(mag_curr, mag_prev, delta_t, needle);

        // Send control command to coils
        if (vi_control_coil(mdm.x, mdm.y, mdm.z))
            return DETUMBLING_FAILURE_CONTROL_COILS;

        // Get IMU readings (A.K.A.: Angular Velocity) for exit condition
        if (getIMU(imu, imu_prev, &imu_curr)) {
            return DETUMBLING_FAILURE_IMU;
        }

        // Keep track of deltaT and timeElapsed
        prev_millis = curr_millis;
        if (vi_get_curr_millis(&curr_millis))
            return DETUMBLING_FAILURE_CURR_MILLIS;
        delta_t = get_delta_t(curr_millis, prev_millis);

        // Decide whether detumbling needs to continue
        timeElapsed = curr_millis - startTime;
        bool isTimeOut = timeElapsed > maxTime;
        bool isTooSoon = timeElapsed < minTime;
        keepDetumbling =
            aboveThreshold(imu_curr, 0.5) && !isTimeOut && isTooSoon;

    } while (isTesting || keepDetumbling);

    // Increment generation on successful execution
    vi_increment_detumbling_generation();

    return DETUMBLING_SUCCESS;
}
