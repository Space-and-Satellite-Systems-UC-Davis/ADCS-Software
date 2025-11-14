/**@file sun_sensors/sun_sensors.h
 *
 * @brief Functions to determine orientation relative to the sun and eclipse status
 *
 * @author Soren Keck (sorenkeck@gmail.com)
 *
 * @date 10/16/2025
 */

#ifndef SUN_SENSORS.H
#define SUN_SENSORS.H

#define NUM_SUN_SENSORS 6

#include "adcs_math/vector.h"

/**@brief Estimate the sun's position based on the readings from the photodiodes
 *
 * @param sensor_readings The voltage values for one photodiode from each face
 *
 * @return sun_vec the vector pointing towards the sun from the frame of refference of the cube sat
 */
vec3 estimate_sun_photodiodes(float sensor_readings[NUM_SUN_SENSORS]);

bool is_eclipsed(
    float sensor_readings[NUM_SUN_SENSORS]
);

#endif

