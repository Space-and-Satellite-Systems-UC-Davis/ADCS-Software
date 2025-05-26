/**
 * @file sensors.h
 * 
 * @brief Interface for general sensor functions
 * 
 * @author Tarini Maram (tarini.maram@gmail.com) 1/8/2025
*/
#ifndef SENSORS_H
#define SENSORS_H

#include "determination/novasc3.1/novas.h"
#include "virtual_intellisat.h"

#include <limits.h>
#include <stdint.h>


typedef struct Sensor_Calibration {
    double last_update_time; 

    float offset;
    float gain;
    float filterConstant;
} sensor_calibration;

static sensor_calibration* cache;

/**
 * @brief Implement lowpass filter on sensor raw values to mitigate the effect of noise from abnormally high values
 * 
 * @param currValue current sensor raw value
 * @param prevValue previous sensor raw value
 * @param filterConstant constant between 0-1; a greater value is a greater damp on unusually large jumps in sensor data
 * 
 * @return filtered sensor value
*/
float lowpass_filter(float currValue, float prevValue, float filterConstant);

/**
 * @brief Calculates sensor calibration value after filtering data through simple lowpass filter with default constant 0.5
 * 
 * @param sensor type of sensor defined by VI enum
 * @param currValue current sensor raw value
 * @param prevValue previous sensor raw value
 * @param offset constant the sensor reads when it is not supposed to read anything
 * @param gain constant multiplier for sensor raw values
 * @param filterConstant attenuation constant between 0-1 for lowpass filter; a greater value is a greater damp on unusually large jumps in sensor data
 * 
 * @return sensor calibration value after filtration 
*/
float get_sensor_calibration(vi_sensor sensor, float currValue, float prevValue, float offset, float gain, float filterConstant);

/**
 * @brief Safely calculate delta_t accounting for integer overflow
 * 
 * @param currTime current timestamp in milliseconds
 * @param prevTime previous timestamp in milliseconds
 * 
 * @return delta_t change in time
*/
uint64_t get_delta_t(uint64_t currTime, uint64_t prevTime);

/**
 * @brief Generate a permutation of active sensors.
 */
char get_alternation(vi_sensor sensor, unsigned int generation);

/**
 * returns 1 if in eclipse, 0 if not
 * threshold for eclipse is 0 - 0.25 sun sensors reading magnitude
*/
int is_in_eclipse();

/**@brief Return a choice of sensors in a pair for the current alternation.
 *
 * @param sensor the sensor pair to choose between.
 *  Either sensor in the pair (e.g. VI_CSS_PX1 and VI_CSS_PX2) can be
 *  specified here to choose the whole pair.
 * @param generation The generation number of the alternation [0-255].
 *
 * @return 1 or 2 The sensor choice.
 */
int sensor_pair_choice(vi_sensor sensor, int generation);


#endif

