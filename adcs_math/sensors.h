/**
 * @file sensors.h
 * 
 * @brief Interface for general sensor functions
 * 
 * @author Tarini Maram (tarini.maram@gmail.com) 1/8/2025
*/
#ifndef SENSORS_H
#define SENSORS_H

#include <limits.h>
#include "virtual_intellisat.h"

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
float get_sensor_calibration(vi_sensors sensor, float currValue, float prevValue, float offset, float gain, float filterConstant);

/**
 * @brief safely calculate delta_t accounting for integer overflow
 * 
 * @param currTime current timestamp in milliseconds
 * @param prevTime previous timestamp in milliseconds
 * 
 * @return delta_t change in time
*/
int get_delta_t(int currTime, int prevTime);

#endif

