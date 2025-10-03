/**
 * @file calibration.h
 *
 * @brief Interface between sensors and control functions
 *
 * @author Chunho Li (lchli@ucdavis.edu) 6/23/2025
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "sensors.h"
#include "adcs_math/vector.h"
#include "virtual_intellisat.h"

#include <stdint.h>

typedef enum getMag{
    GET_MAG_SUCCESS,
    GET_MAG_FAILURE,
    MAG_CALIBRATION_FAILURE
} getMag_status;

/**
 * @brief Retrive sensor data from Virtual Intellisat and perform calibration 
 *
 * @param sensor the sensor to read from 
 * @param prevValue previous sensor value
 * @param currValue the current sensor value to be read 
 */
getMag_status getMag(vi_sensor sensor, vec3 prevVal, vec3 *currVal);

typedef enum getIMU{
    GET_IMU_SUCCESS,
    GET_IMU_FAILURE,
    IMU_CALIBRATION_FAILURE
} getIMU_status;

/**
 * @brief Retrive sensor data from Virtual Intellisat and perform calibration 
 *
 * @param sensor the sensor to read from 
 * @param prevValue previous sensor value
 * @param currValue the current sensor value to be read 
 */
getIMU_status getIMU(vi_sensor sensor, vec3 prevVal, vec3 *currVal);

typedef enum getCSS{
    GET_CSS_SUCCESS,
    GET_CSS_FAILURE,
    CSS_CALIBRATION_FAILURE
} getCSS_status;

/**
 * @brief Retrive sensor data from Virtual Intellisat and perform calibration 
 *
 * @param sensor the sensor to read from 
 * @param prevValue previous sensor value
 * @param currValue the current sensor value to be read 
 */
getCSS_status getCSS(vi_sensor sensor, double prevVal, double *currVal);

#endif