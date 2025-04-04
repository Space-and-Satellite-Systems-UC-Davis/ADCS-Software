/**@file detumble.h
 *
 * @brief Detumbling routine for REALOP.
 * 
 * The detumble() function calls virtual_intellisat.h functions to 
 *  get magnetometer and accelerometer measurements, then calls
 *  bdot_control and issues that command to the magnetorquers. It
 *  continues doing this on a loop until the satellite's angular 
 *  velocity is below a required threshold.
 *
 * @author Charles Kvoriak (charles.kvoriak@gmail.com) 10/04/2024
 */

#ifndef DETUMBLE_H
#define DETUMBLE_H
#include "adcs_math/vector.h"

#include <stdbool.h>

typedef enum detumble{
    DETUMBLING_SUCCESS,
    DETUMBLING_FAILURE,
    COILS_TESTING_SUCCESS,
    COILS_TESTING_FAILURE
} detumble_status;


/**@brief Detumbling function.
 *
 * @param needle a vector to give the detumbling a biase
 * @return detumble_status A return code.
 */
detumble_status detumble(vec3 needle, bool isTesting);

#endif//DETUMBLE_H
