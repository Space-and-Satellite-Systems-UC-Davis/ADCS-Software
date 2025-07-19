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
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum detumble{
    DETUMBLING_SUCCESS,
    COILS_TESTING_SUCCESS,
    COILS_TESTING_FAILURE,
    DET_MAG_FAILURE,
    DET_MILLIS_FAILURE,
    DET_CONTROL_COIL_FAILURE,
    DET_DELAY_MS_FAILURE,
    DET_SENSOR_CALIBRATION_FAILURE_X,
    DET_SENSOR_CALIBRATION_FAILURE_Y,
    DET_SENSOR_CALIBRATION_FAILURE_Z
} detumble_status;

/**@brief find the angular velocity through change in magnetic vector
 * 
 * @param b0 Earth's magnectic field vector (relative to satatlite)
 * @param b1 b0 after delta_t
 * @param delta_t the change in time between mag and mag_prev
 *
 * @return angVel anggular velocity
 */
vec3 findAngVel(vec3 b0, vec3 b1, uint64_t delta_t);

/**@brief convert the coils current into magnetic field magitude
 * 
 * @param current current coils current
 *
 * @return magnetic field magitude
 */
double computeB_coils(double current);

/**@brief compute the time needed for the coil's magnetic field to decay
 * 
 * @const coilInductance the inductance of the coils
 * @const coilResistance the resistance of the coils
 * @const B_Earth the magnetic field of Earth
 * @param B_initial initlian magnetic field of the coils
 * @const decayPercent the desired percentage for the decay
 *
 * @return angVel anggular velocity
 */
double computeDecay(double B_initial);

/**@brief checks if angular velocity exceeds threshold
 * 
 * @param input angular velocity vector to be examined
 * @param threshold should be self explanatory
 *
 * @return true if threshold has been exceeded
 */
bool aboveThreshold(vec3 input, double threshold);

/**@brief checks if current exceeds threshold
 * 
 * @param mdm magnetic dipole moment
 * 
 * @return mdm but capped at 0.158f
 */
void capCurrent(vec3 *mdm);

/**@brief Detumbling function.
 *
 * @param needle a vector to give the detumbling a biase
 * @return detumble_status A return code.
 */
detumble_status detumble(vec3 needle, bool isTesting);

#ifdef __cplusplus
}
#endif

#endif//DETUMBLE_H
