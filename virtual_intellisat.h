/**@file virtual_intellisat.h
 *
 * @brief ADCS Software's interface to Intellisat.
 *
 * This file contains the function prototype for every
 *  actuator-commanding and sensor-reading function
 *  the ADCS Software needs from Intellisat.
 *
 * Their corresponding definitions will be written in the Intellisat
 *  repo by the CS team. While this means any ADCS code that uses
 *  these functions cannot run outside of the Intellisat repo, 
 *  this approach allows for complete separation of the two repos,
 *  which vastly simplifies development for everyone.
 *
 * @author Jacob Tkeio (jacobtkeio@gmail.com)
 * @date 12/30/2023
 */

#ifndef VIRTUAL_INTELLISAT_H
#define VIRTUAL_INTELLISAT_H

#include "ADCS.h"

#include <stdint.h>


/*################ SENSORS AND ACTUATORS ################*/

typedef enum {
    VI_MAG1,
    VI_MAG2
} vi_MAG;

typedef enum {
    VI_IMU1,
    VI_IMU2
} vi_IMU;

typedef enum {
    VI_HDD1,
    VI_HDD2
} vi_HDD;

typedef enum {
	VI_CSS1,
	VI_CSS2
} vi_CSS_PX;


typedef enum {
//Start CSS
	VI_CSS_PX1,
	VI_CSS_PX2,
	
	VI_CSS_NX1,
	VI_CSS_NX2,
	
	VI_CSS_PY1,
	VI_CSS_PY2,

	VI_CSS_NY1,
	VI_CSS_NY2,
	
	VI_CSS_PZ1,
	VI_CSS_PZ2,
	
	VI_CSS_NZ1,
	VI_CSS_NZ2,
//End CSS
//Start MAG
	VI_MAG1_X,
	VI_MAG2_X,
	
	VI_MAG1_Y,
	VI_MAG2_Y,
	
	VI_MAG1_Z,
	VI_MAG2_Z,
//End MAG
//Start IMU
	VI_IMU1_X,
	VI_IMU2_X,
	
	VI_IMU1_Y,
	VI_IMU2_Y,
	
	VI_IMU1_Z,
	VI_IMU2_Z
//End IMU
} vi_sensor;

typedef enum {
	VI_TEMP1_X,
	VI_TEMP2_X,

	VI_TEMP1_Y,
	VI_TEMP2_Y,

	VI_TEMP1_Z,
	VI_TEMP2_Z
} vi_temp_sensor;

typedef enum {
	VI_SP1_X,
	VI_SP2_X,

	VI_SP1_Y,
	VI_SP2_Y
} vi_solar_panel;

/*################## ACTUATOR COMMANDS ##################*/

typedef enum {
    HDD_COMMAND_SUCCESS,
    HDD_COMMAND_FAILURE
} vi_hdd_command_status;

/**@brief Send a throttle command to the HDD.
 *
 * @param hdd Which HDD to command.
 * @param throttle The desired throttle in the range [-1.0, 1.0].
 *
 * Intellisat must check these bounds for the input.
 *
 * Positive throttle must correspond to positive angular
 *  acceleration as defined by the satellite's positive Z axis
 *  and the Right-Hand-Rule.
 *
 * @return vi_hdd_command_status A return code.
 */
vi_hdd_command_status
vi_hdd_command(
    vi_HDD hdd,
    double throttle
);


typedef enum {
	VI_CONTROL_COIL_SUCCESS,
	VI_CONTROL_COIL_FAILURE
} vi_control_coil_status;

/**@brief Set the coils' dipole vector.
 *
 * @param command_x,command_y,command_z The dipole vector.
 *
 * @return vi_control_coil_status A return code, success/failure.
 */
vi_control_coil_status 
vi_control_coil(
	double command_x,
	double command_y,
	double command_z
);


/*################### SENSOR READINGS ###################*/

typedef enum {
    GET_EPOCH_SUCCESS,
    GET_EPOCH_FAILURE
} vi_get_epoch_status;

/**@brief Report the current date and time to second accuracy.
 *
 * @param year,month,day,hour,minute,second Return-by-reference ptrs.
 *
 * @return vi_get_epoch_status A return code.
 */
vi_get_epoch_status
vi_get_epoch(
    int *year,
    int *month,
    int *day,
    int *hour,
    int *minute,
    int *second
);


typedef enum {
    GET_CURR_MILLIS_SUCCESS,
    GET_CURR_MILLIS_FAILURE
} vi_get_curr_millis_status;

/**@brief Report the value of Intellisat's millisecond counter.
 *
 * This will be used to calculate change in time for control loops.
 *
 * @param curr_millis Return-by-reference pointer.
 *
 * @return vi_get_curr_millis_status A return code.
 */
vi_get_curr_millis_status
vi_get_curr_millis(
    uint64_t *curr_millis
);


typedef enum {
    GET_ANGVEL_SUCCESS,
    GET_ANGVEL_FAILURE
} vi_get_angvel_status;

/**@brief Retrieve angular velocity data from the IMU.
 *
 * @param imu Which inertial measurement unit to read from.
 * @param angvel_x,angvel_y,angvel_z Return-by-reference ptrs.
 *
 * The sign of the angular velocity values must adhere to the
 *   Right-Hand-Rule as defined by the satellite's positive axes.
 *
 * @return vi_get_angvel_status A return code.
 */
vi_get_angvel_status
vi_get_angvel(
    vi_IMU imu,
    double *angvel_x, 
    double *angvel_y,
    double *angvel_z
);


typedef enum {
	VI_GET_MAG_SUCCESS,
	VI_GET_MAG_FAILURE
} vi_get_mag_status;

/**@brief Get the current magnetic field value.
 *
 * @param mag Which magnetometer to read from.
 * @param mag_x,mag_y,mag_z The magnetic field vector.
 *
 * @return vi_get_mag_status A return code, success/failure.
 */
vi_get_mag_status
vi_get_mag(
    vi_MAG mag,
	double *mag_x,
	double *mag_y,
	double *mag_z
);


typedef enum {
    VI_GET_CSS_SUCCESS,
    VI_GET_CSS_FAILURE
} vi_get_css_status;

/**@brief Get a coarse sun sensor (CSS) reading.
 *
 * @param css Which CSS to read from.
 * @param magnitude Return-by-reference pointer.
 *
 * The CSS' magnitude would ideally in the range [0, 1]
 *  with 1 corresponding to the sun being directly overhead,
 *  but users should expect the output of this function to
 *  deviations both above and below this range.
 *
 * @return vi_get_css_status A return code, success/failure.
 */
vi_get_css_status
vi_get_css(
    vi_sensor css,
    double *magnitude
);

typedef enum {
    VI_GET_TEMP_SUCCESS,
    VI_GET_TEMP_FAILURE
} vi_get_temp_status;

/**@brief Get a temperature sensor reading.
 *
 * @param sensor Which temperature sensor to read from.
 * @param temp Return-by-reference pointer.
 *
 * @return vi_get_temp_status A return code, success/failure.
 */
vi_get_temp_status 
vi_get_temp(
	vi_temp_sensor sensor, 
	double* temp
);

/**@brief Get a coils current reading.
 *
 * @param currentX, currentY, currentZ Return-by-reference pointer for each of the coils' current
 *
 * @return vi_get_coils_current_status A return code, success/failure.
 */
typedef enum {
    VI_GET_COILS_CURRENT_SUCCESS,
    VI_GET_COILS_CURRENT_FAILURE
} vi_get_coils_current_status;

vi_get_coils_current_status
vi_get_coils_current(
	double* currentX,
	double* currentY, 
	double* currentZ
);

/**@brief Get a solar panel current reading.
 *
 * @param sp Which solar panel to read from.
 * @param current Return-by-reference pointer.
 *
 * @return vi_get_solar_panel_current_status A return code, success/failure.
 */
typedef enum {
    VI_GET_SOLAR_PANEL_CURRENT_SUCCESS,
    VI_GET_SOLAR_PANEL_CURRENT_FAILURE
} vi_get_solar_panel_current_status;

vi_get_solar_panel_current_status
vi_get_solar_panel_current(
	vi_solar_panel sp,
	double* current
);

/*###################### CONSTANTS ######################*/

typedef enum{
	GET_CONSTANT_SUCCESS,
	GET_CONSTANT_FAILURE
} vi_get_constant_status;

/**@brief Get the current calibration values for a sensor.
 *
 * @param sensor we want calibration for.
 * @param offset,scalar Return-by-reference ptrs.
 * @param filter_constant An attenuation constant between
 *  0.0 - 1.0 for lowpass filter calculation. A greater
 *  value causes more damping on large jumps in sensor data.
 *
 * @return vi_get_constant_status A return code, success/failure.
 */
vi_get_constant_status
vi_get_sensor_calibration(
	vi_sensor sensor, 
	float *offset,
	float *scalar,
	float *filter_constant
);


/**@brief Get the current status (on/off) of a given sensor.
 *
 * @param sensor we want calibration for.
 * @param status (boolean) for Return-by-reference pointer.
 *
 * @return vi_get_constant_status A return code, success/failure.
 */
vi_get_constant_status
vi_get_sensor_status(
	vi_sensor sensor,
	int *sensor_status
);


typedef enum{
	GET_TLE_SUCCESS_NEW,
	GET_TLE_SUCCESS_OLD,
	GET_TLE_FAILURE
} vi_get_TLE_status;

/**@brief Get the current TLE.
 *
 * @param TLE line 1 and line 2 Return-by-reference ptrs.
 *
 * @return vi_get_constant_status A return code, success/failure.
 */
vi_get_TLE_status
vi_get_TLE(
	char *tle_line1, 
	char *tle_line2
);


/**@brief Get the experiment generation.
 *
 * @return The generation as an int.
 */
int vi_get_experiment_generation();


/**@brief Increment the experiment generation.
 *
 * @return Void.
 */
void vi_increment_experiment_generation();


/**@brief Get the detumbling generation.
 *
 * @return The generation as an int.
 */
int vi_get_detumbling_generation();


/**@brief Increment the detumbling generation.
 *
 * @return Void.
 */
void vi_increment_detumbling_generation();

/**@brief Get the detumbling generation.
 *
 * @return The generation as an int.
 */
int vi_get_determination_generation();


/**@brief Increment the detumbling generation.
 *
 * @return Void.
 */
void vi_increment_determination_generation();


/*###################### OPERATIONS ######################*/

typedef enum {
    VI_DELAY_MS_SUCCESS,
    VI_DELAY_MS_FAILURE
} vi_delay_ms_status;

/**@brief Sleep for some number of milliseconds.
 *
 * @param ms The number of milliseconds to sleep for.
 *
 * @return vi_delay_ms_status A return code, success/failure.
 */
vi_delay_ms_status
vi_delay_ms(
    int ms
);


/**@brief Print a string.
 *
 * @param string The string to print.
 *
 * @return Void.
 */
void 
vi_print (
  const char *message, ...
);



/**@brief Configure the data logger for a particular mode.
 *
 * @param mode The logger mode setting.
 *
 * @return Void.
 */
void
vi_configure_logging_mode(
    adcs_mode mode
);


#endif//VIRTUAL_INTELLISAT_H
