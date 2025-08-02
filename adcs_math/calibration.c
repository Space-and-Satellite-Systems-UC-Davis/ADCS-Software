#include "adcs_math/vector.h"
#include "virtual_intellisat.h"
#include "calibration.h"

getMag_status getMag(vi_sensor sensor, vec3 prevVal, vec3 *currVal) {

    float sensor_offset, sensor_scalar, sensor_filter_constant;
    vi_MAG_choice choice = sensor.field.mag_choice;
    vec3 mag; // Local varible to store the magnotometer reading

    if (vi_get_mag(choice, &(mag.x), &(mag.y), &(mag.z))) {
        return GET_MAG_FAILURE;
    }

    
    /*################ PROTOTYPE CODE ################*/
    // I think it works but I have noooo idea if it works
    // If it breaks then fall back to BACKUP CODE

    /*

    sensor.field.mag_value = VI_MAG_X1;
    double *magCurrPtr = (double *)&mag;
    double *magPrevPtr = (double *)&prevVal;

    // Perform calibration on each axis
    for (int i = 0; i < 3; i++) {

        if (vi_get_sensor_calibration(sensor, &sensor_offset, &sensor_scalar,
                                      &sensor_filter_constant))
            return MAG_CALIBRATION_FAILURE;

        double currVal = *(magCurrPtr + i);
        double prevVal = *(magPrevPtr + i);
        currVal = get_sensor_calibration(currVal, prevVal, sensor_offset,
                                         sensor_scalar, sensor_filter_constant);
        *(magCurrPtr + i) = currVal;

        sensor.field.mag_value += 2;
    }

    */

    /*################ BACKUP CODE ################*/
    
    sensor.field.mag_value = VI_MAG_X1;
    if (vi_get_sensor_calibration(sensor, &sensor_offset,
                                      &sensor_scalar, &sensor_filter_constant))
            return MAG_CALIBRATION_FAILURE;
    mag.x = get_sensor_calibration(mag.x, prevVal.x, sensor_offset,
                                    sensor_scalar, sensor_filter_constant);

    sensor.field.mag_value = VI_MAG_Y1;
    if (vi_get_sensor_calibration(sensor, &sensor_offset,
                                      &sensor_scalar, &sensor_filter_constant))
            return MAG_CALIBRATION_FAILURE;
    mag.y = get_sensor_calibration(mag.y, prevVal.y, sensor_offset,
                                         sensor_scalar, sensor_filter_constant);

    sensor.field.mag_value = VI_MAG_Z1;
    if (vi_get_sensor_calibration(sensor, &sensor_offset,
                                      &sensor_scalar, &sensor_filter_constant))
            return MAG_CALIBRATION_FAILURE;
    mag.z = get_sensor_calibration(mag.z, prevVal.z, sensor_offset,
                                         sensor_scalar, sensor_filter_constant);

    return GET_MAG_SUCCESS;
}

getIMU_status getIMU(vi_sensor sensor, vec3 prevVal, vec3 *currVal) {

    float sensor_offset, sensor_scalar, sensor_filter_constant;
    vi_IMU_choice choice = sensor.field.imu_choice;
    vec3 reading; // Local varible to store sensor

    if (vi_get_angvel(choice, &(reading.x), &(reading.y), &(reading.z))) {
        return GET_IMU_FAILURE;
    }

    sensor.field.imu_value = VI_IMU1_X;
    if (vi_get_sensor_calibration(sensor, &sensor_offset, &sensor_scalar,
                                  &sensor_filter_constant))
        return IMU_CALIBRATION_FAILURE;
    reading.x = get_sensor_calibration(reading.x, prevVal.x, sensor_offset,
                                       sensor_scalar, sensor_filter_constant);

    sensor.field.imu_value = VI_IMU1_Y;
    if (vi_get_sensor_calibration(sensor, &sensor_offset, &sensor_scalar,
                                  &sensor_filter_constant))
        return IMU_CALIBRATION_FAILURE;
    reading.y = get_sensor_calibration(reading.y, prevVal.y, sensor_offset,
                                       sensor_scalar, sensor_filter_constant);

    sensor.field.imu_value = VI_IMU1_Z;
    if (vi_get_sensor_calibration(sensor, &sensor_offset, &sensor_scalar,
                                  &sensor_filter_constant))
        return IMU_CALIBRATION_FAILURE;
    reading.z = get_sensor_calibration(reading.z, prevVal.z, sensor_offset,
                                       sensor_scalar, sensor_filter_constant);

    return GET_IMU_SUCCESS;
}

getCSS_status getCSS(vi_sensor sensor, double prevVal, double *currVal) {

    float sensor_offset, sensor_scalar, sensor_filter_constant;
    vi_CSS_choice choice = sensor.field.css_choice;
    double reading; // Local varible to store sensor

    if (vi_get_css(choice, &reading)) {
        return GET_CSS_FAILURE;
    }

    sensor.field.css_value = choice;
    if (vi_get_sensor_calibration(sensor, &sensor_offset, &sensor_scalar,
                                  &sensor_filter_constant))
        return CSS_CALIBRATION_FAILURE;
    reading = get_sensor_calibration(reading, prevVal, sensor_offset,
                                       sensor_scalar, sensor_filter_constant);


    return GET_CSS_SUCCESS;
}
