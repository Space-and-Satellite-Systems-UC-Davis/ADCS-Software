/**@file sun_sensors/sun_sensors.c
 *
 * @brief Implementation of eclipse detection and sun vector determination
 *
 * @author Soren Keck (sorenkeck@gmail.com)
 * @date 10/16/2025
 */


#define NUM_SUN_SENSORS 6
#define pX 0
#define nX 1
#define pY 2
#define nY 3
#define pZ 4
#define nZ 5

#include "adcs_math/vector.h"
#include "determination/sun_lookup/sun_lookup.h"

vec3 estimate_sun_photodiodes(float sensor_readings[NUM_SUN_SENSORS],vec3 *sun_vec)
{
    float x_reading = 0.0, y_reading = 0.0, z_reading = 0.0;
    //Take either + or - for x,y,z
    if(sensor_readings[pX]>sensor_readings[nX])
    {
        x_reading = sensor_readings[pX];
    }
    else
    {
        x_reading = -sensor_readings[nX];
    }
    if(sensor_readings[pY]>sensor_readings[nY])
    {
        y_reading = sensor_readings[pY];
    }
    else
    {
        y_reading = -sensor_readings[nY];
    }
    if(sensor_readings[pZ]>sensor_readings[nZ])
    {
        z_reading = sensor_readings[pZ];
    }
    else
    {
        z_reading = -sensor_readings[nZ];
    }
    vec3 sun_vec_raw = (vec3){x_reading,y_reading,z_reading};
    //Normalize the vector
    vec_norm(sun_vec_raw,&sun_vec);
}


int is_eclipsed(float sensor_readings[NUM_SUN_SENSORS])
{
    //Check whether photodiode totals are below a threshold

    float photodiode_sum = 0.0;
    for(int i = 0; i < NUM_SUN_SENSORS; i++)
    {
        photodiode_sum += sensor_readings[i];
    }

    //Check whether the dot product of the earth vector and the sun vector is positive
    vec3 sun_vec = sun_lookup() //need to decide whether this vector will be passed or recomputed here

}
