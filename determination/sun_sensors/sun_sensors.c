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
#include <math.h>
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
    vec_norm(sun_vec_raw,sun_vec);
}


//TODO: Implement Confidence, Convert sun_vec to ECI, determine eclipse threshold
int is_eclipsed(float sensor_readings[NUM_SUN_SENSORS], double longitude, double latitude, double altitude,
                             int year, int month, int day, int hour, int minute,
                             double second)
{
    float confidence = 0.0;
    //Check whether photodiode totals are below a threshold

    float photodiode_sum = 0.0;
    for(int i = 0; i < NUM_SUN_SENSORS; i++)
    {
        photodiode_sum += sensor_readings[i];
    }

    //Check whether the satellite is predicted to be in Earth's shadow
    vec3 sun_vec = (vec3) {0.0,0.0,0.0}; 
    sun_lookup(longitude,latitude, altitude,
                year, month, day, hour, minute,second, &sun_vec);
    //Need to add a conversion to bring sun_vec into ECI frame of reference

    vec3 earth_vec = {0.0,0.0,0.0};
    get_earth_direction(&earth_vec);
    // ASSUMPTION: r_eci is just the opposite of the vector from the satellite to earth
    vec3 r_eci = (vec3) {earth_vec.x * -1, earth_vec.y * -1, earth_vec.z * -1};

    int earth_radius = 6378137;
    double flattening_factor = 1/298.257223563;
    double eccentricity = sqrt(2.0f*flattening_factor-powf(flattening_factor,2));
    double mag_sun=sqrt(powf(sun_vec.x,2) + powf(sun_vec.y,2)+ powf(sun_vec.z,2));
    double mag_sat=sqrt(powf(r_eci.x,2) + powf(r_eci.x,2) + powf(r_eci.z,2));
    sun_vec.z=sun_vec.z/sqrt(1-powf(eccentricity,2));
    r_eci.z=r_eci.z/sqrt(1-powf(eccentricity,2));

    float t_min = (mag_sun*mag_sun - vec_dot(sun_vec,r_eci))/(mag_sun*mag_sun + mag_sat*mag_sat - 2 * vec_dot(sun_vec,r_eci));

    int los = 0; 

    if(t_min < 0 || t_min > 1)
    {
        los = 1;
    }
    else
    {
       double dist_sqrd = ((1.0f-t_min)*mag_sun*mag_sun+vec_dot(sun_vec,r_eci)*t_min)/powf(earth_radius,2);
            if (dist_sqrd > 1 )
            {
                los=1;
            }
            else
            {
                los = 0;
            }
    }


}
