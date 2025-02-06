//
//  detumble.c
//  detumbling
//
//  Created by Chandler Shellabarger on 9/21/24.
//

#define LIMIT 100

#include "control/detumble/detumble.h"
#include "adcs_math/vector.h"
#include "virtual_intellisat.h"
#include "control/bdot/bdot_control.h"
#include "adcs_math/sensors.h"

#include <math.h>
#include <stdbool.h>

/**@brief find the angular velocity through change in magnetic vector
 * 
 * @param b0 Earth's magnectic field vector (relative to satatlite)
 * @param b1 b0 after delta_t
 * @param delta_t the change in time between mag and mag_prev
 *
 * @return angVel anggular velocity
 */
vec3 findAngVel(vec3 b0, vec3 b1, int delta_t)
{
	vec3 bdot; 		//The vector pointing from b0 to b1
	vec3 bVel; 		//The velocity of bdot
	vec3 angVel; 	//The angular velocity 

	if (delta_t == 0) return (vec3){0.0, 0.0, 0.0};
	
	vec_sub(b1, b0, &bdot);
	vec_scalar((1.0 / (double)delta_t), bdot, &bVel);
	vec_scalar((1.0 / vec_mag(b0)), bVel, &angVel);

	//Convert into degree per second
    double RPSmultiplier = (180 / M_PI) * 1000;
    vec_scalar(RPSmultiplier, angVel, &angVel);

	return angVel;
}

/**@brief checks if angular velocity exceeds threshold
 * 
 * @param input angular velocity vector to be examined
 * @param threshold should be self explanatory
 *
 * @return true if threshold has been exceeded
 */
bool aboveThreshold(vec3 input, double threshold)
{
	if(input.x > threshold || input.y > threshold|| input.z > threshold)
		return true;

	return false;
}

detumble_status detumble()
{
	vec3 mag, mag_prev;
	int delta_t;
	vec3 coils_curr;
	int curr_millis = 0, prev_millis = 0;
	int startTime = 0;

	//Get the current time
	if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
		return DETUMBLING_FAILURE;

	startTime = curr_millis;

	//Get current magnetic field reading
	if(vi_get_mag(&(mag.x), &(mag.y), &(mag.z)) == VI_GET_MAG_FAILURE)
			return DETUMBLING_FAILURE;

	//Compute the delta angle 
	vec3 angVel = findAngVel(mag_prev, mag, delta_t);

	//Note: May be do something to account for integer overflow
	while(aboveThreshold(angVel, 0.5) && curr_millis - startTime < LIMIT)
	{
		mag_prev = mag;
		//Get new magnectic field reading
		if(vi_get_mag(&(mag.x), &(mag.y), &(mag.z)) == VI_GET_MAG_FAILURE)
			return DETUMBLING_FAILURE;

		prev_millis = curr_millis;
		//Get the current time
		if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
			return DETUMBLING_FAILURE;
		
		vi_get_mag(&(mag.x), &(mag.y), &(mag.z));

		delta_t = get_delta_t(curr_millis, prev_millis);

		delta_t = curr_millis - prev_millis;
		bdot_control(mag, mag_prev, delta_t, &coils_curr);
		angVel = findAngVel(mag_prev, mag, delta_t);
	}

	return DETUMBLING_SUCCESS;
}
