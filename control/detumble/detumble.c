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

//TODO: IMU, HDD alternation?
#define MAG_CHOICE MAG1

const double control_constant = 67200.0; //TODO: tune :p


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
	vec3 bdot; 		//The velocity vector pointing from b0 to b1
	vec3 angVel; 	//The angular velocity 

	if (delta_t == 0) return (vec3){0.0, 0.0, 0.0};

	bdot_control(b1, b0, delta_t, &bdot);
	vec_scalar((1.0 / vec_mag(b0)), bdot, &angVel);

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

/**@brief checks if current exceeds threshold
 * 
 * @param mdm magnetic dipole moment
 * 
 * @return mdm but capped at 0.158f
 */
void capCurrent(vec3 *mdm)
{
	vec3 coils_output = *mdm;

	//cap output at maximum 0.158 Amps across all coils. 
	double temp_mag = vec_mag(coils_output);
	if (temp_mag > 0.158f) {
		//this step is equivalent to 0.158 * normalized temp.
		vec_scalar(0.158f / temp_mag, coils_output, &coils_output);
	}

	(*mdm) = coils_output;
}

detumble_status detumble(vec3 needle, bool isTesting)
{
	vec3 mag, mag_prev;
	int delta_t = 0;
	vec3 coils_curr;
	int curr_millis = 0, prev_millis = 0;
	int startTime = 0;
	vec3 mdm; //Magnetic Dipole Moment 

	//Get the current time
	if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
    {
	    if (isTesting) return COILS_TESTING_FAILURE;
	    else return DETUMBLING_FAILURE;
    }

	startTime = curr_millis;

	//Get current magnetic field reading
	if(vi_get_mag(MAG_CHOICE, &(mag.x), &(mag.y), &(mag.z)) == VI_GET_MAG_FAILURE)
	{
	    if (isTesting) return COILS_TESTING_FAILURE;
	    else return DETUMBLING_FAILURE;
	}

	//Compute the delta angle 
	vec3 angVel = findAngVel(mag_prev, mag, delta_t);
  
	//Boolean variable to decide if detumbling is needed to continue
	bool keepDetumbling = true;

	//Note: May be do something to account for integer overflow
	while(isTesting || keepDetumbling)
	{
		mag_prev = mag;
		//Get new magnectic field reading
		if(vi_get_mag(MAG_CHOICE, &(mag.x), &(mag.y), &(mag.z)) == VI_GET_MAG_FAILURE)
    	{
			if (isTesting) return COILS_TESTING_FAILURE;
			else return DETUMBLING_FAILURE;
    	}

		prev_millis = curr_millis;
		//Get the current time
		if(vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
    	{
			if (isTesting) return COILS_TESTING_FAILURE;
			else return DETUMBLING_FAILURE;
    	}
		
		if(vi_get_mag(MAG_CHOICE, &(mag.x), &(mag.y), &(mag.z)) == VI_GET_MAG_FAILURE)
    	{
			if (isTesting) return COILS_TESTING_FAILURE;
			else return DETUMBLING_FAILURE;
    	}

		//Compute the delta_t
		delta_t = get_delta_t(curr_millis, prev_millis);
		delta_t = curr_millis - prev_millis;
		
		//M = -k(bDot - n)
		bdot_control(mag, mag_prev, delta_t, &coils_curr);
		vec_sub(coils_curr, needle, &coils_curr);
		vec_scalar(-control_constant, coils_curr, &mdm);

		//Prevent sending too much current to the coils
		//UNLIMITED POWEREERERR
		capCurrent(&mdm);

		//Send control command to coils
		if (vi_control_coil(mdm.x, mdm.y, mdm.z) == VI_CONTROL_COIL_FAILURE)
        {
			if (isTesting) return COILS_TESTING_FAILURE;
			else return DETUMBLING_FAILURE;
        }

		//Compute new angular velocity
		angVel = findAngVel(mag_prev, mag, delta_t);
    
    	//Decide whether detumbling needs to continue
    	int timeElapsed = curr_millis - startTime;
		bool timeout = timeElapsed > LIMIT;
    	keepDetumbling = aboveThreshold(angVel, 0.5) && !timeout;
	}

	return DETUMBLING_SUCCESS;
}
