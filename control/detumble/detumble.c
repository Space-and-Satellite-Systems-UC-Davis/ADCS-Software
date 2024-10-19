//
//  detumble.c
//  detumbling
//
//  Created by Chandler Shellabarger on 9/21/24.
//

#include "control/detumble/detumble.h"
#include "adcs_math/vector.h"
#include "virtual_intellisat.h"
#include "control/bdot/bdot_control.h"
#include "adcs_math/sensors.h"

detumble_status detumble()
{
	vec3 mag,mag_prev;
	int delta_t;
	vec3 angvel;
	vec3 coils_curr;
	int curr_millis=0,prev_millis=0;

	int magx,magy,magz;
	vi_get_angvel_status angvel_status;
	vi_get_curr_millis_status time_status;

	time_status = vi_get_curr_millis(&curr_millis);

	if(time_status == GET_CURR_MILLIS_FAILURE)
	{
		return DETUMBLING_FAILURE;
	}

	angvel_status =  vi_get_angvel(&(angvel.x), &(angvel.y), &(angvel.z));

	if(angvel_status == GET_ANGVEL_FAILURE)
	{
		return DETUMBLING_FAILURE;
	}

	vi_get_mag(&(mag.x), &(mag.y), &(mag.z));

	while( 0.5 < angvel.x  || 0.5 < angvel.y || 0.5 < angvel.z)
	{
		mag_prev = mag;
		prev_millis= curr_millis;
		time_status = vi_get_curr_millis(&curr_millis);

		if(time_status == GET_CURR_MILLIS_FAILURE)
		{
			return DETUMBLING_FAILURE;
		}

		angvel_status =  vi_get_angvel(&(angvel.x), &(angvel.y), &(angvel.z));

		if(angvel_status == GET_ANGVEL_FAILURE)
		{
			return DETUMBLING_FAILURE;
		}

		vi_get_mag(&(mag.x), &(mag.y), &(mag.z));

		delta_t = get_delta_t(curr_millis, prev_millis);

		bdot_control(mag, mag_prev, delta_t, &coils_curr);
	}

	return DETUMBLING_SUCCESS;
}
