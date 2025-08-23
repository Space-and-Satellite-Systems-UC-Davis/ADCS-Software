#include "determination/pos_lookup/pos_lookup.h"
#include "determination/pos_lookup/sgp4/src/c/TLE.h"
#include "determination/novasc3.1/novas.h"

#include "adcs_math/vector.h"
#include "adcs_math/matrix.h"
#include <math.h>

// Header created not by original dev. 
// Please include actual documentation and delete these remarks

/*
 * WIP
 */
void point_dish(double gs_lat, double gs_lon, double gs_el, vec3 sat_pos_ecef, double *dish_az, double *dish_el);

/*
 * WIP
 */
int satellite_pos(char* tle1, char* tle2, double UTC1, double UTC2, vec3 *output);