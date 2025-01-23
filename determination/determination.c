#include "determination/determination.h"
#include "determination/novasc3.1/novas.h"

#include "determination/pos_lookup/pos_lookup.h"
#include "determination/sun_lookup/sun_lookup.h"
#include "determination/mag_lookup/mag_lookup.h"
#include "determination/TRIAD/triad.h"

#include "adcs_math/vector.h"
#include "adcs_math/matrix.h"

#include "virtual_intellisat.h"
#include "ADCS.h"


typedef struct Determination_Cache {
    double last_update_time;

    // for pos_lookup 
    double longitude;
    double latitude;
    double altitude;
    double geocentric_radius;
    double geocentric_latitude;
    pos_lookup_status pos_status;

    vec3 reference_sun;
    sun_lookup_status sun_status;
    vec3 reference_mag;
} determination_cache;

static determination_cache cache;

determination_status
determination(
    int year,
    int month,
    int day,
    int hour,
    int minute,
    double second,
    vec3 measured_mag,
    vec3 measured_sun,
    mat3 *attitude
) {

    double UTC = julian_date(
        year, month, day,
        hour + minute/60.0 + second/3600.0
    );

    int recent_lookup = 0; //false

    if ((UTC - cache.last_update_time) <= julian_date(0, 0, 0, 5/60.0)) {
        recent_lookup = 1; //true
    } else {
        cache.last_update_time = UTC;
    }
    
    int update_IGRF = 0; //false
    char *tle_line1;
    char *tle_line2;

    vi_get_TLE_status tle_status =
    vi_get_TLE(tle_line1, tle_line2);

    switch (tle_status) {
        case GET_TLE_FAILURE: return DET_NO_TLE;
        case GET_TLE_SUCCESS_OLD: break;
        case GET_TLE_SUCCESS_NEW: update_IGRF = 1; break;
        // update IGRF when we get a new TLE, including the 1st time
    }

    double longitude;
    double latitude;
    double altitude;
    double geocentric_radius;
    double geocentric_latitude;
    pos_lookup_status pos_status;

    if (recent_lookup) {
        longitude = cache.longitude;
        latitude = cache.latitude;
        altitude = cache.altitude;
        geocentric_radius = cache.geocentric_radius;
        geocentric_latitude = cache.geocentric_latitude;
        pos_status = cache.pos_status;
    } else {
        pos_status =
        pos_lookup(
            tle_line1,
            tle_line2,
            UTC,
            0.0,
            &longitude,
            &latitude,
            &altitude,
            &geocentric_radius,
            &geocentric_latitude
        );

        cache.pos_status = pos_status;
        cache.longitude = longitude;
        cache.latitude = latitude;
        cache.altitude = altitude;
        cache.geocentric_radius = geocentric_radius;
        cache.geocentric_latitude = geocentric_latitude;
    }

    switch (pos_status) {
        case SGP4_ERROR:            return DET_POS_LOOKUP_ERROR;
        case TEME2ITRS_ERROR:       return DET_POS_LOOKUP_ERROR;
        case ITRS2LLA_ERROR:        return DET_POS_LOOKUP_ERROR;
        case POS_LOOKUP_SUCCESS:    break;
    }


	vec3 reference_sun;
    sun_lookup_status sun_status;

    if (recent_lookup) {
        reference_sun = cache.reference_sun;
        sun_status = cache.sun_status;
    } else {
        sun_status =
        sun_lookup(
            longitude,
            latitude,
            altitude,
            year,
            month,
            day,
            hour,
            minute,
            second,
            &reference_sun
        );

        cache.sun_status = sun_status;
        cache.reference_sun = reference_sun;
    }

    switch (sun_status) {
        case SUN_LOOKUP_BAD_DATE:       return DET_POS_LOOKUP_ERROR;
        case SUN_LOOKUP_BAD_ENVIRONMENT:return DET_POS_LOOKUP_ERROR;
        case SUN_LOOKUP_BAD_LLA:        return DET_POS_LOOKUP_ERROR;
        case SUN_LOOKUP_SUCCESS:        break;
    }


    vec3 reference_mag;

    // Only update IGRF date before recalculating coeffs
    if (update_IGRF) {
        int igrf_time_status =
        igrf_set_date_time(
            year,
            month,
            day,
            hour,
            minute,
            second
        );

        switch (igrf_time_status) {
            //TODO: use 'default' approximate time if out of bounds?
            case IGRF_SET_DATE_OUT_OF_BOUNDS:
                return DET_IGRF_TIME_ERROR;
            case IGRF_SET_DATE_SUCCESS:
                break;
        }
    } 

    if (recent_lookup) {
        reference_mag = cache.reference_mag;
    } else {
        igrf_update(
            geocentric_latitude,
            longitude,
            geocentric_radius,
            update_IGRF, //recalculate coefficients only on new TLE
            &reference_mag
        );
        cache.reference_mag = reference_mag;
    }

    triad_run_status triad_status = 
    triad(
    	measured_sun,
        measured_mag,
    	reference_sun,
        reference_mag,
    	attitude
    );

    switch (triad_status) {
        case TRIAD_NORM_FAILURE:    return DET_TRIAD_ERROR;
        case TRIAD_SUCCESS:         break;
    }

    return DET_SUCCESS;
}





