#include "pointdish.h"

void point_dish(double gs_lat, double gs_lon, double gs_el, vec3 sat_pos_ecef, double *dish_az, double *dish_el){
    double R_total = 6371.0 + gs_el; // Radius of earth + elevation of groundstation

    gs_lat = gs_lat * (M_PI/180.0);
    gs_lon = gs_lon * (M_PI/180.0);

    vec3 gs_pos_ecef;
    vec_set(R_total * cos(gs_lat) * cos(gs_lon),
            R_total * cos(gs_lat) * sin(gs_lon),
            R_total * sin(gs_lat),
            &gs_pos_ecef);

    vec3 diff_vector_ecef; // Can also be thought of as the satellite's position in earth fixed gs centered coordinates
    vec_sub(sat_pos_ecef, gs_pos_ecef, &diff_vector_ecef);

    mat3 E; // Rotation matrix from earth fixed gs centered to local tangent coordinates
            //Tangent coordinates are currently east north zenith but it should be easy to modify to NED if need be.
    mat_set(-sin(gs_lon)            , cos(gs_lon), 0 ,
            -sin(gs_lat)*cos(gs_lon), -sin(gs_lat) * sin(gs_lon), cos(gs_lat),
            cos(gs_lat)*cos(gs_lon) , cos(gs_lat)*sin(gs_lon)   , sin(gs_lat), &E);

    vec3 sat_local_tan;
    mat_vec_mult(E, diff_vector_ecef, &sat_local_tan);

    *dish_az = (180.0/M_PI) * atan(sat_local_tan.x/sat_local_tan.y);
    *dish_el = (180.0/M_PI) * atan(sat_local_tan.z/sqrt(pow(sat_local_tan.x, 2) + pow(sat_local_tan.y, 2)));
}


int satellite_pos(char* tle1, char* tle2, double UTC1, double UTC2, vec3 *output) {
    TLE tle; //TODO: store TLE struct more efficiently

    double realop_position_TEME[3];
    double realop_velocity_TEME[3];
    double realop_position_ITRS[3];


    //Get TEME position vector (kilometers) from TLE.

    parseLines(&tle, tle1, tle2);
    getRVForDate(
            &tle,
            (UTC1+UTC2 - 2440587.5)*86400000.0, //check sgp4/src/c/TLE.h
            realop_position_TEME,
            realop_velocity_TEME
            );

    if (tle.sgp4Error != 0)
        return SGP4_ERROR;

    //Convert TEME vector to ITRS vector (both in meters).

    double MJD = (UTC1+UTC2) - 2400000.5; //for x_p,y_p approximation

    short int status_cel2ter =
        cel2ter(
                UTC1,
                UTC2,
                70, //TODO: update, probably pass as argument from ADCS main
                1,  //Equinox-based so we can ask for equinox of date
                1,  //Reduced accuracy
                1,  //Asking for equinox of date (TEME instead of GCRS)
                0.000005*MJD - 0.1183, //x_p (see footnote)
                0.00001 *MJD - 0.0177, //y_p (    ....    )
                realop_position_TEME,  //Transform from TEME
                realop_position_ITRS   //to ITRS
               );

    if (status_cel2ter != 0)
        return TEME2ITRS_ERROR;

    output->x = realop_position_ITRS[0];
    output->y = realop_position_ITRS[1];
    output->z = realop_position_ITRS[2];

    return POS_LOOKUP_SUCCESS;
}