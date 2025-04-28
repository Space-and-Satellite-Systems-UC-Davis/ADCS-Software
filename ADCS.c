/**@file ADCS.c
 *
 * @brief Implementation of Intellisat's interface to the ADCS software.
 *
 * @author Jacob Tkeio (jacobtkeio@gmail.com)
 * @date 8/5/2023
 */

#include "ADCS.h"
#include "virtual_intellisat.h"
#include "determination/determination.h"
#include "control/detumble/detumble.h"
#include "adcs_math/vector.h"

#include <stdbool.h>


adcs_main_status
ADCS_MAIN(adcs_mode mode) {
    switch(mode) {
        case ADCS_DETUMBLE:
            switch(detumble((vec3){0,0,0}, false)) {
                case DETUMBLING_FAILURE:
                    return ADCS_MAIN_DETUMBLE_ERR;
                    break;
                case DETUMBLING_SUCCESS:
                    break;

                case COILS_TESTING_SUCCESS: //noop
                    break;
                case COILS_TESTING_FAILURE: //noop
                    break;
            }
            break;
        case ADCS_COILS_TESTING:
            switch(detumble((vec3){0,0,0}, true)) {
                case COILS_TESTING_FAILURE:
                    return ADCS_MAIN_COILS_TESTING_ERR;
                    break;
                case COILS_TESTING_SUCCESS:
                    break;

                case DETUMBLING_SUCCESS: //noop
                    break;
                case DETUMBLING_FAILURE: //noop
                    break;
            }
            break;
        case ADCS_HDD_EXP_ANGVEL:
            break;
        case ADCS_HDD_EXP_TRIAD:
            break;
        case ADCS_HDD_EXP_RAMP:
            break;
        case ADCS_TESTING:
            vi_print("Testing!");
            break;
    }

    return ADCS_MAIN_SUCCESS;
}

adcs_mode ADCS_recommend_mode(){
    static int iteration = 0; //Starts as true on reboot
    adcs_mode mode;

    if (iteration == 0) {
        mode = ADCS_HDD_EXP_ANGVEL;
    // loop through other experiments
    } else if (iteration % 3 == 1) {
        mode = ADCS_HDD_EXP_TRIAD;
    } else if (iteration % 3 == 2) {
        mode = ADCS_HDD_EXP_RAMP;
    } else /*if (iteration % 3 == 0)*/ {
        mode = ADCS_HDD_EXP_ANGVEL;
    } 

    iteration++;
    return mode;
}

int is_in_eclipse() {
    double px1, px2, nx1, nx2, py1, py2, ny1, ny2, pz1, pz2, nz1, nz2;
    vi_get_css(VI_CSS_PX1, &px1);
    vi_get_css(VI_CSS_PX2, &px2);
    vi_get_css(VI_CSS_NX1, &nx1);
    vi_get_css(VI_CSS_NX2, &nx2);
    vi_get_css(VI_CSS_PY1, &py1);
    vi_get_css(VI_CSS_PY2, &py2);
    vi_get_css(VI_CSS_NY1, &ny1);
    vi_get_css(VI_CSS_NY2, &ny2);
    vi_get_css(VI_CSS_PZ1, &pz1);
    vi_get_css(VI_CSS_PZ2, &pz2);
    vi_get_css(VI_CSS_NZ1, &nz1);
    vi_get_css(VI_CSS_NZ2, &nz2);

    double magnitude = sqrt(pow(px1, 2) + pow(px2, 2) + pow(nx1, 2) + pow(nx2, 2) + pow(py1, 2) + pow(py2, 2) + pow(ny1, 2) + pow(ny2, 2)
                    + pow(pz1, 2) + pow(pz2, 2) + pow(nz1, 2) + pow(nz2, 2));

    if (magnitude <= 0.25) {
        return 1;
    } 
    return 0;
}
