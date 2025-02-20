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


adcs_main_status
ADCS_MAIN(adcs_mode mode) {
    switch(mode) {
        case ADCS_DETUMBLE:
            detumble_status detumble_status = detumble();
            switch(detumble_status) {
                case DETUMBLING_FAILURE:
                    return ADCS_MAIN_DETUMBLE_ERR;
                    break;
                case DETUMBLING_SUCCESS:
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
}

adcs_mode ADCS_recommend_mode(){
    static int first_time = 1; //Starts as true on reboot

    if (first_time) {
        return ADCS_HDD_EXP_ANGVEL;
    } else {
        // loop through other experiments
    }
}
