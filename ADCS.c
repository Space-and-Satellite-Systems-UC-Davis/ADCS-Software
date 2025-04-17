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
#include "control/experiment/PID_experiment.h"
#include "adcs_math/vector.h"


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
                case COILS_TESTING_FAILURE: //noop
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
                case DETUMBLING_FAILURE: //noop
            }
            break;
        case ADCS_HDD_EXP_ANGVEL:
        	vi_print("Running PID experiment! \r\n");
        	vi_print("Called PID_experiment \r\n");
        	vi_hdd_initiate(0);
        	vi_hdd_arm(0, VI_HDD_ARM);
        	while (1){
        		PID_experiment();
        		vi_delay_ms(3000);
        	}
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
    static int iteration = 0; //Starts as true on reboot

    if (iteration == 0) {
        return ADCS_HDD_EXP_ANGVEL;
        // loop through other experiments
    } else if (iteration % 3 == 1) {
        return ADCS_HDD_EXP_TRIAD;
    } else if (iteration % 3 == 2) {
        return ADCS_HDD_EXP_RAMP;
    } else if (iteration % 3 == 0) {
        return ADCS_HDD_EXP_ANGVEL;
    } 

    iteration++;
}
