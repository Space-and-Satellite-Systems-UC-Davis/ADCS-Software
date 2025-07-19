#ifndef PID_EXPERIMENT_H
#define PID_EXPERIMENT_H

#include "adcs_math/vector.h"
#include "control/PID/PID.h"
#include "virtual_intellisat.h"


typedef enum determination_experiment{
    DETERMINATION_EXPERIMENT_SUCCESS,
    DETERMINATION_EXPERIMENT_MILLIS_FAILURE
}determination_exp_status;

/**@brief Performs an experiement for the PID function.
 *
 * @return PID_status A return code.
 */
determination_exp_status determination_experiment();


#endif//PID_EXPERIMENT_H
