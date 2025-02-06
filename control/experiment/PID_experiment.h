#ifndef PID_EXPERIMENT_H
#define PID_EXPERIMENT_H

#include "adcs_math/vector.h"
#include "control/PID/PID.h"
#include "virtual_intellisat.h"

typedef enum PID_experiment{
    PID_EXPERIMENT_SUCCESS,
    PID_EXPERIMENT_FAILURE
}PID_experiment_status;

/**@brief Performs an experiement for the PID function.
 *
 * @return PID_status A return code.
 */
PID_status PID_experiment();


#endif//PID_EXPERIMENT_H