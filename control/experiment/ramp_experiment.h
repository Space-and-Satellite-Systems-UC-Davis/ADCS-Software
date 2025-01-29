
#ifndef RAMP_EXPERIMENT.H
#define RAMP_EXPERIMENT.H

#include "adcs_math/vector.h"
#include "virtual_intellisat.h"
#include "ramp/ramp.h"

typedef enum {
    RUN_RAMP_EXPERIMENT_SUCCESS,
    RUN_RAMP_EXPERIMENT_FAILURE
} run_ramp_experiment_status;

run_ramp_experiment_status ramp_experiment();

#endif