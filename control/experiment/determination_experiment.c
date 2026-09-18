#include "control/experiment/determination_experiment.h"
#include "adcs_math/calibration.h"
#include "adcs_math/sensors.h"
#include "adcs_math/vector.h"
#include "determination/determination.h"

determination_exp_status determination_experiment()
{
    int generation = vi_get_experiment_generation();
    vi_sensor hdd =
        makeSensor(HDD, selectSensor(makeSensor(HDD, ONE, PZ), generation), PZ);
    mat3 prevAttitude;
    mat3 currAttitude;

    determination(&prevAttitude);

    // Assuming initial angular velocity is 0.
    // This is a fair assumption since we detumble regularly, and
    // our maximum expected angular rate isn't very high anyway.
    int angvel_z = 0;

    // Get the current time (Virtual Intellisat)
    uint64_t prev_millis = 0;
    uint64_t curr_millis = 0;
    if (vi_get_curr_millis(&prev_millis) == GET_CURR_MILLIS_FAILURE)
        return DETERMINATION_EXPERIMENT_MILLIS_FAILURE;

    // Declare and initlialize PID controller
    PID_controller controller;
    double target = 0;
    PID_init(target, angvel_z, prev_millis, 1, 1, 1, &controller);

    // Run a while loop
    while (fabs(target - angvel_z) > 0.1) {
        vi_delay_ms(100);

        vi_enter_critical();
        if (vi_task_has_restarted()) {
            // Return to Schedulers to restart Detumbling
            return DETERMINATION_EXPERIMENT_HAS_RESTARTED;
        }

        determination(&currAttitude);
        // Get the current time (Virtual Intellisat)
        if (vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE)
            return DETERMINATION_EXPERIMENT_MILLIS_FAILURE;

        int delta_t = get_delta_t(curr_millis, prev_millis);

        mat3 derivative;
        mat_sub(currAttitude, prevAttitude, &derivative);
        mat_scalar(1.0 / delta_t, derivative, &derivative);
        double zrotation = derivative.y1;

        // PLug it into the control function
        double throttle =
            PID_command(target, zrotation, curr_millis, &controller);

        // Take output and plug it into HDD
        if (vi_hdd_command(hdd, throttle) == HDD_COMMAND_FAILURE)
            return DETERMINATION_EXPERIMENT_HDD_COMMAND_FAILURE;
        prevAttitude = currAttitude;
        prev_millis = curr_millis;

        vi_exit_critical();
    }

    // Increment generation on successful execution
    vi_increment_experiment_generation();

    return DETERMINATION_EXPERIMENT_SUCCESS;
}
