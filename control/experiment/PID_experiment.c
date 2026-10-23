#include "PID_experiment.h"
#include "adcs_math/sensors.h"
#include "virtual_intellisat.h"
#include "virtual_rtos.h"

#include <math.h>
#include <stdint.h>

#define P_GAIN 0.4
#define I_GAIN 0
#define D_GAIN 0.1

PID_status PID_experiment_helper(double target, uint64_t timeout)
{

    vi_sensor imu = makeSensor(IMU, ONE, PX);

    vec3 angVel_curr = undefined_vec3;
    vec3 angVel_prev = undefined_vec3;

    uint64_t start_millis = 0;
    uint64_t curr_millis = 0;

    // Get current generation for sensor alternation
    int generation = vi_get_experiment_generation();

    // Get IMU sensor choice
    imu.choice = selectSensor(imu, generation);

    // Verify experiment is running

    double throttle = 0;

    // Get the current angular velocity (angVel_prev = undef -> no filtering)
    if (getIMU(imu, angVel_prev, &angVel_curr))
        return PID_EXPERIMENT_ANGVEL_FAILURE;

    // Get the current time (Virtual Intellisat)
    if (vi_get_curr_millis(&start_millis))
        return PID_EXPERIMENT_MILLIS_FAILURE;
    curr_millis = start_millis;

    // Declare and initlialize PID controller
    PID_controller controller;
    PID_init(target, angVel_curr.z, curr_millis, P_GAIN, I_GAIN, D_GAIN,
             &controller);

    // Run a while loop
    while (get_delta_t(curr_millis, start_millis) < timeout) {

        vi_enter_critical();
        if (vi_task_has_restarted()) {
            // Return to Schedulers to restart Detumbling
            return PID_EXPERIMENT_HAS_RESTARTED;
        }

        if (getIMU(imu, angVel_prev, &angVel_curr))
            return PID_EXPERIMENT_ANGVEL_FAILURE;
        // Get the current time (Virtual Intellisat)
        if (vi_get_curr_millis(&curr_millis))
            return PID_EXPERIMENT_MILLIS_FAILURE;

        // PLug it into the control function
        throttle = PID_command(target, angVel_curr.z, curr_millis, &controller);

        // Clamp the output
        if (throttle > 1.0)
            throttle = 1.0;
        if (throttle < -1.0)
            throttle = -1.0;

        // Take output and plug it into HDD
        hdd_command(throttle);

        angVel_prev = angVel_curr;
        vi_exit_critical();
    }

    // Increment generation on successful execution
    vi_increment_experiment_generation();

    return PID_EXPERIMENT_SUCCESS;
}

PID_status PID_experiment(int infinite)
{
    if (infinite)
        return PID_experiment_helper(.0872665, UINT64_MAX);

    for (float target = .0872665; target > 0.0; target /= 2.0) {
        PID_status s = PID_experiment_helper(target, 4000);
        if (s != PID_EXPERIMENT_SUCCESS)
            return s;

        s = PID_experiment_helper(-target, 4000);
        if (s != PID_EXPERIMENT_SUCCESS)
            return s;
    }

    return PID_EXPERIMENT_SUCCESS;
}
