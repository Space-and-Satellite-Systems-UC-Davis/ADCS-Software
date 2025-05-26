//
//  detumble.c
//  detumbling
//
//  Created by Chandler Shellabarger on 9/21/24.
//

#define LIMIT 100

#include "control/detumble/detumble.h"
#include "adcs_math/sensors.h"
#include "adcs_math/vector.h"
#include "control/bdot/bdot_control.h"
#include "virtual_intellisat.h"

#include <math.h>

const double control_constant = 67200.0; // TODO: tune :p
const double coilInductance = 1;         // TODO: Messure (Henrys)
const double coilResistance = 1;         // TODO: Measure (Ohms)
const double B_Earth = 1;                // TODO: I need
const double decayPercent = 0.2;         // TODO: Decide on percentage

vec3 findAngVel(vec3 b0, vec3 b1, uint64_t delta_t) {

  vec3 bdot;   // The velocity vector pointing from b0 to b1
  vec3 angVel; // The angular velocity

  if (delta_t == 0)
    return (vec3){0.0, 0.0, 0.0};

  bdot_control(b1, b0, delta_t, &bdot);
  vec_scalar((1.0 / vec_mag(b0)), bdot, &angVel);

  // Convert into degree per second
  double RPSmultiplier = (180 / M_PI) * 1000;
  vec_scalar(RPSmultiplier, angVel, &angVel);

  return angVel;
}

double computeB_coils(double current) { return current; }

double computeDecay(double B_initial) {

  double tau = coilInductance / coilResistance;
  double ratio = B_Earth / B_initial;

  return -1 * tau * log(decayPercent * ratio);
}

bool aboveThreshold(vec3 input, double threshold) {
  if (input.x > threshold || input.y > threshold || input.z > threshold)
    return true;

  return false;
}

void capCurrent(vec3 *mdm) {
  vec3 coils_output = *mdm;

  // cap output at maximum 0.158 Amps across all coils.
  double temp_mag = vec_mag(coils_output);
  if (temp_mag > 0.158f) {
    // this step is equivalent to 0.158 * normalized temp.
    vec_scalar(0.158f / temp_mag, coils_output, &coils_output);
  }

  (*mdm) = coils_output;
}

detumble_status detumble(vec3 needle, bool isTesting) {
  vec3 mag, mag_prev;
  uint64_t delta_t = 0;
  vec3 coils_curr;
  uint64_t curr_millis = 0, prev_millis = 0;
  uint64_t startTime = 0;
  vec3 mdm;                      // Magnetic Dipole Moment
  vi_MAG magnetometer = VI_MAG1; // Initialize it to VI_MAG1
  static int generation = 0;

  // Ger sensor pair choice
  if (sensor_pair_choice(VI_MAG1_X, generation) == 1) {
    magnetometer = VI_MAG1;
  } else {
    magnetometer = VI_MAG2;
  }

  // Get the current time
  if (vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE) {
    if (isTesting)
      return COILS_TESTING_FAILURE;
    else
      return DETUMBLING_FAILURE;
  }

  startTime = curr_millis;

  // Get current magnetic field reading
  if (vi_get_mag(magnetometer, &(mag.x), &(mag.y), &(mag.z)) ==
      VI_GET_MAG_FAILURE) {
    if (isTesting)
      return COILS_TESTING_FAILURE;
    else
      return DETUMBLING_FAILURE;
  }

  // Compute the delta angle
  vec3 angVel = findAngVel(mag_prev, mag, delta_t);

  // Boolean variable to decide if detumbling is needed to continue
  bool keepDetumbling = true;

  // Note: May be do something to account for integer overflow
  while (isTesting || keepDetumbling) {
    prev_millis = curr_millis;
    // Get the current time
    if (vi_get_curr_millis(&curr_millis) == GET_CURR_MILLIS_FAILURE) {
      if (isTesting)
        return COILS_TESTING_FAILURE;
      else
        return DETUMBLING_FAILURE;
    }

    // Set the coil to zero
    if (vi_control_coil(0, 0, 0) == VI_CONTROL_COIL_FAILURE) {
      if (isTesting)
        return COILS_TESTING_FAILURE;
      else
        return DETUMBLING_FAILURE;
    }

    // Compute and perform the delay so that the coil's magnetic field decays
    double coilsMagnetic = computeB_coils(vec_mag(mdm));
    double delayTime = computeDecay(coilsMagnetic);
    if (vi_delay_ms(delayTime) == VI_DELAY_MS_FAILURE) {
      if (isTesting)
        return COILS_TESTING_FAILURE;
      else
        return DETUMBLING_FAILURE;
    }

    // Compute the delta_t
    delta_t = get_delta_t(curr_millis, prev_millis);

    mag_prev = mag;
    // Get new magnectic field reading
    if (vi_get_mag(magnetometer, &(mag.x), &(mag.y), &(mag.z)) ==
        VI_GET_MAG_FAILURE) {
      if (isTesting)
        return COILS_TESTING_FAILURE;
      else
        return DETUMBLING_FAILURE;
    }

    // M = -k(bDot - n)
    bdot_control(mag, mag_prev, delta_t, &coils_curr);
    vec_sub(coils_curr, needle, &coils_curr);
    vec_scalar(-control_constant, coils_curr, &mdm);

    // Prevent sending too much current to the coils (no UNLIMITED POWER)
    capCurrent(&mdm);

    // Send control command to coils
    if (vi_control_coil(mdm.x, mdm.y, mdm.z) == VI_CONTROL_COIL_FAILURE) {
      if (isTesting)
        return COILS_TESTING_FAILURE;
      else
        return DETUMBLING_FAILURE;
    }

    // Compute new angular velocity
    angVel = findAngVel(mag_prev, mag, delta_t);

    // Decide whether detumbling needs to continue
    int timeElapsed = curr_millis - startTime;
    bool timeout = timeElapsed > LIMIT;
    keepDetumbling = aboveThreshold(angVel, 0.5) && !timeout;
  }

  // Increment the generation
  generation++;

  return DETUMBLING_SUCCESS;
}
