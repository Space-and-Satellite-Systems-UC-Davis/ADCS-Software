#include "sensors.h"

float lowpass_filter(float currValue, float prevValue, float filterConstant) {
    return ((1-filterConstant) * currValue) + (filterConstant * prevValue);
}

float get_sensor_calibration(vi_sensors sensor, float currValue, float prevValue, float offset, float gain, float filterConstant)
{
    vi_get_sensor_calibration(sensor, &offset, &gain, &filterConstant);
    return (lowpass_filter(currValue, prevValue, filterConstant) + offset) * gain;
}

int get_delta_t(int currTime, int prevTime) {
    if (prevTime <= currTime) {
        return currTime - prevTime;
    } else {
        return (INT_MAX - prevTime) + currTime;
    }
}
