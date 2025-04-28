#include "sensors.h"
#include "math.h"

float lowpass_filter(float currValue, float prevValue, float filterConstant) {
    return ((1-filterConstant) * currValue) + (filterConstant * prevValue);
}

float get_sensor_calibration(vi_sensor sensor, float currValue, float prevValue, float offset, float gain, float filterConstant)
{
    int year, month, day, hour, minute, second;
    vi_get_epoch(&year, &month, &day, &hour, &minute, &second);

    double UTC = julian_date(
        year, month, day,
        hour + minute/60.0 + second/3600.0
    );

    int recent_lookup = 0; //false

    if ((UTC - cache[sensor].last_update_time) <= julian_date(0, 0, 0, 5/60.0)) {
        recent_lookup = 1; //true
    } else {
        cache[sensor].last_update_time = UTC;
    }

    if (recent_lookup) {
        offset = cache[sensor].offset;
        gain = cache[sensor].gain;
        filterConstant = cache[sensor].filterConstant;
    } else {
        vi_get_sensor_calibration(sensor, &offset, &gain, &filterConstant);
        cache[sensor].offset = offset;
        cache[sensor].gain = gain;
        cache[sensor].filterConstant = filterConstant;
    }
 
    return (lowpass_filter(currValue, prevValue, filterConstant) + offset) * gain;
}

uint64_t get_delta_t(uint64_t currTime, uint64_t prevTime) {
    if (prevTime <= currTime) {
        return currTime - prevTime;
    } else {
        return (UINT64_MAX - prevTime) + currTime;
    }
}

