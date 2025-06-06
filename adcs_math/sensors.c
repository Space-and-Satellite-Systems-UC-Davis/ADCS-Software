#include "sensors.h"
#include "math.h"

float lowpass_filter(float currValue, float prevValue, float filterConstant) {
    return ((1-filterConstant) * currValue) + (filterConstant * prevValue);
}

float get_sensor_calibration(float currValue, float prevValue, float offset, float scalar, float filterConstant)
{
    return (lowpass_filter(currValue, prevValue, filterConstant) + offset) * scalar;
}


uint64_t get_delta_t(uint64_t currTime, uint64_t prevTime) {
    if (prevTime <= currTime) {
        return currTime - prevTime;
    } else {
        return (UINT64_MAX - prevTime) + currTime;
    }
}

int is_in_eclipse() {
    double px1, px2, nx1, nx2, py1, py2, ny1, ny2, pz1, pz2, nz1, nz2;
    vi_get_css(VI_CSS_PX1, &px1);
    vi_get_css(VI_CSS_PX2, &px2);
    vi_get_css(VI_CSS_NX1, &nx1);
    vi_get_css(VI_CSS_NX2, &nx2);
    vi_get_css(VI_CSS_PY1, &py1);
    vi_get_css(VI_CSS_PY2, &py2);
    vi_get_css(VI_CSS_NY1, &ny1);
    vi_get_css(VI_CSS_NY2, &ny2);
    vi_get_css(VI_CSS_PZ1, &pz1);
    vi_get_css(VI_CSS_PZ2, &pz2);
    vi_get_css(VI_CSS_NZ1, &nz1);
    vi_get_css(VI_CSS_NZ2, &nz2);

    double magnitude = sqrt(pow(px1, 2) + pow(px2, 2) + pow(nx1, 2) + pow(nx2, 2) + pow(py1, 2) + pow(py2, 2) + pow(ny1, 2) + pow(ny2, 2)
                    + pow(pz1, 2) + pow(pz2, 2) + pow(nz1, 2) + pow(nz2, 2));

    if (magnitude <= 0.25) {
        return 1;
    } 
    return 0;
}
