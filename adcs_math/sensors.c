#include "sensors.h"

#include "math.h"

float lowpass_filter(float currValue, float prevValue, float filterConstant) {
    return ((1 - filterConstant) * currValue) + (filterConstant * prevValue);
}

float get_sensor_calibration(vi_sensor sensor, float currValue, float prevValue,
                             float offset, float gain, float filterConstant) {
    int year, month, day, hour, minute, second;
    vi_get_epoch(&year, &month, &day, &hour, &minute, &second);

    double UTC =
        julian_date(year, month, day, hour + minute / 60.0 + second / 3600.0);

    int recent_lookup = 0; // false

    if ((UTC - cache[sensor].last_update_time) <=
        julian_date(0, 0, 0, 5 / 60.0)) {
        recent_lookup = 1; // true
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

    return (lowpass_filter(currValue, prevValue, filterConstant) + offset) *
           gain;
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

    double magnitude =
        sqrt(pow(px1, 2) + pow(px2, 2) + pow(nx1, 2) + pow(nx2, 2) +
             pow(py1, 2) + pow(py2, 2) + pow(ny1, 2) + pow(ny2, 2) +
             pow(pz1, 2) + pow(pz2, 2) + pow(nz1, 2) + pow(nz2, 2));

    if (magnitude <= 0.25) {
        return 1;
    }
    return 0;
}

static const char alternations[256] = {
    0b01100111, 0b01110101, 0b10000000, 0b11101011, 0b01100100, 0b11111111,
    0b11011100, 0b10100100, 0b01011011, 0b01010111, 0b01010101, 0b00010000,
    0b00100100, 0b11100011, 0b00110110, 0b00011001, 0b11101010, 0b10100111,
    0b10000011, 0b01010011, 0b10001100, 0b10100010, 0b11100110, 0b11101100,
    0b11010011, 0b10111001, 0b00001001, 0b11010010, 0b00100000, 0b01111110,
    0b01010010, 0b11100010, 0b01010001, 0b11101000, 0b10000110, 0b10101100,
    0b00111111, 0b10010000, 0b10110101, 0b10100000, 0b00100110, 0b10011000,
    0b11010111, 0b11000010, 0b11010110, 0b01001110, 0b01111001, 0b00111011,
    0b01001100, 0b01000100, 0b00011110, 0b01110000, 0b11000001, 0b00110101,
    0b00101010, 0b01101110, 0b01110100, 0b10111010, 0b01101111, 0b00101001,
    0b10010101, 0b00110010, 0b11010100, 0b01000101, 0b10110000, 0b10010110,
    0b00011011, 0b00110001, 0b11100101, 0b10010100, 0b11100000, 0b10111000,
    0b11111110, 0b11111000, 0b11001010, 0b00110000, 0b01111010, 0b11110100,
    0b10011110, 0b11101110, 0b10101111, 0b00011000, 0b00001010, 0b10100011,
    0b11011010, 0b00111100, 0b01110111, 0b00011111, 0b00001110, 0b00111010,
    0b00011101, 0b11110011, 0b11001111, 0b00010011, 0b01101101, 0b10101110,
    0b01100110, 0b10011101, 0b00011100, 0b10101011, 0b00100101, 0b00101011,
    0b00010100, 0b10011011, 0b00000101, 0b01010000, 0b00010010, 0b00111101,
    0b01011111, 0b01011010, 0b00101110, 0b11011101, 0b01111101, 0b10001011,
    0b10101000, 0b10100001, 0b10110011, 0b11111101, 0b11011111, 0b11000111,
    0b00000111, 0b11110010, 0b01110010, 0b01101100, 0b10000100, 0b10101001,
    0b10100101, 0b01111011, 0b10000111, 0b10010001, 0b01111100, 0b10110111,
    0b01011000, 0b01011101, 0b01100101, 0b01101000, 0b10000101, 0b10001111,
    0b10010011, 0b10101010, 0b00111000, 0b01000011, 0b10101101, 0b11001011,
    0b10110001, 0b01010110, 0b01001111, 0b10111111, 0b11000100, 0b01100011,
    0b11100100, 0b01100001, 0b00101000, 0b01001001, 0b01111000, 0b11101001,
    0b00111001, 0b01100000, 0b11111100, 0b11000000, 0b10000001, 0b00010111,
    0b00010101, 0b11000101, 0b00011010, 0b10111011, 0b00110011, 0b11011110,
    0b11111011, 0b11110111, 0b10111100, 0b11011000, 0b00100111, 0b10001101,
    0b01110110, 0b01101011, 0b01101001, 0b01000010, 0b01001101, 0b11010101,
    0b01001000, 0b10011010, 0b00101101, 0b11111010, 0b11100111, 0b10001000,
    0b11011001, 0b00101100, 0b00100001, 0b10010111, 0b01111111, 0b11000011,
    0b10010010, 0b01011001, 0b00010110, 0b01011100, 0b11001100, 0b11110000,
    0b01101010, 0b01110011, 0b11111001, 0b01000111, 0b00110100, 0b01001010,
    0b11001110, 0b01000001, 0b11101111, 0b00100010, 0b10001001, 0b10111101,
    0b10110100, 0b11010000, 0b01110001, 0b10100110, 0b10110110, 0b10000010,
    0b00000011, 0b01000110, 0b11011011, 0b11001000, 0b01001011, 0b10001010,
    0b11100001, 0b01000000, 0b00010001, 0b01011110, 0b10110010, 0b11110001,
    0b11001101, 0b00001101, 0b00001100, 0b11110110, 0b00101111, 0b01010100,
    0b11101101, 0b11110101, 0b00000001, 0b00000010, 0b00001000, 0b00100011,
    0b10001110, 0b11000110, 0b11001001, 0b11010001, 0b00111110, 0b10011111,
    0b00000100, 0b10011001, 0b00110111, 0b10011100, 0b10111110, 0b00000110,
    0b00001011, 0b00001111, 0b01100010, 0b00000000};

int sensor_pair_choice(vi_sensor sensor, int generation) {
    switch (sensor) {
    case VI_CSS_PX1:
    case VI_CSS_PX2:
        int mask = 1;
    case VI_CSS_NX1:
    case VI_CSS_NX2:
        int mask = 2;
    case VI_CSS_PY1:
    case VI_CSS_PY2:
        int mask = 3;
    case VI_CSS_NY1:
    case VI_CSS_NY2:
        int mask = 4;
    case VI_CSS_PZ1:
    case VI_CSS_PZ2:
        int mask = 5;
    case VI_CSS_NZ1:
    case VI_CSS_NZ2:
        int mask = 6;
    case VI_MAG1_X:
    case VI_MAG2_X:
    case VI_MAG1_Y:
    case VI_MAG2_Y:
    case VI_MAG1_Z:
    case VI_MAG2_Z:
        int mask = 7;
    case VI_IMU1_X:
    case VI_IMU2_X:
    case VI_IMU1_Y:
    case VI_IMU2_Y:
    case VI_IMU1_Z:
    case VI_IMU2_Z:
        int mask = 8;
    }


    if ((alternations[generation % 256] | (1 << mask)) != 0) {
        return 2;
    } else {
        return 1;
    }
}
