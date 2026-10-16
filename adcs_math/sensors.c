#include "sensors.h"
#include "matrix.h"

#include <math.h>

#define MAX_RETRIES 3

#define SLIP_DUTY 0.7f
#define SLIP_TIME 750
#define STALE_TIME 2000


// Note: sensor calibration is performed in the body frame.
// Ultimately, as long as prevVal and currVal are in the same frame,
//  the calibration routine will work. Our approach is to convert
//  all values to the body frame before applying any calibration.

const vec3 undefined_vec3 = { NAN, NAN, NAN };

const mat3 mag_to_body_transform = {
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0
};

const mat3 imu_to_body_transform = {
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0
};

vi_sensor makeSensor(vi_component component, vi_choice choice, vi_axis axis)
{
    vi_sensor sensor;
    sensor.component = component;
    sensor.choice = choice;
    sensor.axis = axis;
    return sensor;
}

vi_choice selectSensor(vi_sensor sensor, int generation)
{
    // Sensor Alteration
    sensor.choice = sensor_pair_choice(sensor, generation) == 1 ? ONE : TWO;

    // Logic to swap the sensor is the sensor is OFF
    sensor_status status;
    vi_get_sensor_status(sensor, &status);
    if (status == SENSOR_OFF)
        sensor.choice = (sensor.choice == ONE) ? TWO : ONE;

    return sensor.choice;
}

getMag_status getMag(vi_sensor sensor, vec3 prevVal, vec3 *currVal)
{
    int errorCount = 0;

    // Temporarily store mag-frame values in currVal
    while (vi_get_mag(sensor, &(currVal->x), &(currVal->y), &(currVal->z))) {
        errorCount++;
        if (errorCount >= MAX_RETRIES)
            return GET_MAG_FAILURE;
    };

    // Transform the mag-frame values into the satellite's body frame
    mat_vec_mult(mag_to_body_transform, *currVal, currVal);

    // Output calibrated body-frame values to currVal (return-by-reference)
    errorCount = 0;
    while (calibrateVec3(sensor, prevVal, currVal) != CALIBRATION_SUCESS) {
        errorCount++;
        if (errorCount >= MAX_RETRIES)
            return MAG_CALIBRATION_FAILURE;
    }

    return GET_MAG_SUCCESS;
}

getIMU_status getIMU(vi_sensor sensor, vec3 prevVal, vec3 *currVal)
{
    int errorCount = 0; // Local varible to store error occurances

    // Temporarily store imu-frame values in currVal
    while (vi_get_angvel(sensor, &(currVal->x), &(currVal->y), &(currVal->z))) {
        errorCount++;
        if (errorCount >= MAX_RETRIES)
            return GET_IMU_FAILURE;
    };

    // Transform the imu-frame values into the satellite's body frame
    mat_vec_mult(imu_to_body_transform, *currVal, currVal);

    // Output calibrated body-frame values to currVal (return-by-reference)
    errorCount = 0;
    while (calibrateVec3(sensor, prevVal, currVal)) {
        errorCount++;
        if (errorCount >= MAX_RETRIES)
            return IMU_CALIBRATION_FAILURE;
    }

    return GET_IMU_SUCCESS;
}

getCSS_status getCSS(vi_sensor sensor, double prevVal, double *currVal)
{
    int errorCount = 0;
    while (vi_get_css(sensor, currVal)) {
        errorCount++;
        if (errorCount >= 3)
            return GET_CSS_FAILURE;
    }

    errorCount = 0;
    while (calibrateDbl(sensor, prevVal, currVal)) {
        errorCount++;
        if (errorCount >= 3)
            return CSS_CALIBRATION_FAILURE;
    }

    return GET_CSS_SUCCESS;
}

int is_in_eclipse()
{

    // static int iteration = 0;
    //      px1, px2, nx1, nx2, py1, py2, ny1, ny2, pz1, pz2, nz1, nz2;
    double readingsOne[6];
    double readingsTwo[6];
    double prevValOne = NAN, prevValTwo = NAN;

    vi_sensor sensorOne = { CSS, ONE, PX };
    vi_sensor sensorTwo = { CSS, TWO, PX };

    for (int face = PX; face <= NZ; face++) {

        // Set sensor face to current face
        sensorOne.axis = face;
        sensorTwo.axis = face;

        // Get Readings from sensor
        getCSS(sensorOne, prevValOne, &readingsOne[face - 1]);
        getCSS(sensorTwo, prevValTwo, &readingsTwo[face - 1]);
    }

    double sum = 0;
    for (int i = 0; i < 6; i++) {
        sum += pow(readingsOne[i], 2) + pow(readingsTwo[i], 2);
    }

    double magnitude = sqrt(sum);

    // iteration++;

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
    0b00001011, 0b00001111, 0b01100010, 0b00000000
};

vi_choice sensor_pair_choice(vi_sensor sensor, int generation)
{
    int mask = 0;
    switch (sensor.component) {
        case CSS:
            mask = sensor.axis;
            break;

        case MAG:
            mask = 7;
            break;

        case IMU:
            mask = 8;
            break;

        case HDD:
        case TEMP:
        case SOL:
            return 0;
    }

    if ((alternations[generation % 256] | (1 << mask)) != 0) {
        return TWO;
    } else {
        return ONE;
    }
}

/*
From document WEEK 7.5 SM25 HDD MEETING:
```
The HDD, or possibly this specific ESC with HDD's (?), cannot jump to a high
duty from a standstill, or else slipping will occur. To avoid slipping, an
experimentally determined maximum "slip duty" was determined, which can be
jumped to from a standstill. After reaching this duty w/in ~750ms, the HDD can
be ramped up to higher duties. Jumping from a standstill to the slip duty and
then again to the target was not tested, but may work better
```
This is the basis for this function, which aims to implement the 750ms delay.
*/
void hdd_command(double throttle) {
    static uint64_t slip_start_time = 0;
    static uint64_t last_time = 0;

    vi_sensor hdd_one = makeSensor(HDD, ONE, PX);
    vi_sensor hdd_two = makeSensor(HDD, TWO, PX);

    bool positive = (throttle > 0.0);
    throttle = fabs(throttle);

    uint64_t curr_time = 0;
    (void)vi_get_curr_millis(&curr_time);

    // slip_start_time is when we last commanded a throttle < SLIP_DUTY.
    // If slip_start_time is stale, reset it.
    if (throttle <= SLIP_DUTY || get_delta_t(curr_time, last_time) > STALE_TIME)
        slip_start_time = curr_time;

    double slip_throttle;

    // Case 1: throttle < SLIP_DUTY: just use throttle.
    // Case 2: throttle > SLIP_DUTY and it's been less than SLIP_TIME: use SLIP_DUTY.
    // Case 3: throttle > SLIP_DUTY and it's been more than SLIP_TIME: just use throttle.
    // SLIP_TIME is compared to the difference between curr_time and slip_start_time.
    // This is the time since a throttle less than SLIP_DUTY was commanded.
    if (throttle > SLIP_DUTY && get_delta_t(curr_time, slip_start_time) < SLIP_TIME)
        slip_throttle = SLIP_DUTY;
    else
        slip_throttle = throttle;

    if (positive) {
        // Assume hdd_one corresponds with the positive z axis
        vi_hdd_command(hdd_one, slip_throttle);
        vi_hdd_command(hdd_two, 0.0f);
    } else {
        // Assume hdd_two corresponds with the negative z axis
        vi_hdd_command(hdd_one, 0.0f);
        vi_hdd_command(hdd_two, slip_throttle);
    }

    last_time = curr_time;
}
