#include "mpu.h"

mpu_state_t mpu_state = MPU_S_INIT; // State of the MPU state machine
csch_curr_t mpu_proc; // Current process tracking the MPU
uint8_t mpu_trackers = 0; // Number of processes tracking the MPU, up to 255

mpu_cal_t  _mpu_cal = { 0, 0, 0, 0, 0, 0 }; // Calibration data

// Accumulated rotation about each axis, in the range [0, 360)
float _mpu_rot_ac_x = 0;
float _mpu_rot_ac_y = 0;
float _mpu_rot_ac_z = 0;

// Last immediate acceleration along each axis, in the range [-32768, 32767]
int16_t _mpu_acc_x = 0;
int16_t _mpu_acc_y = 0;
int16_t _mpu_acc_z = 0;

// Maximum acceleration along each axis
uint16_t _mpu_acc_max_x = 0;
uint16_t _mpu_acc_max_y = 0;
uint16_t _mpu_acc_max_z = 0;

uint8_t _mpu_autocal_samples; // Number of samples to take (total) during automatic calibration
uint8_t _mpu_autocal_sample_ct; // Number of samples taken so far during automatic calibration
uint16_t mpu_autocal_delay_tk; // Ticks to wait between samples during automatic calibration

int32_t _mpu_cal_ac[6]; // Calibration accumulators

unsigned long _mpu_last_event = 0; // Indicate when the last MPU read event occurred; 0 => no events have occurred yet

void mpu_csch_tick() {
    switch (mpu_state) {
        case MPU_S_INIT:
            mpu_proc = csch_ctask();

            // Configure accelerometer for +/- 2G range
            Wire.beginTransmission(MPU_I2C_ADDR);
            Wire.write(0x1C);
            Wire.write(0x00);

            if (Wire.endTransmission(true) != 0) {
                csch_cqueue(MPU_I2C_TK_TIMEOUT);
                return; // Failed to communicate with MPU, try again later
            }

            // Configure accelerometer for +/- 250 degrees/s gyro range
            Wire.beginTransmission(MPU_I2C_ADDR);
            Wire.write(0x1B);
            Wire.write(0x00);
            Wire.endTransmission(true);

            // Ensure device is not in sleep mode
            Wire.beginTransmission(MPU_I2C_ADDR);
            Wire.write(0x6B);
            Wire.write(0x00);
            Wire.endTransmission(true);

            // PWR_MGMT_1
            Wire.beginTransmission(MPU_I2C_ADDR);
            Wire.write(0x6B);
            Wire.write(0x01); // PLL with gyro X axis
            Wire.endTransmission();

            mpu_state = MPU_S_IDLE;
            csch_cqueue(0); // Queue next step immediately
            break;

        // Do nothing when idle
        case MPU_S_IDLE:
            if (_mpu_autocal_samples != 0) {
                mpu_state = MPU_S_AUTOCAL_1; // Start automatic calibration if it has been requested
                csch_cqueue(0); // Queue next step immediately
                break;
            }

            if (mpu_trackers != 0) {
                mpu_state = MPU_S_TRACKING; // Start tracking the MPU if any processes are tracking it
                csch_cqueue(0); // Queue next reading
                break;
            }
            break;

        case MPU_S_TRACKING: {
            if (mpu_trackers == 0) {
                mpu_state = MPU_S_IDLE; // No processes are tracking the MPU, so stop tracking
                break;
            }

            csch_cqueue(MPU_CSCH_TK_PERIOD); // Queue next reading

            unsigned long curr_time = mpu_proc.csch->curr_time();
            unsigned long delta = curr_time - _mpu_last_event;

            // Read from registers (0x3B - 0x40) for AXH, AXL, AYH, AYL, AZH, AZL
            Wire.beginTransmission(MPU_I2C_ADDR);
            Wire.write(0x3B);
            if (Wire.endTransmission(true) != 0) {
                break;
            }

            Wire.requestFrom(MPU_I2C_ADDR, 14);

            // Failed to read all required data from MPU, try again later
            if (Wire.available() < 14) break;

            // Read+translate accelerometer readings (assumes +/- 2G range)
            _mpu_acc_x = ((int16_t) ((Wire.read() << 8) | Wire.read())) - _mpu_cal.ax;
            _mpu_acc_y = ((int16_t) ((Wire.read() << 8) | Wire.read())) - _mpu_cal.ay;
            _mpu_acc_z = ((int16_t) ((Wire.read() << 8) | Wire.read())) - _mpu_cal.az;

            // Skip temperature range
            Wire.read();
            Wire.read();

            // Read+translate gyro readings (assumes +/- 250 degrees/s range)
            float gyro_x = (((int16_t) ((Wire.read() << 8) | Wire.read())) - _mpu_cal.gx) / 131.0;
            float gyro_y = (((int16_t) ((Wire.read() << 8) | Wire.read())) - _mpu_cal.gy) / 131.0;
            float gyro_z = (((int16_t) ((Wire.read() << 8) | Wire.read())) - _mpu_cal.gz) / 131.0;

            // Update accumulated rotation about each axis, in the range [0, 360)
            // Ignore gyro readings until the first MPU event has occurred
            if (_mpu_last_event != 0) {
                _mpu_rot_ac_x += gyro_x * delta / 1000.0;
                _mpu_rot_ac_y += gyro_y * delta / 1000.0;
                _mpu_rot_ac_z += gyro_z * delta / 1000.0;
            }

            // Wrap accumulated rotation readings to the range [0, 360)
            if (_mpu_rot_ac_x >= 360) _mpu_rot_ac_x -= 360;
            else if (_mpu_rot_ac_x < 0) _mpu_rot_ac_x += 360;
            if (_mpu_rot_ac_y >= 360) _mpu_rot_ac_y -= 360;
            else if (_mpu_rot_ac_y < 0) _mpu_rot_ac_y += 360;
            if (_mpu_rot_ac_z >= 360) _mpu_rot_ac_z -= 360;
            else if (_mpu_rot_ac_z < 0) _mpu_rot_ac_z += 360;

            // Update max acceleration readings
            if (abs(_mpu_acc_x) > _mpu_acc_max_x) _mpu_acc_max_x = abs(_mpu_acc_x);
            if (abs(_mpu_acc_y) > _mpu_acc_max_y) _mpu_acc_max_y = abs(_mpu_acc_y);
            if (abs(_mpu_acc_z) > _mpu_acc_max_z) _mpu_acc_max_z = abs(_mpu_acc_z);

            // Update the time of the last MPU read event
            _mpu_last_event = curr_time;
            break;
        }

        case MPU_S_AUTOCAL_1:
            csch_cqueue(0); // Queue next step immediately

            // No samples to take, go back to idle
            if (_mpu_autocal_samples == 0) {
                mpu_state = MPU_S_IDLE; 
                break;
            }

            // Offsets will be stored in _mpu_cal
            memset(&_mpu_cal_ac, 0, sizeof(_mpu_cal_ac));

            // Reset sample count
            _mpu_autocal_sample_ct = 0;

            mpu_state = MPU_S_AUTOCAL_2;
            // nobreak;

        case MPU_S_AUTOCAL_2:
            csch_cqueue(mpu_autocal_delay_tk); // Queue next sample

            // Finished sampling; calculate average and go back to idle
            if (_mpu_autocal_samples == _mpu_autocal_sample_ct) {
                
                // Push calibration values to _mpu_cal by averaging the accumulated readings over all samples taken
                _mpu_cal.ax = _mpu_cal_ac[0] / _mpu_autocal_samples;
                _mpu_cal.ay = _mpu_cal_ac[1] / _mpu_autocal_samples;
                _mpu_cal.az = _mpu_cal_ac[2] / _mpu_autocal_samples;
                _mpu_cal.gx = _mpu_cal_ac[3] / _mpu_autocal_samples;
                _mpu_cal.gy = _mpu_cal_ac[4] / _mpu_autocal_samples;
                _mpu_cal.gz = _mpu_cal_ac[5] / _mpu_autocal_samples;

                mpu_state = MPU_S_IDLE; // Done with autocal, go back to idle
                _mpu_autocal_samples = 0; // Reset autocal samples to indicate that autocal is no longer running
                break;
            }

            // Read from registers (0x3B - 0x40) for AXH, AXL, AYH, AYL, AZH, AZL
            Wire.beginTransmission(MPU_I2C_ADDR);
            Wire.write(0x3B);
            if (Wire.endTransmission(true) != 0) break; // Error communicating with MPU, try again later
            Wire.requestFrom(MPU_I2C_ADDR, 14);

            // Accumulate readings into _mpu_cal, which will store the average offsets after all samples are taken
            _mpu_cal_ac[0] += ((int16_t) ((Wire.read() << 8) | Wire.read()));
            _mpu_cal_ac[1] += ((int16_t) ((Wire.read() << 8) | Wire.read()));
            _mpu_cal_ac[2] += ((int16_t) ((Wire.read() << 8) | Wire.read()));

            // Skip temperature range
            Wire.read();
            Wire.read();

            // Accumulate gyro readings into _mpu_cal, which will store the average offsets after all samples are taken
            _mpu_cal_ac[3] += ((int16_t) ((Wire.read() << 8) | Wire.read()));
            _mpu_cal_ac[4] += ((int16_t) ((Wire.read() << 8) | Wire.read()));
            _mpu_cal_ac[5] += ((int16_t) ((Wire.read() << 8) | Wire.read()));

            _mpu_autocal_sample_ct++;
    }
}


void mpu_cal(mpu_cal_t cal) {
    _mpu_cal = cal;
}

void mpu_autocal(uint8_t samples, uint16_t delay_tk) {
    if (mpu_state == MPU_S_IDLE || mpu_state == MPU_S_TRACKING) {
        mpu_state = MPU_S_AUTOCAL_1;
        csch_queue(mpu_proc.csch, mpu_proc.pid, 0); // Queue next step immediately
    }

    _mpu_autocal_samples = samples;
    mpu_autocal_delay_tk = delay_tk;
}

mpu_cal_t mpu_cal_get() {
    return _mpu_cal;
}


bool mpu_ready() {
    return mpu_state == MPU_S_TRACKING;
}

void mpu_rot_ac(float *x, float *y, float *z) {
    *x = _mpu_rot_ac_x;
    *y = _mpu_rot_ac_y;
    *z = _mpu_rot_ac_z;
}

void mpu_rot_reset(bool x, bool y, bool z) {
    if (x) _mpu_rot_ac_x = 0;
    if (y) _mpu_rot_ac_y = 0;
    if (z) _mpu_rot_ac_z = 0;
}

void mpu_acc(int16_t *x, int16_t *y, int16_t *z) {
    *x = _mpu_acc_x;
    *y = _mpu_acc_y;
    *z = _mpu_acc_z;
}

void mpu_acc_max(uint16_t *x, uint16_t *y, uint16_t *z) {
    *x = _mpu_acc_max_x;
    *y = _mpu_acc_max_y;
    *z = _mpu_acc_max_z;
}

void mpu_acc_reset(bool x, bool y, bool z) {
    if (x) _mpu_acc_max_x = 0;
    if (y) _mpu_acc_max_y = 0;
    if (z) _mpu_acc_max_z = 0;
}


void mpu_track() {
    if (mpu_trackers == 255) return; // Prevent overflow

    // Queue MPU process to start running again
    if (mpu_trackers == 0 && mpu_state == MPU_S_IDLE) {
        csch_queue(mpu_proc.csch, mpu_proc.pid, 0); // Queue next reading immediately
        mpu_state = MPU_S_TRACKING; // Start tracking the MPU if this is the first tracker
    }

    mpu_trackers++;
}

void mpu_untrack() {
    if (mpu_trackers == 0) return; // Prevent underflows
    
    mpu_trackers--;
    if (mpu_trackers == 0 && mpu_state == MPU_S_TRACKING) {
        mpu_state = MPU_S_IDLE; // Stop tracking the MPU if there are no more trackers
    };
}