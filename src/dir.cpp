#include "dir.h"

// -------- INTERNALS --------
float _dir_mag_heading(int16_t x, int16_t y, int16_t z); // Get the current heading, in the range [0, 360) from the raw x/y/z magnetometer readings
float _dir_gyro_heading(float x, float y, float z); // Get the current heading, in the range [0, 360) from the accumulated x/y/z gyro readings

csch_curr_t dir_proc; // Current process tracking the compass
dir_state_t dir_state = DIR_S_INIT; // State of the direction state machine
uint8_t dir_trackers = 0; // Number of processes tracking the compass, up to 255

dir_cal_t _dir_cal = { 0, 1, 0, 1 }; // Calibration data for the compass, which consists of minimum and maximum readings along each axis

// Latest raw magnetometer readings
int16_t _dir_mag_x = 0;
int16_t _dir_mag_y = 0;
int16_t _dir_mag_z = 0;

bool _dir_magnetic = true; // Whether using magnetic readings from the QMC5883L compass (true) or rotational readings from the MPU6050 IMU (false)
bool _dir_autocal_running = false; // Whether the automatic calibration process is currently running or not
bool _dir_using_mpu = false; // Whether we are tracking the MPU module

// Store heading information as raw x/y readings
float _dir_heading = 0; // Absolute heading relative to magnetic north
float _dir_ref_heading = 0; // Reference heading
dir_orient_t _dir_orient = DIR_O_XY; // The orientation of the compass module

void dir_csch_tick() {
    switch (dir_state) {
        case DIR_S_INIT:
            dir_proc = csch_ctask();

            // Configure QMC5883L compass
            Wire.beginTransmission(DIR_I2C_ADDR);
            Wire.write(0x0A);   // CONTROL_1
            Wire.write(0x21);   // OSR=4, ODR=50Hz, MODE=normal
            if (Wire.endTransmission(true) != 0) {
                csch_cqueue(DIR_I2C_TK_TIMEOUT);
                return; // Failed to communicate with compass, try again later
            };

            // Enable SET/RESET pin
            Wire.beginTransmission(DIR_I2C_ADDR);
            Wire.write(0x0B); // CONTROL_2C
            Wire.write(0x0C); // RNG=2G, Set on
            Wire.endTransmission(true);


            dir_state = DIR_S_IDLE;
            csch_cqueue(0); // Queue next step immediately
            break;

        case DIR_S_IDLE:
            if (_dir_autocal_running) {
                dir_state = DIR_S_AUTOCAL_1; // Start automatic calibration if it has been requested
                csch_cqueue(0); // Queue next step immediately
                break;
            }

            if (dir_trackers != 0 && _dir_magnetic) {
                dir_state = DIR_S_TRACKING; // Start tracking the compass if any processes are tracking it
                csch_cqueue(0); // Queue next reading

                // Stop tracking MPU for relative rotational offsets
                if (_dir_using_mpu) {
                    mpu_untrack();
                    _dir_using_mpu = false;
                }

                break;
            }
            break;

        case DIR_S_TRACKING: {
            csch_cqueue(DIR_CSCH_TK_PERIOD); // Queue next step immediately

            if (dir_trackers == 0 || !_dir_magnetic) {
                dir_state = DIR_S_IDLE; // No processes are tracking the compass, so stop tracking

                // Start tracking MPU for relative rotational offsets
                if (!_dir_using_mpu) {
                    mpu_track();
                    _dir_using_mpu = true;
                }
                break;
            }

            Wire.beginTransmission(DIR_I2C_ADDR);
            Wire.write(0x01); // Read from x/y/z data registers starting at 0x01
            Wire.endTransmission(true);
            Wire.requestFrom(DIR_I2C_ADDR, 6);

            // Read+translate compass readings
            _dir_mag_x = Wire.read() | (Wire.read() << 8);
            _dir_mag_y = Wire.read() | (Wire.read() << 8);
            _dir_mag_z = Wire.read() | (Wire.read() << 8);

            _dir_heading = _dir_mag_heading(_dir_mag_x, _dir_mag_y, _dir_mag_z);
            break;
        }

        case DIR_S_AUTOCAL_1:
            csch_cqueue(0); // Queue next step immediately

            if (!_dir_autocal_running) {
                dir_state = DIR_S_IDLE; // Automatic calibration was cancelled, go back to idle
                break;
            }

            // Reset min/max recorded values
            _dir_cal.x_min = 32767;
            _dir_cal.x_max = -32768;
            _dir_cal.y_min = 32767;
            _dir_cal.y_max = -32768;
            _dir_cal.z_min = 32767;
            _dir_cal.z_max = -32768;

            dir_state = DIR_S_AUTOCAL_2;
            break;

        case DIR_S_AUTOCAL_2: {
            csch_cqueue(0); // Queue next reading immediately

            if (!_dir_autocal_running) {
                dir_state = DIR_S_IDLE; // Automatic calibration was cancelled, go back to idle
                break;
            }

            // Read from registers (0x00 - 0x05) for X, Y readings and accumulate min/max readings for each axis
            Wire.beginTransmission(DIR_I2C_ADDR);
            Wire.write(0x00); // STATUS
            Wire.endTransmission(false);

            Wire.requestFrom(DIR_I2C_ADDR, 6);

            _dir_mag_x = Wire.read() | (Wire.read() << 8);
            _dir_mag_y = Wire.read() | (Wire.read() << 8);
            _dir_mag_z = Wire.read() | (Wire.read() << 8);


            // Update min/max recorded values
            if (_dir_mag_x < _dir_cal.x_min) _dir_cal.x_min = _dir_mag_x;
            if (_dir_mag_x > _dir_cal.x_max) _dir_cal.x_max = _dir_mag_x;
            if (_dir_mag_y < _dir_cal.y_min) _dir_cal.y_min = _dir_mag_y;
            if (_dir_mag_y > _dir_cal.y_max) _dir_cal.y_max = _dir_mag_y;
            if (_dir_mag_z < _dir_cal.z_min) _dir_cal.z_min = _dir_mag_z;
            if (_dir_mag_z > _dir_cal.z_max) _dir_cal.z_max = _dir_mag_z;
            break;
        }
    }
}

void dir_cal(dir_cal_t cal) {
    _dir_cal = cal;
}

void dir_autocal_begin() {
    _dir_autocal_running = true;

    if (dir_state == DIR_S_IDLE || dir_state == DIR_S_TRACKING) {
        dir_state = DIR_S_AUTOCAL_1; // Start automatic calibration if currently idle
        csch_queue(dir_proc.csch, dir_proc.pid, 0); // Queue next step immediately
    }
}

bool dir_autocal_end() {
    if (!_dir_autocal_running) return false; // Automatic calibration process was not running, so cannot end it

    if (dir_state == DIR_S_AUTOCAL_1 || dir_state == DIR_S_AUTOCAL_2) {
        dir_state = DIR_S_IDLE; // End automatic calibration and go back to idle
    }

    _dir_autocal_running = false;
    return true;

}

dir_cal_t dir_cal_get() {
    return _dir_cal;
}

bool dir_ready() {
    return dir_state == DIR_S_TRACKING;
}

float dir_heading() {
    float heading = _dir_heading - _dir_ref_heading;

    // Compute angle from accumulated rotation about Z-axis, if required
    if (!_dir_magnetic) {
        float x, y, z;
        mpu_rot_ac(&x, &y, &z);

        // Add accumulated rotation to in-progress heading
        heading += _dir_gyro_heading(x, y, z);
    }

    if (isnanf(heading) || isinff(heading)) return 0; // Prevent invalid heading

    // Clamp output heading to range [0, 360)
    while (heading < 0) heading += 360;
    while (heading >= 360) heading -= 360;

    return heading;
}

void dir_raw(int16_t* x, int16_t* y, int16_t* z) {
    *x = _dir_mag_x;
    *y = _dir_mag_y;
    *z = _dir_mag_z;
}

void dir_ref(float heading) {
    
    if (isinff(heading) || isnanf(heading)) heading = 0; // Prevent invalid reference heading

    // Ensure heading is in range [0, 360)
    while (heading < 0) heading += 360;
    while (heading >= 360) heading -= 360;

    _dir_ref_heading = heading;
}

float dir_ref_get() {
    return _dir_ref_heading;
}

void dir_orient(dir_orient_t orient) {
    _dir_orient = orient;
}

dir_orient_t dir_orient_get() {
    return _dir_orient;
}

void dir_magnet_safe(bool safe) {
    if (safe == _dir_magnetic) return; // No change in magnetic reading safety, so do nothing

    _dir_magnetic = safe;

    if (safe) {
        if (dir_state == DIR_S_IDLE && dir_trackers != 0) {
            dir_state = DIR_S_TRACKING; // Start tracking the compass if currently idle
            csch_queue(dir_proc.csch, dir_proc.pid, 0); // Queue next reading immediately
        }

        if (_dir_using_mpu) {
            mpu_untrack(); // Stop tracking MPU, since we will be using magnetic readings for heading
            _dir_using_mpu = false;
        }

    }
    else {
        mpu_rot_reset(true, true, true); // Reset accumulated rotation readings, which will be used for heading when not using magnetic readings

        // Start tracking MPU if required for heading readings, since we will not be using magnetic readings for heading
        if (!_dir_using_mpu) {
            mpu_track();
            _dir_using_mpu = true;
        }
    }
}

bool dir_magnet_safe_get() {
    return _dir_magnetic;
}

void dir_track() {
    if (dir_trackers == 255) return; // Prevent overflow

    if (dir_trackers == 0) {
        if (dir_state == DIR_S_IDLE && _dir_magnetic) {
            csch_queue(dir_proc.csch, dir_proc.pid, 0); // Queue next reading immediately
            dir_state = DIR_S_TRACKING; // Start tracking the compass if this is the first tracker
        }

        // Need to use MPU for relative rotational offsets
        if (!_dir_using_mpu && !_dir_magnetic) {
            mpu_track(); // Start tracking MPU if required
            _dir_using_mpu = true;
        }
    }

    dir_trackers++;
}

void dir_untrack() {
    if (dir_trackers == 0) return; // Prevent underflow

    dir_trackers--;

    if (dir_trackers == 0) {
        if (dir_state == DIR_S_AUTOCAL_2) {
            dir_state = DIR_S_IDLE; // Stop tracking the MPU if there are no more trackers
        }

        if (_dir_using_mpu) {
            mpu_untrack(); // No longer using MPU readings for heading
            _dir_using_mpu = false;
        }
    }
}



float _dir_mag_heading(int16_t x, int16_t y, int16_t z) {
    float cal_x = (((float) x) - _dir_cal.x_min) / (_dir_cal.x_max - _dir_cal.x_min) * 2.0 - 1.0;
    float cal_y = (((float) y) - _dir_cal.y_min) / (_dir_cal.y_max - _dir_cal.y_min) * 2.0 - 1.0;
    float cal_z = (((float) z) - _dir_cal.z_min) / (_dir_cal.z_max - _dir_cal.z_min) * 2.0 - 1.0;

    float result = 0;
    switch (_dir_orient) {
        case DIR_O_YZ:
            result = atan2f(cal_y, cal_z);
            break;
        case DIR_O_XZ:
            result = atan2f(cal_x, cal_z);
            break;
        case DIR_O_XY:
            result = atan2f(cal_x, cal_y);
            break;
    }

    result *= 180 / 3.14159; // Convert to degrees
    if (result < 0) result += 360; // Ensure result is in range [0, 360)

    return result;
}

float _dir_gyro_heading(float x, float y, float z) {
    switch (_dir_orient) {
        case DIR_O_YZ:
            return x;
        case DIR_O_XZ:
            return y;
        case DIR_O_XY:
            return z;
    }

    // HOW DID YOU GET HERE!?
    return 0;
}
