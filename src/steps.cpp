#include "steps.h"

csch_curr_t steps_proc;
steps_state_t steps_state = STEPS_S_INIT;
uint8_t steps_trackers = 0;

// Last read raw value from the left/right motors
// In range [0, 1023) due to 10-bit ADC
uint16_t _steps_raw_left = 0;
uint16_t _steps_raw_right = 0;
bool _steps_raw_stale = true; // Whether the raw values have been initialized with a reading yet; If true, the first reading will be used to initialize the raw values rather than calculate deltas

// Accumulated deltas between the current and last raw left/right motor values
// Note that these are _NOT_ steps, but rather simply an accumulation of the deltas between raw readings, which can be converted to steps by multiplying by the efficiency coefficient and accounting for calibration
int32_t _steps_ac_left = 0;
int32_t _steps_ac_right = 0;

steps_cal_t _steps_cal = {
    .left_min = 1023,
    .left_max = 0,
    .right_min = 1023,
    .right_max = 0
};
bool _steps_autocal_running = false; // Whether the automatic calibration process is currently running or not

float _steps_coef = 1.0; // Coefficient to adjust observed step counts for efficiency (e.g. slippage)

void steps_csch_tick() {
    switch (steps_state) {
        case STEPS_S_INIT:
            steps_proc = csch_ctask();

            // Ensure pins are setup for reading step counts
            pinMode(STEPS_LEFT_PIN, INPUT);
            pinMode(STEPS_RIGHT_PIN, INPUT);

            csch_cqueue(0); // Queue next tick immediately to start initialization
            steps_state = STEPS_S_IDLE;
            break;

        case STEPS_S_IDLE:
            if (steps_trackers != 0) {
                steps_state = STEPS_S_TRACKING; // Start tracking if there are any trackers
                _steps_raw_stale = true;
                csch_cqueue(0); // Queue next tick immediately to start tracking
            }

            break;

        case STEPS_S_TRACKING: {
            if (steps_trackers == 0) {
                steps_state = STEPS_S_IDLE; // No more trackers, go back to idle state
                break;
            }

            // Poll for readings
            csch_cqueue(STEPS_CSCH_TK_PERIOD);

            // Read raw values from the motors
            uint16_t raw_left = analogRead(STEPS_LEFT_PIN);
            uint16_t raw_right = analogRead(STEPS_RIGHT_PIN);

            // Initialize raw values if they are stale
            if (_steps_raw_stale) {
                _steps_raw_left = raw_left;
                _steps_raw_right = raw_right;
                _steps_raw_stale = false;
            }

            // Accumulate deltas, accounting for wraparound
#if STEPS_LEFT_REV
            _steps_ac_left -= _steps_delta(raw_left, _steps_raw_left, _steps_cal.left_min, _steps_cal.left_max);
#else
            _steps_ac_left += _steps_delta(raw_left, _steps_raw_left, _steps_cal.left_min, _steps_cal.left_max);
#endif
#if STEPS_RIGHT_REV
            _steps_ac_right -= _steps_delta(raw_right, _steps_raw_right, _steps_cal.right_min, _steps_cal.right_max);
#else
            _steps_ac_right += _steps_delta(raw_right, _steps_raw_right, _steps_cal.right_min, _steps_cal.right_max);
#endif

            // Update last raw values
            _steps_raw_left = raw_left;
            _steps_raw_right = raw_right;
            break;
        }

        case STEPS_S_AUTOCAL_1:
            csch_cqueue(0); // Queue next tick immediately to start autocalibration

            if (!_steps_autocal_running) {
                steps_state = STEPS_S_IDLE; // Automatic calibration was cancelled, go back to idle
                break;
            }

            // Initialize autocalibration data here (e.g. reset min/max values)
            _steps_cal.left_min = 1023;
            _steps_cal.left_max = 0;
            _steps_cal.right_min = 1023;
            _steps_cal.right_max = 0;

            steps_state = STEPS_S_AUTOCAL_2;
            break;

        case STEPS_S_AUTOCAL_2: {
            csch_cqueue(0); // Queue next tick immediately to continue autocalibration

            if (!_steps_autocal_running) {
                steps_state = STEPS_S_IDLE; // Automatic calibration was cancelled, go back to idle
                break;
            }

            // Read raw values from the motors
            uint16_t raw_left = analogRead(STEPS_LEFT_PIN);
            uint16_t raw_right = analogRead(STEPS_RIGHT_PIN);

            // Update calibration values based on observed raw readings
            if (raw_left < _steps_cal.left_min) _steps_cal.left_min = raw_left;
            if (raw_left > _steps_cal.left_max) _steps_cal.left_max = raw_left;
            if (raw_right < _steps_cal.right_min) _steps_cal.right_min = raw_right;
            if (raw_right > _steps_cal.right_max) _steps_cal.right_max = raw_right;
            break;
        }

    }
}

void steps_cal(steps_cal_t cal) {
    _steps_cal = cal;
}

void steps_autocal_begin() {
    _steps_autocal_running = true;

    if (steps_state == STEPS_S_IDLE || steps_state == STEPS_S_TRACKING) {
        steps_state = STEPS_S_AUTOCAL_1; // Start automatic calibration if currently idle or tracking
        csch_queue(steps_proc.csch, steps_proc.pid, 0); // Queue next tick immediately to start autocalibration
    }
}

bool steps_autocal_end() {
    if (!_steps_autocal_running) return false;

    if (steps_state == STEPS_S_AUTOCAL_1 || steps_state == STEPS_S_AUTOCAL_2) {
        steps_state = STEPS_S_IDLE; // End automatic calibration and go back to idle
    }

    _steps_autocal_running = false;
    return true;
}

steps_cal_t steps_cal_get() {
    return _steps_cal;
}


bool steps_ready() {
    return steps_state == STEPS_S_TRACKING;
}

void steps_read(int32_t* left, int32_t* right) {
    *left = (int32_t) (_steps_ac_left * _steps_coef * (STEPS_STEPS_PER_REV / (_steps_cal.left_max - _steps_cal.left_min + 1.0)));
    *right = (int32_t) (_steps_ac_right * _steps_coef * (STEPS_STEPS_PER_REV / (_steps_cal.right_max - _steps_cal.right_min + 1.0)));
}

void steps_raw(int16_t* left, int16_t* right) {
    *left = (int16_t) _steps_raw_left;
    *right = (int16_t) _steps_raw_right;
}

void steps_reset(bool left, bool right) {
    if (left) _steps_ac_left = 0;
    if (right) _steps_ac_right = 0;
}

void steps_coef(float coef) {
    _steps_coef = coef;
}

float steps_coef_get() {
    return _steps_coef;
}

void steps_track() {
    if (steps_trackers == 255) return; // Avoid overflow

    if (steps_trackers == 0 && steps_state == STEPS_S_IDLE) {
        // First tracker, start tracking immediately
        steps_state = STEPS_S_TRACKING;
        _steps_raw_stale = true;
        csch_queue(steps_proc.csch, steps_proc.pid, 0); // Queue next tick immediately to start tracking
    }

    steps_trackers++;
}

void steps_untrack() {
    if (steps_trackers == 0) return; // Avoid underflow

    steps_trackers--;

    if (steps_trackers == 0) {
        // No more trackers, go back to idle state
        steps_state = STEPS_S_IDLE;
    }
}

int16_t _steps_delta(uint16_t raw, uint16_t last, uint16_t min, uint16_t max) {
    int16_t range = max - min + 1; // The number of distinct raw values based on calibration
   
    // Clamp values to expected range based on calibration to avoid issues
    if (raw < min) raw = min;
    if (raw > max) raw = max;
    if (last < min) last = min;
    if (last > max) last = max;

    // Adjust values to be in range [0, range] for easier wraparound calculation
    raw -= min;
    last -= min;

    int16_t delta = (int16_t) raw - (int16_t) last;  // Delta (assuming no wraparound)
    int16_t wdelta = range - abs(delta);              // Delta (assuming wraparound)

    // Algorithm
    // Return the smaller of the two deltas (magnitude-wise), including the correct sign

    if (abs(delta) < abs(wdelta)) return delta; // Smaller distance for no wraparound
    return wdelta;                              // Smaller distance for wraparound
}