#include "pos.h"

csch_curr_t pos_proc;
pos_state_t pos_state = POS_S_INIT;
uint8_t pos_trackers = 0;

// Last read position, used to calculate deltas for the next position reading
int32_t _pos_steps_forward = 0;
bool _pos_steps_last_stale = true; // Whether the last position reading has been initialized with a reading yet; If true, the first reading will be used to initialize the last position rather than calculate deltas

float _pos_x = 0;
float _pos_y = 0;

bool _pos_tracking = false;

void pos_csch_tick() {
    switch (pos_state) {
        case POS_S_INIT:
            pos_proc = csch_ctask();

            csch_cqueue(0); // Queue next tick immediately to start initializing
            pos_state = POS_S_IDLE;
            break;

        case POS_S_IDLE:
            if (pos_trackers > 0) {
                // Start tracking immediately if there are already trackers
                pos_state = POS_S_TRACKING;
                csch_queue(pos_proc.csch, pos_proc.pid, 0); // Queue next tick immediately to start tracking
                _pos_steps_last_stale = true;

                if (!_pos_tracking) {
                    dir_track();
                    steps_track();
                    _pos_tracking = true;
                }
            }
            break;

        case POS_S_TRACKING: {
            if (pos_trackers == 0) {
                // No trackers, go back to idle state
                pos_state = POS_S_IDLE;

                if (_pos_tracking) {
                    dir_untrack();
                    steps_untrack();
                    _pos_tracking = false;
                }
                break;
            }

            // Ensure that dir and steps are ready before trying to read from them
            if (!dir_ready() || !steps_ready()) {
                csch_cqueue(POS_WAIT_TK_PERIOD); // Queue next tick after the specified wait period
                break;
            }

            csch_cqueue(POS_CSCH_TK_PERIOD); // Queue next tick after the specified period

            int32_t steps_left, steps_right;

            // Poll for readings
            steps_read(&steps_left, &steps_right);

            int32_t steps_forward = ((steps_left) + (steps_right)) / 2; // Average left/right deltas movement to get forward movement

            // First reading, initialize last steps
            if (_pos_steps_last_stale) {
                _pos_steps_forward = steps_forward;
                _pos_steps_last_stale = false;
                 break;
            }

            if (steps_forward == _pos_steps_forward) break; // No movement, skip rest of loop

            int32_t delta_forward = steps_forward - _pos_steps_forward; // Calculate forward movement since last tick
            _pos_steps_forward = steps_forward; // Update last steps forward with current steps forward for the next tick

            float heading = dir_heading();
            
            // Extract unit vector components from heading
            float heading_x = cosf(heading * PI / 180.0);
            float heading_y = sinf(heading * PI / 180.0);

            // Update position based on movement along the heading direction
            _pos_x += delta_forward * heading_x;
            _pos_y += delta_forward * heading_y;
            break;
        }
    }
}

bool pos_ready() {
    return pos_state == POS_S_TRACKING;
}

void pos_get(float* x, float* y) {
    *x = _pos_x;
    *y = _pos_y;
}

void pos_set(float x, float y) {
    _pos_x = x;
    _pos_y = y;
}

void pos_track() {
    if (pos_trackers == 255) return; // Avoid overflow

    if (pos_trackers == 0 && pos_state == POS_S_IDLE) {

        // First tracker, start tracking immediately
        pos_state = POS_S_TRACKING;
        csch_queue(pos_proc.csch, pos_proc.pid, 0); // Queue next tick immediately to start tracking

        if (!_pos_tracking) {
            dir_track();
            steps_track();
            _pos_tracking = true;
        }
    }

    pos_trackers++;
}

void pos_untrack() {
    if (pos_trackers == 0) return; // Avoid underflow

    pos_trackers--;

    if (pos_trackers == 0 && pos_state == POS_S_TRACKING) {
        // No more trackers, go back to idle state
        pos_state = POS_S_IDLE;

        if (_pos_tracking) {
            dir_untrack();
            steps_untrack();
            _pos_tracking = false;
        }
    }
}
