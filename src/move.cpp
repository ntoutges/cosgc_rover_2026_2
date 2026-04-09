#include "move.h"

csch_curr_t move_proc;
move_state_t move_state = MOVE_S_INIT;

move_mstate_t _move_motor_left;
move_mstate_t _move_motor_right;
bool _move_planning = false; // Whether we are currently in the process of planning a move, which inhibits all movement actions until plan mode is deactivated

float _move_accel = 0; // Acceleration profile for movement, in steps (mm) per second squared; 0 => instant acceleration

// PID constants for movement control
int32_t _move_kp = 1; // 1 PWM per step/s of speed error
int32_t _move_ki = 1; // Integral correction for steady-state offset
int32_t _move_kd = 0; // Derivative off by default (noisy with cheap encoders)

uint8_t _move_min_pwm = MOVE_MIN_PWM;
int32_t _move_sync_kp = MOVE_SYNC_KP;
float _move_target_heading = 0; // Target heading for MOVE_S_ROTATE_TO, in degrees [0, 360)
float _move_heading_tol = MOVE_HEADING_TOL; // Heading tolerance in degrees

uint32_t _move_last_derivative_update = 0; // The last time that the derivative buckets were updated, in ms
unsigned long _move_last_update_ms = 0; // The last time that _move_update_motor was called, in ms
bool _move_reading_stale = false; // Whether the 'last' encoder readings are stale

float _move_desired_speed(move_mstate_t* m); // Compute desired speed from the trapezoidal plan
int16_t _move_pid(move_mstate_t* m, float desired, unsigned long curr_time); // Compute PID output for a single motor

bool _move_tracking = false; // Keep track of if we are tracking required modules

void move_csch_tick() {
    switch (move_state) {
        case MOVE_S_INIT:
            move_proc = csch_ctask();
            move_state = MOVE_S_IDLE;
            break;

        case MOVE_S_IDLE:
            break;

        // Executing a move to a specific heading
        // Update the target positions based on the current heading logic, then continue with standard movement code
        case MOVE_S_ROTATE_TO:
            if (_move_planning) break; // Inhibit all movement if planning

            // Check if we've reached the target heading
            {
                float current = dir_heading();
                float diff = _move_target_heading - current;
                if (diff > 180) diff -= 360;
                if (diff < -180) diff += 360;

                if (fabsf(diff) <= _move_heading_tol) {
                    mot_power_set(0, 0);
                    move_state = MOVE_S_IDLE;
                    break;
                }
            }
            // nobreak — fall through to common movement code

        case MOVE_S_ROTATE:     // Indefinite rotation
        case MOVE_S_DRIVE:      // Indefinite drive
        case MOVE_S_DRIVE_BY:   // Drive to a specific distance
        case MOVE_S_STOP:
            if (_move_planning) break; // Inhibit all movement if planning

            if (!_move_tracking) {
                dir_track();
                steps_track();
                _move_tracking = true;
            }

            csch_cqueue(MOVE_CSCH_TK_PERIOD);
            _move_update_motor(&_move_motor_left, &_move_motor_right);
            _move_evaluate_plan(&_move_motor_left, &_move_motor_right);

            // Check completion for bounded movements
            if (move_state == MOVE_S_DRIVE_BY || move_state == MOVE_S_STOP) {
                if (_move_motor_left.current.pos >= _move_motor_left.target.pos &&
                    _move_motor_right.current.pos >= _move_motor_right.target.pos) {
                    Serial.println("Reached target!");
                    mot_power_set(0, 0);
                    move_state = MOVE_S_IDLE;

                    // Stop tracking since we're idle now
                    if (_move_tracking) {
                        dir_untrack();
                        steps_untrack();
                        _move_tracking = false;
                    }
                }
            }
            break;
    }
}

bool move_ready() {
    return move_state != MOVE_S_INIT;
}

bool move_busy() {
    return move_state != MOVE_S_IDLE;
}

bool move_stop(bool immediate) {
    if (!move_busy()) return false; // Movement module is not currently moving, cannot stop
    
    // Halt execution IMMEDIATELY
    if (immediate) {
        mot_power_set(0, 0);
        move_state = MOVE_S_IDLE;

        if (_move_tracking) {
            dir_untrack();
            steps_untrack();
            _move_tracking = false;
        }

        return true;
    }

    // Attempt to stop by decelerating
    _move_generate_plan(&_move_motor_left, _move_motor_left.current.pos, 0);
    _move_generate_plan(&_move_motor_right, _move_motor_right.current.pos, 0);

    // Perform movement
    _move_reading_stale = true;
    move_state = MOVE_S_STOP;
    csch_queue(move_proc.csch, move_proc.pid, 0);

    return true;
}

bool move_rotate(float speed) {
    if (speed == 0 || move_busy()) return false; // Movement module is currently busy, cannot issue new command

    // Encoder targets
    // Initial assumption is turning _right_
    int32_t left_target = 2147483647;
    int32_t right_target = -2147483648;

    // If speed is negative, we are actually turning left, so swap the targets
    if (speed < 0) {
        left_target = -2147483648;
        right_target = 2147483647;
        speed = -speed; // Make speed positive for the rest of the logic
    }

    // Generate plan to rotate indefinitely
    _move_generate_plan(&_move_motor_left, left_target, speed);
    _move_generate_plan(&_move_motor_right, right_target, speed);

    // Perform movement
    _move_reading_stale = true;
    move_state = MOVE_S_ROTATE;
    csch_queue(move_proc.csch, move_proc.pid, 0);

    return true;
}

bool move_drive(float speed) {
    if (speed == 0 || move_busy()) return false; // Movement module is currently busy, cannot issue new command

    // Generate plan for left and right motors
    // Achieved by making the target position very large
    _move_generate_plan(&_move_motor_left, 2147483647, speed);
    _move_generate_plan(&_move_motor_right, 2147483647, speed);

    // Perform movement
    _move_reading_stale = true;
    move_state = MOVE_S_DRIVE;
    csch_queue(move_proc.csch, move_proc.pid, 0);

    return true;
}

bool move_rotate_to(float angle, float speed) {
    if (speed <= 0 || (move_busy() && !(move_state == MOVE_S_ROTATE_TO || move_state == MOVE_S_ROTATE))) return false; // Movement module is currently busy with a non-rotation action, cannot issue new command

    // Clamp target heading to [0, 360)
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    _move_target_heading = angle;

    // Determine shortest rotation direction
    float current = dir_heading();
    float diff = angle - current;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;

    // Set up indefinite rotation plans (like move_rotate), heading check in tick will stop us
    // diff > 0 → rotate CW: left forward, right backward
    // diff < 0 → rotate CCW: left backward, right forward
    int32_t left_target = (diff >= 0) ? 2147483647 : -2147483648;
    int32_t right_target = (diff >= 0) ? -2147483648 : 2147483647;

    _move_generate_plan(&_move_motor_left, left_target, speed);
    _move_generate_plan(&_move_motor_right, right_target, speed);

    // Perform movement
    _move_reading_stale = true;
    move_state = MOVE_S_ROTATE_TO;
    csch_queue(move_proc.csch, move_proc.pid, 0);

    return true;
}

bool move_drive_by(int32_t distance, float speed) {
    if (speed <= 0 || (move_busy() && !(move_state == MOVE_S_DRIVE_BY || move_state == MOVE_S_DRIVE))) return false; // Movement module is currently busy with a non-driving action, cannot issue new command

    // Generate plan for left and right motors
    _move_generate_plan(&_move_motor_left, distance, speed);
    _move_generate_plan(&_move_motor_right, distance, speed);

    // Perform movement
    _move_reading_stale = true;
    move_state = MOVE_S_DRIVE_BY;
    csch_queue(move_proc.csch, move_proc.pid, 0);

    return true;
}

void move_plan(bool plan) {
    if (plan) {
        // If currently busy, stop the current action immediately before moving into the planning state
        if (move_busy()) move_stop(true);
        _move_planning = true;
    }

    // If exiting in the planning state, execute the updated plan immediately after moving out of the planning state
    else if (_move_planning) {
        _move_update_motor(&_move_motor_left, &_move_motor_right);
        _move_evaluate_plan(&_move_motor_left, &_move_motor_right);

        // Perform movement
        _move_planning = false;
        csch_queue(move_proc.csch, move_proc.pid, 0);
    }
}

void move_ramp(float accel) {
    if (isnanf(accel) || isinff(accel) || accel < 0) accel = 0; // Clamp acceleration and deceleration values to be non-negative and not NaN or Inf
    _move_accel = accel;
}

float move_ramp_get() {
    return _move_accel;
}

bool _move_generate_plan(move_mstate_t* mstate, int32_t distance, float target_speed) {
    // Determine direction from signs of distance and target_speed, then normalize both to positive
    // If both are negative they cancel out (forward). If one is negative, motor runs backward.
    int8_t dir = 1;
    if (distance < 0) { dir = -dir; }
    if (target_speed < 0) { target_speed = -target_speed; dir = -dir; }
    mstate->direction = dir;

    // Convert distance to unsigned absolute value (position tracking is always absolute)
    int32_t dist = (distance >= 0) ? distance : -distance;

    mstate->plan.initial_speed = fabsf(mstate->current.speed); // Cache the current speed magnitude
    mstate->target.pos = dist;

    // Initialize PID control variables
    mstate->last_error = 0;
    mstate->error_sum = 0;
    memset(&mstate->der_buckets, 0, sizeof(mstate->der_buckets));
    mstate->der_bucket_index = 0;

    // Reset position accumulator
    mstate->current.pos = 0;

    // Degenerate case where acceleration is zero (instant speed changes)
    if (_move_accel == 0) {
        mstate->plan.acc_pos = 0;
        mstate->plan.dec_pos = dist;
        mstate->target.speed = target_speed;
        return true;
    }

    // Algorithm: 4 cases for trapezoidal speed profile
    // I: Accelerate to target speed, cruise, then decelerate to 0
    // II: Accelerate to some speed < target, then immediately decelerate (triangle profile)
    // III: Decelerate to target speed, then maintain (currently going faster than target)
    // IV: Cannot stop in time — decelerate immediately, will overshoot

    float t1 = fabsf(target_speed - mstate->current.speed) / _move_accel; // Time to accelerate to target speed
    float t2 = target_speed / _move_accel;                                 // Time to decelerate from target speed to 0

    float d1 = 0.5 * _move_accel * t1 * t1 + mstate->current.speed * t1; // Distance covered during acceleration
    float d2_abs = 0.5 * _move_accel * t2 * t2;                           // Distance covered during deceleration (= 0.5 * v^2 / a)
    float d2 = (float) dist - d2_abs;                                     // Position at which to start decelerating

    // Case I: Normal trapezoidal profile — enough room to accelerate, cruise, and decelerate
    if (d2 >= d1) {
        mstate->plan.acc_pos = (int32_t) d1;
        mstate->plan.dec_pos = (int32_t) d2;
        mstate->target.speed = target_speed;
        return true;
    }

    // Case II: Not enough room to reach target speed — compute peak speed for a triangle profile
    float speed_sq = mstate->current.speed * mstate->current.speed;
    float det = 4 * speed_sq - 4 * _move_accel * (speed_sq / (2 * _move_accel) - (float) dist);

    // Case IV: Cannot stop in time (negative discriminant or negative time)
    if (det < 0) {
        mstate->plan.acc_pos = 0;
        mstate->plan.dec_pos = 0;
        mstate->target.speed = 0;
        return false;
    }

    t1 = (-2 * mstate->current.speed + sqrtf(det)) / (2 * _move_accel);

    if (t1 < 0) {
        mstate->plan.acc_pos = 0;
        mstate->plan.dec_pos = 0;
        mstate->target.speed = 0;
        return false;
    }

    // Triangle profile: accelerate then immediately decelerate (acc_pos == dec_pos)
    d1 = 0.5 * _move_accel * t1 * t1 + mstate->current.speed * t1;
    target_speed = _move_accel * t1 + mstate->current.speed;

    mstate->plan.acc_pos = (int32_t) d1;
    mstate->plan.dec_pos = (int32_t) d1; // Start decelerating immediately after accelerating
    mstate->target.speed = target_speed;

    return true;
}

void _move_update_motor(move_mstate_t* left, move_mstate_t* right) {
    unsigned long curr_time = move_proc.csch->curr_time();

    // Read the new encoder values
    int32_t left_reading, right_reading;
    steps_read(&left_reading, &right_reading);

    // Calculate deltas and update the last readings
    int16_t left_delta = left_reading - left->last_reading;
    int16_t right_delta = right_reading - right->last_reading;

    left->last_reading = left_reading;
    right->last_reading = right_reading;

    // Update positions (absolute distance traveled)
    left->current.pos += left_delta * left->direction;
    right->current.pos += right_delta * right->direction;

    // First reading after idle: don't compute speed (delta_ms would be bogus)
    if (_move_reading_stale) {
        _move_reading_stale = false;
        _move_last_update_ms = curr_time;
        _move_last_derivative_update = curr_time;
        return;
    }

    // Compute speed from encoder delta and elapsed time
    unsigned long delta_ms = curr_time - _move_last_update_ms;
    if (delta_ms > 0) {
        left->current.speed = fabsf((float) left_delta * 1000.0 / (float) delta_ms);
        right->current.speed = fabsf((float) right_delta * 1000.0 / (float) delta_ms);
    }
    _move_last_update_ms = curr_time;

    // Update derivative buckets periodically for PID smoothing
    if (curr_time - _move_last_derivative_update >= MOVE_DER_MS_PERIOD) {
        left->der_buckets[left->der_bucket_index] = left->current.pos;
        right->der_buckets[right->der_bucket_index] = right->current.pos;

        left->der_bucket_index = (left->der_bucket_index + 1) % MOVE_DER_BUCKETS;
        right->der_bucket_index = (right->der_bucket_index + 1) % MOVE_DER_BUCKETS;

        _move_last_derivative_update = curr_time;
    }
}

float _move_desired_speed(move_mstate_t* m) {
    int32_t pos = m->current.pos;

    // Complete: past target
    if (pos >= m->target.pos) return 0;

    // Accelerating phase
    if (pos < m->plan.acc_pos) {
        if (m->plan.acc_pos == 0) return m->target.speed;
        float speed = m->plan.initial_speed + (m->target.speed - m->plan.initial_speed) * ((float) pos / (float) m->plan.acc_pos);
        // Enforce minimum speed during acceleration so motors can overcome static friction
        if (speed < (float) _move_min_pwm) speed = (float) _move_min_pwm;
        return speed;
    }

    // Cruising phase
    if (pos < m->plan.dec_pos) return m->target.speed;

    // Decelerating phase
    if (m->plan.dec_pos >= m->target.pos) return 0; // Guard: dec_pos past target means stop
    int32_t dec_range = m->target.pos - m->plan.dec_pos;
    return m->target.speed * (1.0 - (float)(pos - m->plan.dec_pos) / (float) dec_range);
}

int16_t _move_pid(move_mstate_t* m, float desired, unsigned long curr_time) {
    float error = desired - m->current.speed;

    // Integral accumulation with anti-windup clamp
    m->error_sum += (int32_t) error;
    if (m->error_sum > 10000) m->error_sum = 10000;
    if (m->error_sum < -10000) m->error_sum = -10000;

    // Derivative from smoothed bucket history
    float der_time = (float)(MOVE_DER_BUCKETS * MOVE_DER_MS_PERIOD) / 1000.0;
    float derivative = 0;
    if (der_time > 0) {
        derivative = (error - m->last_error) / der_time;
    }

    m->last_error = (int32_t) error;

    // Compute PID output
    int32_t output = _move_kp * (int32_t) error + _move_ki * m->error_sum + _move_kd * (int32_t) derivative;

    if (m == &_move_motor_right) {
        Serial.print("Right PID: error=");
        Serial.print(error);
        Serial.print(", sum=");
        Serial.print(m->error_sum);
        Serial.print(", derivative=");
        Serial.print(derivative);
        Serial.print(", output=");
        Serial.println(output);
    }

    // Dead zone compensation: ensure output meets minimum PWM if motor should be moving
    if (desired != 0) {
        if (output > 0 && output < _move_min_pwm) output = _move_min_pwm;
        if (output < 0 && output > -_move_min_pwm) output = -_move_min_pwm;
    }

    // Clamp to motor range
    if (output > 255) output = 255;
    if (output < -255) output = -255;

    return (int16_t) output;
}

void _move_evaluate_plan(move_mstate_t* left, move_mstate_t* right) {
    unsigned long curr_time = move_proc.csch->curr_time();

    // Compute desired speeds from trapezoidal plans
    float left_desired = _move_desired_speed(left);
    float right_desired = _move_desired_speed(right);

    // Compute individual PID outputs (always positive magnitude), then apply direction
    int16_t left_output = _move_pid(left, left_desired, curr_time) * left->direction;
    int16_t right_output = _move_pid(right, right_desired, curr_time) * right->direction;

    // Motor synchronization bias: keep left and right traveling the same absolute distance
    if (_move_sync_kp != 0) {
        int32_t sync_error = (int32_t) left->current.pos - (int32_t) right->current.pos;
        // Apply bias in the direction each motor is traveling
        left_output -= _move_sync_kp * sync_error * left->direction;
        right_output += _move_sync_kp * sync_error * right->direction;

        // Re-clamp after sync
        if (left_output > 255) left_output = 255;
        if (left_output < -255) left_output = -255;
        if (right_output > 255) right_output = 255;
        if (right_output < -255) right_output = -255;
    }

    mot_power_set(left_output, right_output);
}

void move_motors_get(move_mstate_t* left, move_mstate_t* right) {
    *left = _move_motor_left;
    *right = _move_motor_right;
}

void move_pid(int32_t kp, int32_t ki, int32_t kd) {
    _move_kp = kp;
    _move_ki = ki;
    _move_kd = kd;
}

void move_pid_get(int32_t* kp, int32_t* ki, int32_t* kd) {
    *kp = _move_kp;
    *ki = _move_ki;
    *kd = _move_kd;
}

void move_min_pwm(uint8_t min_pwm) { _move_min_pwm = min_pwm; }
uint8_t move_min_pwm_get() { return _move_min_pwm; }

void move_sync(int32_t kp) { _move_sync_kp = kp; }
int32_t move_sync_get() { return _move_sync_kp; }

void move_heading_tol(float tol) {
    if (tol < 0 || isnanf(tol) || isinff(tol)) tol = 0;
    _move_heading_tol = tol;
}
float move_heading_tol_get() { return _move_heading_tol; }
