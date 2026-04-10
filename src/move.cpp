#include "move.h"

csch_curr_t move_proc;
move_state_t move_state = MOVE_S_INIT;

float _move_target_heading = 0;
float _move_last_heading = 0; // Previous heading relative to `target_heading`, in the range [-180, 180)
int32_t _move_target_dist = 0;
int32_t _move_start_left = 0;
int32_t _move_start_right = 0;

// Ramp state: target is what we want, applied is what the motors are currently getting
int16_t _move_target_left = 0;
int16_t _move_target_right = 0;
int16_t _move_applied_left = 0;
int16_t _move_applied_right = 0;

bool _move_tracking = false;

// Start tracking encoder/compass modules when beginning movement
void _move_track() {
    if (!_move_tracking) {
        dir_track();
        steps_track();
        _move_tracking = true;
    }
}

// Stop tracking when movement ends
void _move_untrack() {
    if (_move_tracking) {
        dir_untrack();
        steps_untrack();
        _move_tracking = false;
    }
}

// Step a value toward a target by MOVE_RAMP_STEP
static int16_t _ramp_toward(int16_t current, int16_t target) {
    if (MOVE_RAMP_STEP == 0) return target; // Instant
    if (current < target) {
        current += MOVE_RAMP_STEP;
        if (current > target) current = target;
    } else if (current > target) {
        current -= MOVE_RAMP_STEP;
        if (current < target) current = target;
    }
    return current;
}

// Ramp applied power toward target, apply to motors. Returns true if still ramping.
bool _move_ramp() {
    _move_applied_left = _ramp_toward(_move_applied_left, _move_target_left);
    _move_applied_right = _ramp_toward(_move_applied_right, _move_target_right);
    mot_power_set(_move_applied_left, _move_applied_right);
    return (_move_applied_left != _move_target_left || _move_applied_right != _move_target_right);
}

// Set new target power and begin ramping
void _move_set_power(int16_t left, int16_t right) {
    _move_target_left = left;
    _move_target_right = right;
}

void move_csch_tick() {
    switch (move_state) {
        case MOVE_S_INIT:
            move_proc = csch_ctask();
            move_state = MOVE_S_IDLE;
            break;

        case MOVE_S_IDLE:
            break;

        case MOVE_S_DRIVE:
        case MOVE_S_ROTATE:
            _move_ramp();

            // Only keep ticking while ramping
            if (_move_applied_left != _move_target_left || _move_applied_right != _move_target_right) {
                csch_cqueue(MOVE_CSCH_TK_PERIOD);
            }
            break;

        case MOVE_S_DRIVE_BY: {
            csch_cqueue(MOVE_CSCH_TK_PERIOD);
            _move_ramp();

            // Check if we've traveled the target distance
            int32_t left, right;
            steps_read(&left, &right);

            int32_t left_dist = abs(left - _move_start_left);
            int32_t right_dist = abs(right - _move_start_right);

            if (left_dist >= _move_target_dist && right_dist >= _move_target_dist) {
                move_stop();
            }
            break;
        }

        case MOVE_S_ROTATE_TO: {
            csch_cqueue(MOVE_CSCH_TK_PERIOD);
            _move_ramp();

            // Translate reference frame s.t. _move_target_heading is at 0 and wrap to [-180, 180]
            float origin = dir_heading() - _move_target_heading;
            if (origin > 180) origin -= 360;
            if (origin < -180) origin += 360;

            if ((origin > 0 && _move_last_heading <= 0 || origin < 0 && _move_last_heading >= 0) && abs(origin) < 90 && abs(_move_last_heading) < 90) {
                // We've crossed the target heading, so stop
                move_stop();
            }

            _move_last_heading = dir_heading() - _move_target_heading;
            if (_move_last_heading> 180) _move_last_heading -= 360;
            if (_move_last_heading < -180) _move_last_heading += 360;
            break;
        }
    }
}

bool move_ready() {
    return move_state != MOVE_S_INIT;
}

bool move_busy() {
    return move_state != MOVE_S_IDLE && move_state != MOVE_S_INIT;
}

void move_stop() {
    _move_target_left = 0;
    _move_target_right = 0;
    _move_applied_left = 0;
    _move_applied_right = 0;
    mot_power_set(0, 0);
    move_state = MOVE_S_IDLE;
    _move_untrack();
}

bool move_drive(int16_t power) {
    if (power == 0 || move_busy()) return false;

    _move_track();
    _move_applied_left = 0;
    _move_applied_right = 0;
    _move_set_power(power, power);
    move_state = MOVE_S_DRIVE;
    csch_queue(move_proc.csch, move_proc.pid, MOVE_CSCH_TK_PERIOD);
    return true;
}

bool move_drive_by(int16_t power, int32_t distance) {
    if (power == 0 || move_busy()) return false;

    _move_track();

    // Snapshot current encoder positions
    steps_read(&_move_start_left, &_move_start_right);
    _move_target_dist = distance;

    _move_applied_left = 0;
    _move_applied_right = 0;
    _move_set_power(power, power);
    move_state = MOVE_S_DRIVE_BY;
    csch_queue(move_proc.csch, move_proc.pid, MOVE_CSCH_TK_PERIOD);
    return true;
}

bool move_rotate(int16_t power) {
    if (power == 0 || move_busy()) return false;

    _move_track();
    _move_applied_left = 0;
    _move_applied_right = 0;
    _move_set_power(power, -power);
    move_state = MOVE_S_ROTATE;
    csch_queue(move_proc.csch, move_proc.pid, MOVE_CSCH_TK_PERIOD);
    return true;
}

bool move_rotate_to(uint8_t power, float heading) {
    if (power == 0) return false;
    // Allow interrupting an in-progress rotation
    if (move_busy() && move_state != MOVE_S_ROTATE && move_state != MOVE_S_ROTATE_TO) return false;

    _move_track();

    // Clamp heading to [0, 360)
    while (heading < 0) heading += 360;
    while (heading >= 360) heading -= 360;
    _move_target_heading = heading;

    _move_last_heading = dir_heading() - heading;
    if (_move_last_heading >= 180) _move_last_heading -= 180;
    if (_move_last_heading < -180) _move_last_heading += 180;

    // Determine shortest rotation direction
    float current = dir_heading();
    float diff = heading - current;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;

    // diff > 0 = CW (left forward, right backward)
    int16_t p = (diff >= 0) ? (int16_t) power : -(int16_t) power;
    _move_applied_left = 0;
    _move_applied_right = 0;
    _move_set_power(p, -p);

    move_state = MOVE_S_ROTATE_TO;
    csch_queue(move_proc.csch, move_proc.pid, MOVE_CSCH_TK_PERIOD);
    return true;
}
