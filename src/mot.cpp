#include "mot.h"

csch_curr_t mot_proc;
mot_state_t mot_state = MOT_S_INIT;

// Current power levels for the left and right motors, in the range [-255, 255]
int16_t _mot_left = 0;
int16_t _mot_right = 0;
bool _mot_override = false;

void mot_csch_tick() {
    switch (mot_state) {
        case MOT_S_INIT:
            mot_proc = csch_ctask();

            // Initialize all motor pins to be output
            pinMode(MOT_PIN_L1, OUTPUT);
            pinMode(MOT_PIN_L2, OUTPUT);
            pinMode(MOT_PIN_R1, OUTPUT);
            pinMode(MOT_PIN_R2, OUTPUT);

            // Apply initial state to motors
            _mot_update(_mot_left, _mot_right);

            mot_state = MOT_S_IDLE;
            break;

        case MOT_S_IDLE:
            break;

        // Called after movement to reenable compass after a delay to allow flux from motors to dissipate
        case MOT_S_MOVED:
            dir_magnet_safe(true);
            mot_state = MOT_S_IDLE;
            break;
    }
}

bool mot_ready() {
    return mot_state != MOT_S_INIT;
}

void mot_power_set(int16_t left, int16_t right) {
    _mot_left = left;
    _mot_right = right;
    _mot_update(left, right);
}

void mot_power_get(int16_t* left, int16_t* right) {
    *left = _mot_left;
    *right = _mot_right;
}

void mot_user(bool ovr) {
    _mot_override = ovr;
}

bool mot_user_get() {
    return _mot_override;
}

#include <Arduino.h>

bool mot_user_recv(char cmd) {
    if (!_mot_override) return false; // User is not in control of the motors, ignore input

    switch (cmd) {
        case 'w': // Forward
            _mot_update(255, 255);
            break;
        case 'a': // Left
            _mot_update(-255, 255);
            break;
        case 's': // Backward
            _mot_update(-255, -255);
            break;
        case 'd': // Right
            _mot_update(255, -255);
            break;
        case 'q': // Forward + Left
            _mot_update(0, 255);
            break;
        case 'e': // Forward + Right
            _mot_update(255, 0);
            break;
        case 'z': // Backward + Left
            _mot_update(-255, 0);
            break;
        case 'c': // Backward + Right
            _mot_update(0, -255);
            break;
        case ' ': // Stop
            _mot_update(0, 0);
            break;
        default:
            return false; // Unrecognized command, ignore input
    }

    return true; // Command recognized and processed
}

void _mot_update(int16_t left, int16_t right) {
    
    // Clamp values to be in range [-255, 255]
    if (left < -255) left = -255;
    if (left > 255) left = 255;
    if (right < -255) right = -255;
    if (right > 255) right = 255;
    
    // Update actual motor power levels
    // Note that this does _NOT_ affect the internal _mot_left and _mot_right variables

    // Update left motor
#if MOT_PWM_L1
    if (left >= 0) {
        analogWrite(MOT_PIN_L1, left);
        digitalWrite(MOT_PIN_L2, LOW);
    } else {
        analogWrite(MOT_PIN_L1, 255 + left);
        digitalWrite(MOT_PIN_L2, HIGH);
    }
#else
    if (left >= 0) {
        digitalWrite(MOT_PIN_L1, HIGH);
        analogWrite(MOT_PIN_L2, 255 - left);
    } else {
        digitalWrite(MOT_PIN_L1, LOW);
        analogWrite(MOT_PIN_L2, -left);
    }
#endif

    // Update right motor
#if MOT_PWM_R1
    if (right >= 0) {
        analogWrite(MOT_PIN_R1, right);
        digitalWrite(MOT_PIN_R2, LOW);
    } else {
        analogWrite(MOT_PIN_R1, 255 + right);
        digitalWrite(MOT_PIN_R2, HIGH);
    }
#else
    if (right >= 0) {
        digitalWrite(MOT_PIN_R1, HIGH);
        analogWrite(MOT_PIN_R2, 255 - right);
    } else {
        digitalWrite(MOT_PIN_R1, LOW);
        analogWrite(MOT_PIN_R2, -right);
    }
#endif

    // Tell the direction module whether or not it is safe to use the magnetometer
    if (left != 0 || right != 0) {
        dir_magnet_safe(false);

        // Ensure that queued state machine calls don't override the magnetically safe state
        mot_state = MOT_S_IDLE;
    }
    else {
        // Wait some time before allowing the compass to be considered magnetically safe again, to allow any stray magnetic flux from the motors to dissipate
        mot_state = MOT_S_MOVED;
        csch_queue(mot_proc.csch, mot_proc.pid, MOT_FLUX_TK_DELAY);
    }
}
