#ifndef _MOVE_H
#define _MOVE_H

/**
 * @file move.h
 * @author Nihcolas T.
 * @brief Higher level motor control functions
 * @version 0.1
 * @date 2026-04-08
 * 
 * @copyright PiCO 2026
 * 
 */

#include <string.h>
#include "csch.h"
#include "dir.h"
#include "steps.h"
#include "mot.h"

// -------- SCHEDULING --------

typedef enum move_state_t {
    MOVE_S_INIT, // Initialize movement module
    MOVE_S_IDLE, // Do nothing

    MOVE_S_ROTATE_TO, // Currently executing a rotation to a specific heading
    MOVE_S_ROTATE,    // Currently executing an indefinite rotation
    MOVE_S_DRIVE_BY,  // Currently executing a drive to a specific distance
    MOVE_S_DRIVE,     // Currently executing a drive
    MOVE_S_STOP       // Currently executing a stop by deceleration
} move_state_t;

#define MOVE_S_MIN MOVE_S_INIT
#define MOVE_S_MAX MOVE_S_STOP

extern csch_curr_t move_proc;
extern move_state_t move_state;

/**
 * @brief The entrypoint into the movement state machine, for use by the csch scheduling library
 */
void move_csch_tick();


// -------- API --------

/**
 * @brief Check if the movement module is ready to be issued movement commands, which is true if it has finished initializing and is not currently performing a movement action, and false otherwise
 * 
 * @return true iff the movement module is ready to be issued movement commands
 */
bool move_ready();

/**
 * @brief Check if the movement module is currently busy performing an action, which is true if it is currently driving or rotating, and false if it is idle
 * 
 * @return true iff the movement module is currently busy performing an action
 */
bool move_busy();

/**
 * @brief Stop all movement, either immediately (which will likely result in a more abrupt stop) or by decelerating to a stop (which will likely result in a smoother stop)
 * 
 * @param immediate If true, stop immediately, and if false, decelerate to a stop
 * @return true iff the command was successfully issued, which will fail if the movement module is not currently moving. Note that this can override all other movement commands. The only command that can override this is another `move_stop` command with `immediate` set to true
 */
bool move_stop(bool immediate);

/**
 * @brief Rotate in place at the given speed, where negative values correspond to counterclockwise rotation and positive values correspond to clockwise rotation
 * 
 * @param speed The speed at which to run the motors to rotate, in steps (mm) per second
 * @return true iff the command was successfully issued, which will fail if the movement module is currently busy performing another action
 */
bool move_rotate(float speed);

/**
 * @brief Drive forward or backward at the given speed, where negative values correspond to backward movement and positive values correspond to forward movement
 * 
 * @param speed The speed at which to drive, in steps (mm) per second
 * @return true iff the command was successfully issued, which will fail if the movement module is currently busy performing another action
 */
bool move_drive(float speed);

/**
 * @brief Rotate in place to a specific heading at a given speed, where the heading is in degrees in the range [0, 360), and the speed is positive and in steps (mm) per second
 * 
 * @param angle The target heading in degrees, in the range [0, 360)
 * @param speed The speed at which to run the motors to rotate, in steps (mm) per second
 * @return true iff the command was successfully issued, which will fail if the movement module is currently busy performing another action. Note that this is allowed to interrupt other `move_rotate` or `move_rotate_to` commands
 */
bool move_rotate_to(float angle, float speed);

/**
 * @brief Drive a specific distance at a given speed, where the distance is in encoder steps and the speed is positive and in steps (mm) per second
 * 
 * @param distance The distance to drive in encoder steps
 * @param speed The speed at which to drive, in steps (mm) per second
 * @return true iff the command was successfully issued, which will fail if the movement module is currently busy performing another action. Note that this is allowed to interrupt other `move_drive` or `move_drive_by` commands
 */
bool move_drive_by(int32_t distance, float speed);

/**
 * @brief Move into/out of the planning state, which allows for updating the movement plan without executing it, and then execute the updated plan once ready
 * 
 * @param plan  If true, move into the planning state, and if false, move out of the planning state and execute the updated plan. Note that if the movement module is currently busy performing another action, `move_stop(true)` will be called to immediately stop the current action before moving into the planning state, and the updated plan will be executed immediately after moving out of the planning state
 */
void move_plan(bool plan);


// -------- RAMPING --------

/**
 * @brief Configure the acceleration and deceleration profiles for movement
 * By default, the acceleration and deceleration profiles are set to 0, which corresponds to no ramping (i.e. instant acceleration and deceleration)
 * If the movement module is currently moving, the new profiles will be applied after the current movement is finished
 * 
 * @param accel The acceleration/deceleration profile for movement, in steps (mm) per second squared; 0 => instant acceleration/deceleration
 */
void move_ramp(float accel);

/**
 * @brief Get the current acceleration and deceleration profiles for movement
 * If the movement module is currently moving, the new profiles will be applied after the current movement is finished
 * 
 * @return The acceleration profile for movement, in steps (mm) per second squared; 0 => instant acceleration/deceleration
 */
float move_ramp_get();


// -------- PID --------

/**
 * @brief Set the PID coefficients for movement speed control
 *
 * @param kp Proportional gain (PWM per step/s of speed error)
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void move_pid(int32_t kp, int32_t ki, int32_t kd);

/**
 * @brief Get the current PID coefficients for movement speed control
 */
void move_pid_get(int32_t* kp, int32_t* ki, int32_t* kd);

/**
 * @brief Set the minimum PWM value to overcome motor dead zone
 *
 * @param min_pwm Minimum PWM value (0-255); 0 to disable dead zone compensation
 */
void move_min_pwm(uint8_t min_pwm);

/**
 * @brief Get the current minimum PWM value
 */
uint8_t move_min_pwm_get();

/**
 * @brief Set the proportional gain for left/right motor synchronization
 *
 * @param kp Sync gain; 0 to disable synchronization
 */
void move_sync(int32_t kp);

/**
 * @brief Get the current motor synchronization gain
 */
int32_t move_sync_get();

/**
 * @brief Set the heading tolerance for rotate_to completion
 *
 * @param tol Tolerance in degrees; when the heading is within this many degrees of the target, rotation stops
 */
void move_heading_tol(float tol);

/**
 * @brief Get the current heading tolerance
 */
float move_heading_tol_get();


// -------- INTERNAL --------

typedef struct motor_microstate_t {
    int32_t pos; // Motor position in encoder steps (mm)
                 // Note that this value does _not_ directly correspond with the actual motor position
    float speed; // Motor speed in steps (mm) per second
} motor_microstate_t;

// Store the state of each motor for use in the movement state machine
typedef struct move_mstate_t {
    motor_microstate_t current; // Current state of the motor
    motor_microstate_t target;  // Desired state of the motor
                                // Note that the target.pos is _always_ greater than or equal to the current.pos, even when moving backwards, to simplify the logic for tracking movement progress
                                // Also note that the target speed is _always_ non-negative, where a speed of 0 indicates a desire to stop

    int8_t direction; // +1 for forward, -1 for backward; applied to PID output to determine motor power sign

    int32_t last_reading; // The last encoder reading for the motor, in encoder steps (mm)
    int32_t last_error; // The last position error, for use in the derivative term of PID control, in encoder steps (mm)
    int32_t error_sum; // The sum of the position errors over time, for use in the integral term of PID control, in encoder steps (mm) * seconds

    uint32_t der_buckets[MOVE_DER_BUCKETS]; // Buckets for smoothing out the derivative term of PID control, where each bucket stores the position error at a specific time in the past, in encoder steps (mm)
    uint8_t der_bucket_index; // The index of the current bucket for storing derivative errors

    // The plan for trapezoidal speed control, which is used to determine when to start and stop accelerating and decelerating
    struct {
        int32_t acc_pos; // The position at which the motor will finish accelerating and reach the target speed, in encoder steps
                         // Note that acceleration is implied to start at 0
        int32_t dec_pos; // The position at which the motor will start decelerating, in encoder steps
                         // Note that deceleration is implied to stop at target.pos
        float initial_speed; // The speed from which to start accelerating, in steps (mm) per second
                             // This is derived from the initial state of current.speed 
    } plan;

} move_mstate_t;

/**
 * @brief Generate a movement plan for the given motor state, target speed, and distance to move, which will be used for trapezoidal speed control
 * 
 * @param mstate        The state of the motor for which to generate the movement plan, which will be modified in-place to set the plan parameters
 * @param target_speed  The target speed for the movement, in steps (mm) per second
 * @param distance      The distance to move, in encoder steps
 * @return true iff the plan was successfully generated, which will fail if the given parameters are invalid (eg: unable to stop in time with the given deceleration profile, etc)
 */
bool _move_generate_plan(move_mstate_t* mstate, int32_t distance, float target_speed);

/**
 * @brief Update the current motor states based on the real-world data
 * 
 * @param left  The state of the left motor, which will be modified in-place to update the current state based on the real-world data
 * @param right The state of the right motor, which will be modified in-place to update the current state based on the real-world data
 */
void _move_update_motor(move_mstate_t* left, move_mstate_t* right);

/**
 * @brief Evaluate the current motor states against the movement plans, updating the real-world motor speeds as necessary to execute the plans
 * 
 * @param left  The state of the left motor
 * @param right The state of the right motor
 */
void _move_evaluate_plan(move_mstate_t* left, move_mstate_t* right);


// -------- DEBUGGING --------

/**
 * @brief Get the current states of both motors
 * 
 * @param left  The state of the left motor, which will be modified in-place to set the current state
 * @param right The state of the right motor, which will be modified in-place to set the current state
 */
void move_motors_get(move_mstate_t* left, move_mstate_t* right);

#endif