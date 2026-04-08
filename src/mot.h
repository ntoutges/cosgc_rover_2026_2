#ifndef _MOT_H // Direct motor control
#define _MOT_H

/**
 * @file mot.h
 * @author Nicholas T.
 * @brief Module to control the rover motors
 * @version 0.1
 * @date 2026-04-08
 * 
 * @copyright PiCO 2026
 * 
 */

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include "csch.h"
#include "dir.h"
#include "config.h"

// -------- SCHEDULING --------

typedef enum mot_state_t {
    MOT_S_INIT, // Initialize motor module
    MOT_S_IDLE, // Do nothing
    MOT_S_MOVED // Cooldown after movement to reenable compass
} mot_state_t;

/**
 * @brief The entrypoint into the movement state machine, for use by the csch scheduling library
 */
void mot_csch_tick();

#define MOT_S_MIN MOT_S_INIT
#define MOT_S_MAX MOT_S_MOVED

extern csch_curr_t mot_proc;
extern mot_state_t mot_state;


// -------- READ --------

/**
 * @brief Check if the motors are ready to be controlled, which is only false during initialization 
 * 
 * @return true iff the motors are ready to be controlled (not initializing)
 */
bool mot_ready();

// -------- CONTROL --------

/**
 * @brief Set the power level for the left and right motors, in the range [-255, 255], where negative values correspond to backward movement and positive values correspond to forward movement
 * 
 * @param left  The power level for the left motor, in the range [-255, 255]
 * @param right The power level for the right motor, in the range [-255, 255]
 */
void mot_power_set(int16_t left, int16_t right);

/**
 * @brief Get the current power levels for the left and right motors, in the range [-255, 255]
 * 
 * @param left  The variable to store the current power level for the left motor, in the range [-255, 255]
 * @param right The variable to store the current power level for the right motor, in the range [-255, 255]
 */
void mot_power_get(int16_t* left, int16_t* right);


// -------- USER CONTROL --------

/**
 * @brief Indicate that the user is taking/relinquishing control of the motors, which allows/disallows the use of the mot_user_recv function to control the motors with user input. By default, user control is disabled.
 * 
 * @param ovr   If true, the user is taking control of the motors; If false, the user is relinquishing control of the motors
 */
void mot_user(bool ovr);

/**
 * @brief Check if the user is currently in control of the motors
 * 
 * @return true iff the user is currently in control of the motors
 */
bool mot_user_get();

/**
 * @brief Handle user input for motor control
 * 
 * Accepted commands:
 * - 'w': Move forward at full power (255)
 * - 's': Move backward at full power (-255)
 * - 'a': Turn left in place at full power (left motor at -255, right motor at 255)
 * - 'd': Turn right in place at full power (left motor at 255, right motor at -255)
 * - 'q': Move forward and turn left at full power (left motor at 0, right motor at 255)
 * - 'e': Move forward and turn right at full power (left motor at 255, right motor at 0)
 * - 'z': Move backward and turn left at full power (left motor at -255, right motor at 0)
 * - 'c': Move backward and turn right at full power (left motor at 0, right motor at -255)
 * - ' ': Stop all motors (left motor at 0, right motor at 0)
 * 
 * Note that these commands will _NOT_ affect the return value of `mot_get`, which will still return the last power levels set by `mot_power_set`. This allows user control to be used as a temporary override without affecting the underlying motor state.
 * 
 * @param cmd The character command received from the user to control the motors
 * @returns true iff the command was successfully processed and used to update motor control
 */
bool mot_user_recv(char cmd);


// -------- INTERNALS --------

/**
 * @brief Update the motor power levels based on the input left/right power levels
 * 
 * @param left  The desired power level for the left motor, in the range [-255, 255]
 * @param right The desired power level for the right motor, in the range [-255, 255]
 */
void _mot_update(int16_t left, int16_t right);

#endif