#ifndef _MOVE_H
#define _MOVE_H

/**
 * @file move.h
 * @author Nicholas T.
 * @brief Higher level motor control functions (MVP: direct power control with encoder/heading stopping)
 * @version 0.2
 * @date 2026-04-10
 *
 * @copyright PiCO 2026
 */

#include <string.h>
#include "csch.h"
#include "dir.h"
#include "steps.h"
#include "mot.h"
#include "config.h"

// -------- SCHEDULING --------

typedef enum move_state_t {
    MOVE_S_INIT,      // Initialize movement module
    MOVE_S_IDLE,      // Do nothing

    MOVE_S_DRIVE,     // Driving indefinitely at a set power
    MOVE_S_DRIVE_BY,  // Driving until a target distance is reached
    MOVE_S_ROTATE,    // Rotating indefinitely at a set power
    MOVE_S_ROTATE_TO  // Rotating until a target heading is reached
} move_state_t;

#define MOVE_S_MIN MOVE_S_INIT
#define MOVE_S_MAX MOVE_S_ROTATE_TO

extern csch_curr_t move_proc;
extern move_state_t move_state;

/**
 * @brief The entrypoint into the movement state machine, for use by the csch scheduling library
 */
void move_csch_tick();


// -------- API --------

/**
 * @brief Check if the movement module has finished initializing
 */
bool move_ready();

/**
 * @brief Check if the movement module is currently performing an action
 */
bool move_busy();

/**
 * @brief Stop all movement immediately
 */
void move_stop();

/**
 * @brief Drive indefinitely at the given power level
 *
 * @param power Motor power in [-255, 255]; positive = forward, negative = backward
 * @return true iff the command was accepted (not busy)
 */
bool move_drive(int16_t power);

/**
 * @brief Drive a specific distance at the given power level, then stop
 *
 * @param power Motor power in [-255, 255]; sign determines direction
 * @param distance Distance to travel in encoder steps (must be > 0)
 * @return true iff the command was accepted (not busy)
 */
bool move_drive_by(int16_t power, int32_t distance);

/**
 * @brief Rotate in place indefinitely at the given power level
 *
 * @param power Motor power in [-255, 255]; positive = clockwise, negative = counterclockwise
 * @return true iff the command was accepted (not busy)
 */
bool move_rotate(int16_t power);

/**
 * @brief Rotate in place to a specific heading, then stop
 * Direction is automatically chosen via shortest path.
 *
 * @param power Motor power magnitude (0-255)
 * @param heading Target heading in degrees [0, 360)
 * @return true iff the command was accepted (not busy, or currently rotating)
 */
bool move_rotate_to(uint8_t power, float heading);


// -------- CONFIGURATION --------

/**
 * @brief Set the heading tolerance for rotate_to completion
 * @param tol Tolerance in degrees
 */
void move_heading_tol(float tol);

/**
 * @brief Get the current heading tolerance
 */
float move_heading_tol_get();

#endif
