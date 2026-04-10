#ifndef _GOTO_H
#define _GOTO_H

/**
 * @file move.h
 * @author Nicholas T.
 * @brief Higher level rover position control functions, to go to a specific coordinate
 * Basically a wrapper around move module
 * @version 0.2
 * @date 2026-04-10
 *
 * @copyright PiCO 2026
 */

 #include "csch.h"
 #include "move.h"

//  -------- SCHEDULING --------

typedef enum goto_state_t {
    GOTO_S_INIT,    // Initialize goto module
    GOTO_S_IDLE,    // Do nothing
    GOTO_S_MOVE,    // Moving to target coordinate
} goto_state_t;

extern csch_curr_t goto_proc;
extern goto_state_t goto_state;

/**
 * @brief The entrypoint into the goto state machine, for use by the csch scheduling library
 */
void goto_csch_tick();

// -------- API --------

/**
 * @brief Move the rover to a specific coordinate relative to the starting position, in steps (mm)
 * 
 * @param x The x-coordinate of the target position, in steps (mm)
 * @param y The y-coordinate of the target position, in steps (mm)
 */
void goto_point(float x, float y);

/**
 * @brief Check if the goto module is currently performing an action
 * 
 * @return true iff the goto module is currently performing an action
 */
bool goto_busy();

#endif