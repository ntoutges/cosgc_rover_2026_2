#ifndef _POS_H // Position Reading
#define _POS_H

/**
 * @file pos.h
 * @author Nicholas T.
 * @brief Module to determine rover position
 * @version 0.1
 * @date 2026-04-07
 * 
 * @copyright PiCO 2026
 * 
 */

#include "csch.h"
#include "dir.h"
#include "steps.h"

// -------- SCHEDULING --------

typedef enum pos_state_t {
    POS_S_INIT,     // Initialize position module
    POS_S_IDLE,     // Do nothing
    POS_S_TRACKING  // Poll for readings
} pos_state_t;

#define POS_S_MIN POS_S_INIT
#define POS_S_MAX POS_S_TRACKING

extern csch_curr_t pos_proc;
extern pos_state_t pos_state;
extern uint8_t pos_trackers;

/**
 * @brief The entrypoint into the position state machine, for use by the csch scheduling library
 */
void pos_csch_tick();


// -------- READING --------

/**
 * @brief Get the current position of the rover in a 2D plane, relative to the starting position, in steps (mm)
 * Note that this point describes the rotational center of the rover, s.t. rotation on-the-spot does _not_ change the position
 * 
 * @param x The x-coordinate of the rover's position, in steps (mm)
 * @param y The y-coordinate of the rover's position, in steps (mm)
 */
void pos_get(float* x, float* y);

/**
 * @brief Set the current position of the rover in a 2D plane, relative to the starting position, in steps (mm)
 * 
 * @param x The x-coordinate of the rover's position, in steps (mm)
 * @param y The y-coordinate of the rover's position, in steps (mm)
 */
void pos_set(float x, float y);


// -------- CONTROL --------

/**
 * @brief Indicate that some process is listening to position readings.
 * Starts polling the position sensors for readings, and updates the accumulated position and maximum acceleration values.
 * Up to 256 processes can be tracking the position at once
 * 
 */
void pos_track();

/**
 * @brief Indicate that some process is no longer listening to position readings.
 * If no processes are tracking the position, then stop polling the sensors for readings to save power/processing.
 * 
 */
void pos_untrack();

#endif