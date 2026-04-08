#ifndef _STEPS_H // Motor step counter
#define _STEPS_H

/**
 * @file steps.h
 * @author Nicholas T.
 * @brief Module to determine the distance traveled by the rover, assuming a straight line
 * @version 0.1
 * @date 2026-04-06
 * 
 * @copyright PiCO 2026
 * @todo Create interface + implement
 */

// #include <Arduino.h>
#include <stdbool.h>
#include "csch.h"
#include "config.h"

// -------- SCHEDULING --------

typedef enum steps_state_t {
    steps_S_INIT,     // Initialize direction module
    steps_S_IDLE,     // Do nothing
    steps_S_TRACKING, // Poll for readings
    steps_S_AUTOCAL_1,  // Initialize autocal data
    steps_S_AUTOCAL_2   // Take autocal samples
} steps_state_t;

extern csch_curr_t steps_proc;
extern steps_state_t steps_state;

/**
 * @brief The entrypoint into the steps state machine, for use by the csch scheduling library
 */
void steps_csch_tick();

// -------- CALIBRATION --------

typedef struct steps_cal_t {
    uint16_t left_min; // The minimum raw reading to expect
    uint16_t left_max; // The maximum raw reading to expect
    uint16_t right_min; // The minimum raw reading to expect
    uint16_t right_max; // The maximum raw reading to expect
} steps_cal_t;

/**
 * @brief Calibrate the steps module by providing the minimum and maximum raw readings to expect from the motor encoders, which can be used to adjust for differences in the motors or encoders
 * 
 * @param cal   The calibration data for the steps module, which consists of the minimum and maximum raw readings to expect from the motor encoders
 */
void steps_cal(steps_cal_t cal);

/**
 * @brief Calibrate the steps module by automatically determining the minimum and maximum raw readings to expect from the motor encoders, which can be used to adjust for differences in the motors or encoders
 */
void steps_autocal_begin();

/**
 * @brief End the automatic calibration process for the steps module and apply the automatically determined calibration values
 * 
 * @return true iff the automatic calibration process was successfully ended and the calibration values were applied; false if the automatic calibration process was not running, so it could not be ended
 */
bool steps_autocal_end();

/**
 * @brief Get the current direction calibration data
 * 
 * @return The current direction calibration data
 */
steps_cal_t steps_cal_get();

// -------- READING --------

/**
 * @brief Check if the steps module is ready to be read from (i.e. it has been initialized)
 * 
 * @return true iff the steps module is ready to be read from
 */
bool steps_ready();

/**
 * @brief Get the current step counts for the left and right motors, to be used to determine distance traveled
 * 
 * @param left  The variable to store the current step count for the left motor
 * @param right The variable to store the current step count for the right motor
 */
void steps_read(int32_t* left, int32_t* right);

/**
 * @brief Get the raw encoder readings for the left and right motors, which can be used for debugging or to implement custom step counting logic
 * 
 * @param left  The variable to store the current raw encoder reading for the left motor
 * @param right The variable to store the current raw encoder reading for the right motor
 */
void steps_raw(int16_t* left, int16_t* right);

/**
 * @brief Reset the step counts for the left and right motors to 0, to be used when starting a new movement command
 * 
 * @param left  The variable to store the current step count for the left motor
 * @param right The variable to store the current step count for the right motor
 */
void steps_reset(bool left, bool right);


// -------- CONTROL --------

/**
 * @brief Indicate that the steps module is in a magnetically safe environment, which allows it to use the magnetometer readings for direction estimation.
 * Otherwise, the steps module will rely solely on accumulated gyroscope readings relative to the last "safe" measurement (Note that these relative readings are prone to drift)
 * 
 * @param safe Whether the steps module is in a magnetically safe environment or not
 */
void steps_magnet_safe(bool safe);

/**
 * @brief Get whether the steps module is in a magnetically safe environment or not
 * 
 * @return Whether the steps module is in a magnetically safe environment or not
 */
bool steps_magnet_safe_get();

/**
 * @brief Indicate that some process is listening to step readings.
 * Starts polling the step module for readings, and updates the accumulated rotation and maximum acceleration values.
 * Up to 256 processes can be tracking the MPU at once
 * 
 */
void steps_track();

/**
 * @brief Indicate that some process is no longer listening to step readings.
 * If no processes are tracking the steps, then stop polling the MPU for readings to save power/processing.
 * 
 */
void steps_untrack();

#endif