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

#include <Arduino.h>
#include <stdbool.h>
#include "csch.h"
#include "config.h"

// -------- SCHEDULING --------

typedef enum steps_state_t {
    STEPS_S_INIT,       // Initializing steps module
    STEPS_S_IDLE,       // Idle state
    STEPS_S_TRACKING,   // Module is being used
    STEPS_S_AUTOCAL_1,  // Initialize autocal data
    STEPS_S_AUTOCAL_2   // Take autocal samples
} steps_state_t;

#define STEPS_S_MIN STEPS_S_INIT
#define STEPS_S_MAX STEPS_S_TRACKING

extern csch_curr_t steps_proc;
extern steps_state_t steps_state;
extern uint8_t steps_trackers;

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
 * Note that this value is the result _after_ multiplying by the efficiency coefficient
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

/**
 * @brief Define a coefficient of "effeciency". One measured step may not correspond to one unit of distance (due to slippage, etc.), so this coefficient can be used to adjust the observed step counts
 * 
 * @param coef  The coefficient to apply to the observed step counts (e.g. a value of 0.5 would mean that every observed step corresponds to 0.5 units of distance)
 */
void steps_coef(float coef);

/**
 * @brief Get the current efficiency coefficient
 * 
 * @return The current efficiency coefficient
 */
float steps_coef_get();


// -------- CONTROL --------

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

// -------- INTERNAL --------

/**
 * @brief Get the change in steps based on the change in raw readings from the motor, accounting for wraparound
 * 
 * @param raw   The new raw reading from the motor
 * @param last  The last raw reading from the motor
 * @param min   The minimum raw reading to expect from the motorbased on calibration
 * @param max   The maximum raw reading to expect from the motor
 * @return The change in steps (in range [-512, 511]) based on the change in raw readings, accounting for wraparound
 * Note that this function assumes that the change in raw readings between two consecutive readings will not exceed 512 (half of the ADC range), which should be a safe assumption given the expected speed of the motors and the frequency of readings
 */
int16_t _steps_delta(uint16_t raw, uint16_t last, uint16_t min, uint16_t max);

#endif