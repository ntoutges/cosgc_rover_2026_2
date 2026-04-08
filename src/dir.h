#ifndef _COMPASS_H
#define _COMPASS_H

/**
 * @file dir.h
 * @author Nicholas T.
 * @brief Module to determine rover direction using the MPU6050 IMU and QMC5883L compass
 * @version 0.1
 * @date 2026-04-06
 * 
 * @copyright PiCO 2026
 */

#include <stdint.h>
#include <math.h>
#include "csch.h"
#include "mpu.h"

// -------- SCHEDULING --------

typedef enum dir_state_t {
    DIR_S_INIT,     // Initialize direction module
    DIR_S_IDLE,     // Do nothing
    DIR_S_TRACKING, // Poll for readings
    DIR_S_AUTOCAL_1,  // Initialize autocal data
    DIR_S_AUTOCAL_2   // Take autocal samples
} dir_state_t;

extern csch_curr_t dir_proc;
extern dir_state_t dir_state;

/**
 * @brief The entrypoint into the direction state machine, for use by the csch scheduling library
 */
void dir_csch_tick();


// -------- CALIBRATION --------

/**
 * @brief The calibration data for the direction module, which consists of minimum and maximum readings along each axis
 */
typedef struct dir_cal_t {
    int16_t x_min;
    int16_t x_max;
    int16_t y_min;
    int16_t y_max;
    int16_t z_min;
    int16_t z_max;
} dir_cal_t;

/**
 * @brief Calibrate the QMC5883L compass by providing the minimum and maximum readings along each axis, which can be obtained by rotating the rover in all directions and recording the minimum and maximum readings along each axis
 * If not called, the default calibration data is set to a a min/max of 0/1 on all axes, which will likely result in incorrect readings
 * 
 * @param cal The calibration data for the QMC5883L compass
 */
void dir_cal(dir_cal_t cal);

/**
 * @brief Start the automatic calibration process for the QMC5883L.
 * After this process begins, the rover should be slowly rotated along all major axes of rotation
 */
void dir_autocal_begin();

/**
 * @brief End the automatic calibration process for the QMC5883L, and save the calibration data obtained from the process.
 * 
 * @returns true iff the automatic calibration process was running and has now been ended
 */
bool dir_autocal_end();

/**
 * @brief Get the current direction calibration data
 * 
 * @return The current direction calibration data
 */
dir_cal_t dir_cal_get();


// -------- READING --------

/**
 * @brief Check if the direction module is ready to be read from (i.e. it has been initialized and is not currently calibrating)
 * 
 * @return true iff the direction module is ready to be read from
 */
bool dir_ready();

/**
 * @brief Get the current heading of the rover in degrees, in the range [0, 360), relative to some arbitrary reference direction
 * 
 */
float dir_heading();

/**
 * @brief Set the reference direction for the heading readings, which is the direction that will be considered as 0 degrees.
 * This value is relative to magnetic north.
 * By default, this is set to 0
 * 
 * @param heading   The reference direction for the heading readings, in degrees relative to magnetic north
 */
void dir_ref(float heading);

/**
 * @brief Get the current reference direction for the heading readings, in degrees relative to magnetic north
 * 
 * @return The current reference direction for the heading readings, in degrees relative to magnetic north
 */
float dir_ref_get();

typedef enum dir_orient_t {
    DIR_O_YZ, // Pull heading from rotation about X-axis
    DIR_O_XZ, // Pull heading from rotation about Y-axis
    DIR_O_XY, // Pull heading from rotation about Z-axis
} dir_orient_t;

/**
 * @brief Define the orientation of the compass module, which determines which axes of rotation correspond to which heading directions.
 * If never explicitly set, defaults to DIR_O_XY
 * 
 * @param orient    The orientation of the compass module
 */
void dir_orient(dir_orient_t orient);

/**
 * @brief Get the current orientation of the compass module
 * 
 * @return The current orientation of the compass module
 */
dir_orient_t dir_orient_get();


// -------- CONTROL --------

/**
 * @brief Indicate that the direction module is in a magnetically safe environment, which allows it to use the magnetometer readings for direction estimation.
 * Otherwise, the direction module will rely solely on accumulated gyroscope readings relative to the last "safe" measurement (Note that these relative readings are prone to drift)
 * 
 * @param safe Whether the direction module is in a magnetically safe environment or not
 */
void dir_magnet_safe(bool safe);

/**
 * @brief Get whether the direction module is in a magnetically safe environment or not
 * 
 * @return Whether the direction module is in a magnetically safe environment or not
 */
bool dir_magnet_safe_get();

/**
 * @brief Indicate that some process is listening to compass readings.
 * Starts polling the compass for readings, and updates the accumulated rotation and maximum acceleration values.
 * Up to 256 processes can be tracking the MPU at once
 * 
 */
void dir_track();

/**
 * @brief Indicate that some process is no longer listening to compass readings.
 * If no processes are tracking the compass, then stop polling the MPU for readings to save power/processing.
 * 
 */
void dir_untrack();

#endif
