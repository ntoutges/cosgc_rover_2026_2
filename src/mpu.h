#ifndef _MPU_H
#define _MPU_H

/**
 * @file mpu.h
 * @author Nicholas T.
 * @brief Shared interface for the MPU-6050 IMU
 * @version 0.1
 * @date 2026-04-06
 * 
 * @copyright PiCO 2026
 */

#include <stdbool.h>
#include <Wire.h>
#include "csch.h"
#include "config.h"

// -------- SCHEDULING --------

typedef enum mpu_state_t {
    MPU_S_INIT,     // Initialize MPU
    MPU_S_IDLE,     // Do nothing
    MPU_S_TRACKING, // Poll MPU for readings
    MPU_S_AUTOCAL_1,  // Initialize autocal data
    MPU_S_AUTOCAL_2   // Take autocal samples
} mpu_state_t;

#define MPU_S_MIN MPU_S_INIT
#define MPU_S_MAX MPU_S_AUTOCAL_2

extern mpu_state_t mpu_state;
extern csch_curr_t mpu_proc;
extern uint8_t mpu_trackers;

/**
 * @brief The entrypoint into the MPU state machine, for use by the csch scheduling library
 */
void mpu_csch_tick();


// -------- CALIBRATION --------

/**
 * @brief The calibration data for the MPU, which consists of offsets for the accelerometer and gyroscope readings along each axis
 */
typedef struct mpu_cal_t {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} mpu_cal_t;

/**
 * @brief Setup MPU calibration data
 * 
 * @param cal   The calibration data for the MPU
 */
void mpu_cal(mpu_cal_t cal);

/**
 * @brief Automatically calibrate the MPU by taking a number of samples and averaging them, with a delay between each sample
 * 
 * @param samples   The number of samples to take for calibration
 * @param delay_tk  The delay in ticks between each sample
 */
void mpu_autocal(uint8_t samples, uint16_t delay_tk);

/**
 * @brief Get the current MPU calibration data
 * 
 * @returns The current MPU calibration data
 */
mpu_cal_t mpu_cal_get();


// -------- READING --------

/**
 * @brief Check if the MPU is ready to be read from (i.e. it has been initialized and is not currently calibrating)
 * 
 * @return true iff the MPU is ready to be read from
 */
bool mpu_ready();

/**
 * @brief Get the accumulated rotation about the x/y/z axes
 * 
 * @param x The accumulated rotation about the x axis, in the range [0, 360)
 * @param y The accumulated rotation about the y axis, in the range [0, 360)
 * @param z The accumulated rotation about the z axis, in the range [0, 360)
 */
void mpu_rot_ac(float *x, float *y, float *z);

/**
 * @brief Reset the accumulated rotation about the x/y/z axes to zero
 * 
 * @param x Reset the accumulated rotation about the x axis to zero if true
 * @param y Reset the accumulated rotation about the y axis to zero if true
 * @param z Reset the accumulated rotation about the z axis to zero if true
 */
void mpu_rot_reset(bool x, bool y, bool z);

/**
 * @brief Get the immediate acceleration along the x/y/z axes, in the range [-32768, 32767]
 * Note that the accelerometer is setup for a range of +/- 2g. (maximum value of 32767 corresponds to an acceleration of 2g)
 * 
 * @param x The immediate acceleration along the x axis, in the range [-32768, 32767]
 * @param y The immediate acceleration along the y axis, in the range [-32768, 32767]
 * @param z The immediate acceleration along the z axis, in the range [-32768, 32767]
 */
void mpu_acc(int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief Get the maximum acceleration along the x/y/z axes since the last reset.
 * Note that this is the maximum _magnitude_ of acceleration, so it is always a positive value
 * 
 * @param x The maximum acceleration along the x axis, in the range [0, 65535]
 * @param y The maximum acceleration along the y axis, in the range [0, 65535]
 * @param z The maximum acceleration along the z axis, in the range [0, 65535]
 */
void mpu_acc_max(uint16_t *x, uint16_t *y, uint16_t *z);

/**
 * @brief Reset the maximum acceleration along the x/y/z axes to zero
 * 
 * @param x Reset the maximum acceleration along the x axis to zero if true
 * @param y Reset the maximum acceleration along the y axis to zero if true
 * @param z Reset the maximum acceleration along the z axis to zero if true
 */
void mpu_acc_reset(bool x, bool y, bool z);


// -------- Control --------

/**
 * @brief Indicate that some process is listening to MPU readings.
 * Starts polling the MPU for readings, and updates the accumulated rotation and maximum acceleration values.
 * Up to 256 processes can be tracking the MPU at once
 * 
 */
void mpu_track();

/**
 * @brief Indicate that some process is no longer listening to MPU readings.
 * If no processes are tracking the MPU, then stop polling the MPU for readings to save power/processing.
 * 
 */
void mpu_untrack();

#endif
