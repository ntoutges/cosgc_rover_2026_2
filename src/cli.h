#ifndef _CLI_H
#define _CLI_H

/**
 * @file cli.h
 * @author Nicholas T.
 * @brief Interface `cmd.h` with the rest of the program for easy control and debugging
 * @version 0.1
 * @date 2026-04-07
 * 
 * @copyright PiCO 2026
 * 
 */

#include <Arduino.h>

#include <stdint.h>
#include <string.h>
#include "csch.h"
#include "cmd.h"
#include "config.h"

#include "mpu.h"
#include "dir.h"

#include "mot.h"

// -------- SCHEDULING --------

typedef enum cli_state_t {
    CLI_S_INIT,    // Initialize internal `cmd` structure
    CLI_S_PASSIVE, // No input, wait for input
    CLI_S_ACTIVE   // Input available, read and process it
} cli_state_t;

/**
 * @brief The entrypoint into the cli state machine, for use by the csch scheduling library
 */
void cli_csch_tick();


// -------- INTERNALS --------

/**
 * @brief Command callback for the `mpu` command
 *
 * Available Subcommands:
 * - `acal`: Calibrate the MPU (usage: `!mpu acal <samples> <delay>`)
 * - `cal`: Print the current calibration values of the MPU (usage: `!mpu cal`)
 * - `!cal`: Apply some set of calibration values to the MPU (usage: `!mpu !cal <ax> <ay> <az> <gx> <gy> <gz>`)
 * - `acc`: Print the current acceleration readings from the MPU (usage: `!mpu acc`)
 * - `accmax`: Print the maximum acceleration readings from the MPU since the last reset (usage: `!mpu accmax`)
 * - `accreset`: Reset the maximum acceleration readings (usage: `!mpu !accreset [-xyz]`) (Defaults to all axes if no axes specified)
 * - `rot`: Print the current rotation readings from the MPU (usage: `!mpu rot`)
 * - `rotreset`: Reset the accumulated rotation readings (usage: `!mpu !rotreset [-xyz]`) (Defaults to all axes if no axes specified)
 * 
 * - `ready`: Print whether the MPU is ready to be read from (usage: `!mpu ready`)
 * - `state`: Print the current state of the MPU state machine (usage: `!mpu state`)
 * - `!state`: Print the current state of the MPU state machine (usage: `!mpu !state <state>`)
 * - `track`: Print out the number of MPU trackers
 * - `!track`: Start tracking the MPU readings (usage: `!mpu !track`)
 * - `!untrack`: Stop tracking the MPU readings (usage: `!mpu !untrack`)
 * - `pid`: Print the current PID values for the MPU (usage: `!mpu pid`)
 */
void _cli_mpu_cmd(void*);

/**
 * @brief Command callback for the `dir` command
 * 
 * Available Subcommands:
 * - `acal`: Start/stop automatic calibration of the DIR (usage: `!dir acal <begin/end>`)
 * - `cal`: Calibrate the DIR (usage: `!dir cal <x_min> <x_max> <y_min> <y_max>`)
 * - `!cal`: Apply some set of calibration values to the DIR (usage: `!dir !cal <x_min> <x_max> <y_min> <y_max> <z_min> <z_max>`)`
 * - `heading`: Print the current heading from the DIR (usage: `!dir heading`)
 * - `ref`: Print the current reference direction for the DIR (usage: `!dir ref`)
 * - `!ref`: Set the reference direction for the DIR (usage: `!dir ref <heading>`)
 * - `orient`: Print the current orientation of the DIR (usage: `!dir orient`) or set the orientation of the DIR (usage: `!dir !orient <orientation>`)
 * - `!orient`: Set the orientation of the DIR (usage: `!dir orient <orientation>`) where orientation is one of `yz`, `xz`, or `xy`
 * - `magnet`: Print whether the DIR is in a magnetically safe environment (usage: `!dir magnet`) or set whether the DIR is in a magnetically safe environment (usage: `!dir !magnet <safe>`)
 * - `!magnet`: Set whether the DIR is in a magnetically safe environment (usage: `!dir magnet <true/false>`)
 *
 * - `ready`: Print whether the MPU is ready to be read from (usage: `!mpu ready`)
 * - `state`: Print the current state of the MPU state machine (usage: `!mpu state`)
 * - `!state`: Print the current state of the MPU state machine (usage: `!mpu !state <state>`)
 * - `track`: Print out the number of MPU trackers
 * - `!track`: Start tracking the MPU readings (usage: `!mpu !track`)
 * - `!untrack`: Stop tracking the MPU readings (usage: `!mpu !untrack [# of readings to untrack]`)
 * - `pid`: Print the current PID values for the MPU (usage: `!mpu pid`)
 */
void _cli_dir_cmd(void*);

#endif