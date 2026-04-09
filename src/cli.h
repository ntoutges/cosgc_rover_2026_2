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
#include "steps.h"
#include "mot.h"
#include "pos.h"
#include "move.h"

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
 * - `raw`: Print the current raw readings from the DIR (usage: `!dir raw`)
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

/**
 * @brief Command callback for the `steps` command
 * 
 * Available Subcommands:
 * - `acal`: Start/stop automatic calibration of the steps module (usage: `!steps acal <begin/end>`)
 * - `cal`: Get the current calibration values of the steps module (usage: `!steps cal`)
 * - `!cal`: Apply some set of calibration values to the steps module (usage: `!steps !cal <left_min> <left_max> <right_min> <right_max>`)
 * - `read`: Print the current step counts for the left and right motors (usage: `!steps read`)
 * - `raw`: Print the current raw readings from the motor encoders for the left and right motors (usage: `!steps raw`)
 * - `reset`: Reset the step counts for the left and right motors (usage: `!steps !reset [-lr]`) (Defaults to both motors if no motors specified)
 * - `coef`: Print the current efficiency coefficient for the steps (usage: `!steps coef`)
 * - `!coef`: Set the efficiency coefficient for the steps (usage: `!steps !coef <coef>`)
 * 
 * - `ready`: Print whether the steps module is ready to be read from (usage: `!steps ready`)
 * - `state`: Print the current state of the steps state machine (usage: `!steps state`)
 * - `!state`: Print the current state of the steps state machine (usage: `!steps !state <state>`)
 * - `track`: Print out the number of steps trackers
 * - `!track`: Start tracking the steps readings (usage: `!steps !track`)
 * - `!untrack`: Stop tracking the steps readings (usage: `!steps !untrack [# of readings to untrack]`)
 * - `pid`: Print the current PID values for the steps (usage: `!steps pid`)
 */
void _cli_steps_cmd(void*);

/**
 * @brief Command callback for the `pos` command
 * 
 * Available Subcommands:
 * - `get`: Print the current position of the rover in a 2D plane,
 * - `set`: Set the current position of the rover in a 2D plane (usage: `!pos set <x> <y>`)
 * 
 * - `ready`: Print whether the position module is ready to be read from (usage: `!pos ready`)
 * - `state`: Print the current state of the position state machine (usage: `!pos state`)
 * - `!state`: Print the current state of the position state machine (usage: `!pos !state <state>`)
 * - `track`: Print out the number of position trackers
 * - `!track`: Start tracking the position readings (usage: `!pos !track`)
 * - `!untrack`: Stop tracking the position readings (usage: `!pos !untrack [# of readings to untrack]`)
 * - `pid`: Print the current PID values for the position (usage: `!pos pid`)
 */
void _cli_pos_cmd(void*);

/**
 * @brief Command callback for the `mot` command
 * 
 * Available Subcommands:
 * - `get`: Print the current power levels for the left and right motors (usage: `!mot get`)
 * - `set`: Set the current power levels for the left and right motors (usage: `!mot set <left> <right>`)
 * - `stop`: Stop the motors (usage: `!mot stop`)
 * - `override`: Print whether the user is currently in control of the motors (usage: `!mot override`) or set whether the user is currently in control of the motors (usage: `!mot !override <true/false>`)
 * - `!override`: Set whether the user is currently in control of the motors (usage: `!mot override <true/false>`)
 * 
 * - `ready`: Print whether the motors are ready to be controlled (usage: `!mot ready`)
 * - `state`: Print the current state of the motor state machine (usage: `!mot state`)
 * - `!state`: Print the current state of the motor state machine (usage: `!mot !state <state>`)
 * - `pid`: Print the current PID values for the motors (usage: `!mot pid`)
 */
void _cli_mot_cmd(void*);

/**
 * @brief Command callback for the `pos` command
 * 
 * Available Subcommands:
 * - `get`: Print the current position of the rover in a 2D plane, relative to the starting position, in steps (mm) (usage: `!pos get`)
 * - `set`: Set the current position of the rover in a 2D plane, relative to the starting position, in steps (mm) (usage: `!pos set <x> <y>`)
 * 
 * - `ready`: Print whether the position module is ready to be read from (usage: `!pos ready`)
 * - `state`: Print the current state of the position state machine (usage: `!pos state`)
 * - `!state`: Print the current state of the position state machine (usage: `!pos !state <state>`)
 * - `track`: Print out the number of position trackers
 * - `!track`: Start tracking the position readings (usage: `!pos !track`)
 * - `!untrack`: Stop tracking the position readings (usage: `!pos !untrack [# of readings to untrack]`)
 * - `pid`: Print the current PID values for the position (usage: `!pos pid`)
 */
void _cli_pos_cmd(void*);

/**
 * @brief Command callback for the `move` command
 * 
 * Available Subcommands:
 * - `busy`: Print whether the movement module is currently executing a movement command (usage: `!move busy`)
 * - `drive`: Drive forward or backward at a given speed (usage: `!move drive <speed>`) where speed a positive value corresponds to forward movement and a negative value corresponds to backward movement, and the speed is in steps (mm) per second
 * - `rotate`: Rotate in place at a given speed (usage: `!move rotate <speed>`) where a positive value corresponds to clockwise rotation and a negative value corresponds to counterclockwise rotation, and the speed is in steps (mm) per second
 * - `rotateto`: Rotate in place to a specific heading at a given speed (usage: `!move rotateto <angle> <speed>`) where the angle is in degrees in the range [0, 360) and the speed is a positive value in steps (mm) per second
 * - `driveby`: Drive forward or backward a specific distance at a given speed (usage: `!move driveby <distance> <speed>`) where a positive value for distance corresponds to forward movement and a negative value for distance corresponds to backward movement, the distance is in encoder steps, and the speed is in steps (mm) per second
 * - `stop`: Stop all movement, either immediately (-f) or by decelerating (usage: `!move stop [-f]`) (Defaults to decelerating if no flag specified)
 * - `ramp`: Get the acceleration/deceleration profile for movement (usage: `!move ramp`)
 * - `!ramp`: Set the acceleration/deceleration profile for movement (usage: `!move !ramp <accel>`) where accel is in steps (mm) per second squared, and a value of 0 corresponds to no ramping (i.e. instant acceleration and deceleration)
 * 
 * - `plan`: Move into/out of the planning state, which allows for updating the movement plan without executing it, and then execute the updated plan once ready (usage: `!move plan <true/false>`)
 * - `motors`: Print the current states of the specified motors (usage: `!move motors [-lr]`) (Defaults to both motors if no motors specified)
 * 
 * - `ready`: Print whether the movement module is ready to be controlled (usage: `!move ready`)
 * - `state`: Print the current state of the movement state machine (usage: `!move state`)
 * - `!state`: Print the current state of the movement state machine (usage: `!move !state <state>`)
 * - `pid`: Print the current PID values for the movement module (usage: `!move pid`)
 */
void _cli_move_cmd(void*);

#endif