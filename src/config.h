/**
 * @file config.h
 * @author Nicholas T.
 * @brief Global config settings for the rover
 * @version 0.1
 * @date 2026-04-06
 * 
 * @copyright Copyright (c) 2026
 */

#define MPU_I2C_ADDR 0x68
#define MPU_I2C_TK_TIMEOUT 1000 // Timeout for I2C communication with MPU, in ticks
#define MPU_CSCH_TK_PERIOD 1    // Period of MPU state machine, in ticks

#define DIR_I2C_ADDR 0x2C
#define DIR_I2C_TK_TIMEOUT 1000 // Timeout for I2C communication with DIR, in ticks
#define DIR_CSCH_TK_PERIOD 100  // Period of DIR state machine, in ticks

#define CLI_CSCH_TK_PASSIVE 100 // Number of ticks between CLI ticks without input
#define CLI_CSCH_TK_ACTIVE 1    // Number of ticks between CLI ticks with input
#define CLI_ENTRY_BUF_SIZE 8    // Number of commands that can be registered in the CLI
#define CLI_BUF_BUF_SIZE 64     // Size of the buffer for incoming CLI commands
#define CLI_MAX_TICK_PARSE 8    // Maximum number of characters to parse each tick
#define CLI_SERIAL1_BAUD 9600   // Baud rate on Serial1 for CLI output
#define CLI_SERIAL1_ENABLE 1    // Whether to enable Serial1 for CLI output (0 = disable, 1 = enable)
                                // If 0: Uses Serial

#define STEPS_CSCH_TK_PERIOD 1   // Period of movement state machine, in ticks
#define STEPS_LEFT_PIN A2        // Pin to measure steps for the left motor
#define STEPS_RIGHT_PIN A3       // Pin to measure steps for the right motor
#define STEPS_STEPS_PER_REV 200  // Number of steps per revolution of the motor
                                 // Note that a step is used as a stand-in for mm
#define STEPS_LEFT_REV 0         // Whether the left encoder is reversed
#define STEPS_RIGHT_REV 1        // Whether the right encoder is reversed

// Motor Control Pins
// Convention: (+1, -2) corresponds to forward movement, (-1, +2) corresponds to backward movement
#define MOT_PIN_L1 6            // Left motor control pin 1
#define MOT_PIN_L2 5            // Left motor control pin 2
#define MOT_PIN_R1 4            // Right motor control pin 1
#define MOT_PIN_R2 3            // Right motor control pin 2

#define MOT_PWM_L1 1            // Indicates that left motor control pin 1 is a PWM pin
                                // If 0, the L2 pin will be considered the PWM pin
#define MOT_PWM_R1 1            // Indicates that right motor control pin 1 is a PWM pin
                                // If 0, the R2 pin will be considered the PWM pin
#define MOT_FLUX_TK_DELAY 500   // Time to wait after movement before allowing the compass to be considered magnetically safe again, in ticks

#define POS_CSCH_TK_PERIOD 1    // Period of position state machine, in ticks
#define POS_WAIT_TK_PERIOD 100  // Period to wait for dir/steps modules to be ready before trying to track position, in ticks

#define MOVE_CSCH_TK_PERIOD 1       // Period of movement state machine, in ticks
#define MOVE_RAMP_STEP 1            // PWM units to add per tick during ramp-up (0 = instant; 5 = 0→255 in ~50ms)
