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