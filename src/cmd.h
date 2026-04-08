#ifndef _CMD_H
#define _CMD_H

/**
 * @file cmd.h
 * @author Nicholas T.
 * @brief Command line argument parser
 * @version 0.1
 * @date 2026-04-07
 * 
 * @copyright PiCO 2026
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
    Command Format:
    command: <initiator><command> <args>\n

    - <initiator>: A character that indicates a command was sent (eg: '!')
    - <command>: A string of characters that identifies the command (eg: 'help')
    - <args>: Arguments to run the command with. Format of arguments is (ordered ordered ... ordered --flag named --flag named)
              Note that single-character arguments are also supported under the -f format, and can be grouped together (eg: -abc is equivalent to -a -b -c)
*/

typedef struct cmd_entry_t {
    const char* ch;   // Command character(s) to trigger on
    void (*cb)(void* args); // Function to run on command receipt
    void* args; // Arguments to pass to the function when the command is triggered. The objected pointed to must remain valid when the command is triggered.
} cmd_entry_t;

typedef struct cmd_entries_t {
    cmd_entry_t* buf;   // Array of command entries
    uint8_t cap;        // Capacity of the command buffer
} cmd_entries_t;

typedef struct cmd_buf_t {
    uint8_t* buf;  // 

    uint8_t cap; // Capacity of the command buffer

    uint8_t chr_len; // Current character index in the command buffer (grows from 0 to cap-1)
    uint8_t cch_idx; // Current character index in the command buffer (shrinks from cap-1 to 0)
} cmd_buf_t;

// Hold onto flag/value name
// If name = value: flag is ordered; Otherwise: flag is unordered
typedef struct cmd_cache_t {
    uint8_t name; // Pointer to name of arg; If points to '-' character, indicates single-character ordered flag list
    uint8_t value; // Pointer to value of arg
} cmd_cache_t;

typedef enum cmd_recv_state_t {
    CMD_RECV_OPEN,      // Able to receive command characters
    CMD_RECV_CLOSED,    // Initator not found; Wait for next newline to reset
    CMD_RECV_COMMAND,   // Initator found; Receiving command characters
    CMD_RECV_COMMAND_I, // Inhibit new flag/value pairs: Receiving a single token
    CMD_RECV_ERR        // Receive buffer full
} cmd_recv_state_t;

typedef struct cmd_t {
    char initiator; // Character that indicates a command was sent (eg: '!')

    cmd_recv_state_t state; // Receive state of the command handler
    uint8_t last_cache; // Index of the last cached character in the command buffer

    cmd_entries_t entries; // Command entries
    cmd_buf_t buf; // Buffer to hold current command character(s)
} cmd_t;

/**
 * Create a new command handler
 * @param entry_buf Buffer to hold command entries
 * @param entry_size Size of the command entry buffer
 * @param buf_buf Buffer to hold current command + cache
 * @param buf_size Size of the buf_buf
 * @param initiator Character that indicates a command was sent (eg: '!<cmd>')
 */
cmd_t cmd(cmd_entry_t* entry_buf, uint8_t entry_size, uint8_t* buf_buf, uint8_t buf_size, char initiator);

/**
 * Trigger on receiving a command character
 * @param ch The received command character
 * @param cmd   The command handler to trigger on
 * @returns     true if the character was part of a command, false otherwise
 */
bool cmd_recv(cmd_t* cmd, char ch);

/**
 * Attempt to attach a command entry point to the command handler
 * @param cmd Command handler to attach to
 * @param command Command string (0th ordered argument) to trigger on (eg: "help")
 * @param cb Function to run when the command is triggered
 * @param args Arguments to pass to the function when the command is triggered. The objected pointed to must remain valid when the command is triggered.
 * @returns The id of the attached command entry, or 0xFF on failure (eg: command buffer full, command already exists, etc.)
 */
uint8_t cmd_attach(cmd_t* cmd, const char* command, void (*cb)(void* args), void* args);

/**
 * Attempt to detach a command entry point from the command handler
 * @param cmd Command handler to detach from
 * @param id ID of the command entry to detach
 * @returns The id of the detached command entry, or 0xFF on failure
 */
uint8_t cmd_detach(cmd_t* cmd, uint8_t id);

/**
 * Attempt to grab an integer value from the unordered command cache
 * @param cmd Command to grab from
 * @param name Name of the argument to grab
 * @param default_val Value to return if the argument is not found
 */
int cmd_ugeti(cmd_t* cmd, const char* name, int default_val);

/**
 * Attempt to grab a float value from the unordered command cache
 * @param cmd Command to grab from
 * @param name Name of the argument to grab
 * @param default_val Value to return if the argument is not found
 */
float cmd_ugetf(cmd_t* cmd, const char* name, float default_val);

/**
 * Attempt to grab a boolean value from the unordered command cache
 * @param cmd Command to grab from
 * @param name Name of the argument to grab
 * @param default_val Value to return if the argument is not found
 */
bool cmd_ugetb(cmd_t* cmd, const char* name, bool default_val);

/**
 * Attempt to grab a boolean value from the ordered command cache
 * @param cmd Command to grab from
 * @param idx Index of the argument to grab (0-indexed)
 * @param default_val Default string value to copy into buffer if argument is not found
 */
const char* cmd_ugets(cmd_t* cmd, const char* name, const char* default_val);

/**
 * Attempt to grab an integer value from the ordered command cache
 * @param cmd Command to grab from
 * @param idx Index of the argument to grab (0-indexed)
 * @param default_val Value to return if the argument is not found
 */
int cmd_ogeti(cmd_t* cmd, uint8_t idx, int default_val);

/**
 * Attempt to grab a float value from the ordered command cache
 * @param cmd Command to grab from
 * @param idx Index of the argument to grab (0-indexed)
 * @param default_val Value to return if the argument is not found
 */
float cmd_ogetf(cmd_t* cmd, uint8_t idx, float default_val);

/**
 * Attempt to grab a boolean value from the ordered command cache
 * @param cmd Command to grab from
 * @param idx Index of the argument to grab (0-indexed)
 * @param default_val Value to return if the argument is not found
 */
bool cmd_ogetb(cmd_t* cmd, uint8_t idx, bool default_val);

/**
 * Attempt to grab a boolean value from the ordered command cache
 * @param cmd Command to grab from
 * @param idx Index of the argument to grab (0-indexed)
 * @param default_val Default string value to copy into buffer if argument is not found
 */
const char* cmd_ogets(cmd_t* cmd, uint8_t idx, const char* default_val);

#ifdef __cplusplus
}
#endif

#endif