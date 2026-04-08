#include "cmd.h"

bool _cmd_update_cache(cmd_t* cmd); // Attempt to update cache based on newly received character in command buffer; Returns `false` on error
uint8_t _cmd_find_ucache(cmd_t* cmd, const char* name); // Search for a cache entry with the given name; Returns index of cache entry if found, 0xFF otherwise
uint8_t _cmd_find_ocache(cmd_t* cmd, uint8_t idx); // Search for a cache entry with the given index; Returns index of cache entry if found, 0xFF otherwise
uint8_t _cmd_find_fcache(cmd_t* cmd, uint8_t start); // Search for the next single-character flag cache entry starting from the given index; Returns index of cache entry if found, 0xFF otherwise
cmd_cache_t _cmd_read_cache(cmd_t* cmd, uint16_t cache_idx); // Read a cache entry at the given index; Returns the cache entry
uint8_t _cmd_cache_len(cmd_t* cmd); // Get the length of the command cache (number of entries)

cmd_t cmd(cmd_entry_t* entry_buf, uint8_t entry_size, uint8_t* buf_buf, uint8_t buf_size, char initiator) {
    memset(entry_buf, 0, entry_size * sizeof(cmd_entry_t)); // Clear command entry buffer

    return (cmd_t) {
        .initiator = initiator,
        .state = CMD_RECV_OPEN,
        .last_cache = 0,
        .entries = (cmd_entries_t) {
            .buf = entry_buf,
            .cap = entry_size,
        },
        .buf = (cmd_buf_t) {
            .buf = buf_buf,
            .cap = buf_size,
            .chr_len = 0,
            .cch_idx = buf_size - sizeof(cmd_cache_t)
        }
    };
}

bool cmd_recv(cmd_t* cmd, char ch) {
    if (ch == '\r') return false; // Ignore the evil character

    if (ch == '\n') {
        if (cmd->state == CMD_RECV_COMMAND) {
            // Newline (not preceded by space) indicates end of command token; Attempt to analyze and cache new token
            if ((cmd->buf.chr_len > 0 && cmd->buf.buf[cmd->buf.chr_len - 1] != ' ')) {

                // Command interpretation failed; Move to closed state
                if (!_cmd_update_cache(cmd)) cmd->state = CMD_RECV_ERR;
            }
        }

        bool char_used = false;
        if (cmd->state == CMD_RECV_COMMAND || cmd->state == CMD_RECV_COMMAND_I) {
            // Attempt to run associated command
            const char* command_buf = cmd_ogets(cmd, 0, ""); // Ensure cache is updated with all received tokens before running command callback

            for (uint8_t i = 0; i < cmd->entries.cap; i++) {
                if (cmd->entries.buf[i].ch != NULL && strcmp(cmd->entries.buf[i].ch, command_buf) == 0) {
                    cmd->entries.buf[i].cb(cmd->entries.buf[i].args); // Found matching command entry; Run callback with associated arguments
                    break;
                }
            }

            char_used = true;
        }

        // Reset command state on newline
        cmd->state = CMD_RECV_OPEN;
        return char_used;
    }

    switch (cmd->state) {
        case CMD_RECV_OPEN:
            if (ch == cmd->initiator) {
                cmd->state = CMD_RECV_COMMAND;

                // Reset cache and command buffer indices to prepare for new command
                cmd->last_cache = 0;
                cmd->buf.chr_len = 0;
                cmd->buf.cch_idx = cmd->buf.cap - sizeof(cmd_cache_t);
                return true;
            }
            cmd->state = CMD_RECV_CLOSED;
            return false;

        case CMD_RECV_CLOSED:
            return false;

        case CMD_RECV_COMMAND:
            
            // Quote not preceded by backslash indicates start of an inhibited token
            if (ch == '"' && (cmd->buf.chr_len == 0 || cmd->buf.buf[cmd->buf.chr_len - 1] != '\\')) {
                cmd->state = CMD_RECV_COMMAND_I;
                return true;
            }

            // New space indicates end of command token; Attempt to analyze and cache new token
            if (ch == ' ' && (cmd->buf.chr_len > 0 && cmd->buf.buf[cmd->buf.chr_len - 1] != ' ')) {

                // Cache update failed; Drop command and wait for newline to reset
                if (!_cmd_update_cache(cmd)) {
                    cmd->state = CMD_RECV_ERR;
                }

                return true;
            }

            if (cmd->buf.chr_len < cmd->buf.cch_idx) {
                cmd->buf.buf[cmd->buf.chr_len++] = ch;
                return true;
            }

            // Buffer overflow; Drop command and wait for newline to reset
            cmd->state = CMD_RECV_ERR;
            return true;
        
        case CMD_RECV_COMMAND_I:
            // Quote not preceded by backslash indicates end of inhibited token
            if (ch == '"' && (cmd->buf.chr_len == 0 || cmd->buf.buf[cmd->buf.chr_len - 1] != '\\')) {
                cmd->state = CMD_RECV_COMMAND;
                return true;
            }

            // Ensure enough space is left for the next flag cache entry describing this token
            if (cmd->buf.chr_len + 1 < cmd->buf.cch_idx) {
                cmd->buf.buf[cmd->buf.chr_len++] = ch;
                return true;
            }

            // Ran out of space in buffer; Drop command and wait for newline to reset
            cmd->state = CMD_RECV_ERR;
            return true;

        case CMD_RECV_ERR:
            // Buffer overflow; Drop command and wait for newline to reset
            return true;
    }

    // How did you get here!?
    return false;
}

uint8_t cmd_attach(cmd_t* cmd, const char* command, void (*cb)(void* args), void* args) {
    // Search for an empty slot in the command entry buffer
    for (uint8_t i = 0; i < cmd->entries.cap; i++) {
        if (cmd->entries.buf[i].ch == NULL) { // Found empty slot
            cmd->entries.buf[i] = (cmd_entry_t) {
                .ch = command,
                .cb = cb,
                .args = args
            };
            return i; // Return id of attached command entry
        }
    }

    return 0xFF; // No empty slot found; Return failure code
}

uint8_t cmd_detach(cmd_t* cmd, uint8_t id) {
    if (id >= cmd->entries.cap) return 0xFF; // Invalid id; Return failure code

    cmd->entries.buf[id].ch = NULL; // Mark command entry as empty
    return id; // Return id of detached command entry
}

int cmd_ugeti(cmd_t* cmd, const char* name, int default_val) {
    uint8_t cidx = _cmd_find_ucache(cmd, name);
    if (cidx == 0xFF) return default_val; // Cache entry not found; Return default value

    cmd_cache_t cache = _cmd_read_cache(cmd, cidx);
    return atoi((char*) &cmd->buf.buf[cache.value]);
}

float cmd_ugetf(cmd_t* cmd, const char* name, float default_val) {
    uint8_t cidx = _cmd_find_ucache(cmd, name);
    if (cidx == 0xFF) return default_val; // Cache entry not found; Return default value

    cmd_cache_t cache = _cmd_read_cache(cmd, cidx);
    return atof((char*) &cmd->buf.buf[cache.value]);
}

bool cmd_ugetb(cmd_t* cmd, const char* name, bool default_val) {
    uint8_t cidx = 0xFF;
    
    // Look for single-character unordered flag
    if (strlen(name) == 1) {
        uint8_t last = 0;

        while (1) {
            cidx = _cmd_find_fcache(cmd, last);
            if (cidx == 0xFF) break; // No more single-character flags in cache; Stop searching

            cmd_cache_t cache = _cmd_read_cache(cmd, cidx);

            // Check if any of the characters in the flag value match the given name character
            uint8_t i = 0;
            while (cmd->buf.buf[cache.value + i] != 0x00) {
                if (cmd->buf.buf[cache.value + i] == name[0]) return true; // Found flag character in cache; Return true
                i++;
            }

            // Increment to check next single-character flag in cache
            last = cidx + 1;
        }
    }

    // Fallback to standard unordered flag lookup for non-single-character flags
    cidx = _cmd_find_ucache(cmd, name);
    if (cidx == 0xFF) return default_val; // Cache entry not found; Return default value

    cmd_cache_t cache = _cmd_read_cache(cmd, cidx);
    return atoi((char*) &cmd->buf.buf[cache.value]) != 0;
}

const char* cmd_ugets(cmd_t* cmd, const char* name, const char* default_val) {
    uint8_t cidx = _cmd_find_ucache(cmd, name);
    if (cidx == 0xFF) {
        return default_val;
    }

    cmd_cache_t cache = _cmd_read_cache(cmd, cidx);
    return (const char*) &cmd->buf.buf[cache.value];
}

int cmd_ogeti(cmd_t* cmd, uint8_t idx, int default_val) {
    uint8_t cidx = _cmd_find_ocache(cmd, idx);
    if (cidx == 0xFF) return default_val; // Cache entry not found; Return default value

    cmd_cache_t cache = _cmd_read_cache(cmd, cidx);
    return atoi((char*) &cmd->buf.buf[cache.value]);
}

float cmd_ogetf(cmd_t* cmd, uint8_t idx, float default_val) {
    uint8_t cidx = _cmd_find_ocache(cmd, idx);
    if (cidx == 0xFF) return default_val; // Cache entry not found; Return default value

    cmd_cache_t cache = _cmd_read_cache(cmd, cidx);
    return atof((char*) &cmd->buf.buf[cache.value]);
}

bool cmd_ogetb(cmd_t* cmd, uint8_t idx, bool default_val) {
    uint8_t cidx = _cmd_find_ocache(cmd, idx);
    if (cidx == 0xFF) return default_val; // Cache entry not found; Return default value

    cmd_cache_t cache = _cmd_read_cache(cmd, cidx);
    return atoi((char*) &cmd->buf.buf[cache.value]) != 0;
}

const char* cmd_ogets(cmd_t* cmd, uint8_t idx, const char* default_val) {
    uint8_t cidx = _cmd_find_ocache(cmd, idx);
    if (cidx == 0xFF) {
        return default_val;
    }

    cmd_cache_t cache = _cmd_read_cache(cmd, cidx);
    return (const char*) &cmd->buf.buf[cache.value];
}


// PRIVATE FUNCTIONS

bool _cmd_update_cache(cmd_t* cmd) {
    
    // Place null byte at the end of the current token to indicate end of string
    if (cmd->buf.chr_len < cmd->buf.cch_idx)
        cmd->buf.buf[cmd->buf.chr_len++] = 0x00;
    else return false; // Out of space in buffer

    cmd_cache_t cache;

    // Advance last_cache index to start of token (ignore any leading spaces)
    while (cmd->last_cache < cmd->buf.chr_len - 1 && cmd->buf.buf[cmd->last_cache] == ' ') {
        cmd->last_cache++;
    }

    // Token is raw value; Assume unordered value
    // Indicated by lack of leading '-' character
    if (cmd->buf.buf[cmd->last_cache] != '-') {

        if (cmd->last_cache >= cmd->buf.cch_idx - 1 || cmd->buf.cch_idx < sizeof(cmd_cache_t))
            return false; // Out of space in buffer

        // Cache new token as unordered value
        cache.name = cmd->last_cache;
        cache.value = cmd->last_cache;

        // Copy new cache entry to end of buffer, growing backwards from the end
        memcpy(&cmd->buf.buf[cmd->buf.cch_idx], &cache, sizeof(cmd_cache_t));
        cmd->buf.cch_idx -= sizeof(cmd_cache_t);
        cmd->last_cache = cmd->buf.chr_len;

        return true;
    }

    // Ensure token is long enough to be some valid flag
    if (cmd->last_cache + 1 >= cmd->buf.chr_len) {
        cmd->buf.chr_len = cmd->last_cache; // Drop invalid from buffer
        return false; // String too short to be a valid flag
    }

    // Token is single-character flag; Assume ordered flags
    // Indicated by a single leading '-' character followed by some number of characters
    if (cmd->buf.buf[cmd->last_cache + 1] != '-') {

        if (cmd->last_cache >= cmd->buf.cch_idx - 1 || cmd->buf.cch_idx < sizeof(cmd_cache_t))
            return false; // Out of space in buffer

        // Cache new token as ordered flag with empty value
        cache.name = cmd->last_cache; // Name pointing to '-' character indicates ordered single-char flag
        cache.value = cmd->last_cache + 1;

        // Copy new cache entry to end of buffer, growing backwards from the end
        memcpy(&cmd->buf.buf[cmd->buf.cch_idx], &cache, sizeof(cmd_cache_t));
        cmd->buf.cch_idx -= sizeof(cmd_cache_t);
        cmd->last_cache = cmd->buf.chr_len;
        return true;
    }

    // Looking at multi-character flag, in the format of '--flag value'
    // Note that this requires 2 tokens (--flag, value)
    // Ensure that >= 2 tokens are present; Otherwise: wait for more tokens to arrive
    uint8_t first_token_idx = cmd->last_cache + 2; // Skip past '--' characters
    uint8_t second_token_idx = first_token_idx;

    // Search for the second token by looking for the next null byte before the end of the buffer
    while (++second_token_idx < cmd->buf.chr_len - 1 && cmd->buf.buf[second_token_idx] != 0x00);
    second_token_idx++; // Move past null byte to start of second token

    // Reached end of buffer without finding second token; Wait for more tokens to arrive
    if (second_token_idx == cmd->buf.chr_len) {
        return true;
    }

    if (cmd->last_cache >= cmd->buf.cch_idx - 1 || cmd->buf.cch_idx < sizeof(cmd_cache_t))
        return false; // Out of space in buffer

    // Remove initial "--" from flag to save space
    memmove(&cmd->buf.buf[cmd->last_cache], &cmd->buf.buf[first_token_idx], cmd->buf.chr_len - first_token_idx);
    first_token_idx -= 2;
    second_token_idx -= 2;

    // Cache new token as ordered flag with value
    cache.name = first_token_idx;
    cache.value = second_token_idx;
    cmd->buf.chr_len -= 2; // Account for removed "--" characters

    // Copy new cache entry to end of buffer, growing backwards from the end
    memcpy(&cmd->buf.buf[cmd->buf.cch_idx], &cache, sizeof(cmd_cache_t));
    cmd->buf.cch_idx -= sizeof(cmd_cache_t);
    cmd->last_cache = cmd->buf.chr_len;

    return true;
}

// Look for any unordered flag cache entries (name != value) with the given name.
uint8_t _cmd_find_ucache(cmd_t* cmd, const char* name) {
    uint8_t cache_entries = _cmd_cache_len(cmd);

    for (uint8_t i = 0; i < cache_entries; i++) {
        cmd_cache_t cache = _cmd_read_cache(cmd, i);

        if (
            cache.name < cmd->buf.cap &&
            cache.value < cmd->buf.cap &&
            cache.value != cache.name && // name != value indicates unordered flag; Skip ordered flags
            cmd->buf.buf[cache.name] != '-' && // '-' character in name indicates ordered single-character flag, not an unordered flag; Skip these entries
            memcmp(&cmd->buf.buf[cache.name], name, strlen(name) + 1) == 0
        ) {
            return i;
        }
    }

    return 0xFF;
}

// Look for any ordered flag cache (name = value) entries.
// Returns the `idx`th entry
uint8_t _cmd_find_ocache(cmd_t* cmd, uint8_t idx) {
    uint8_t cache_entries = _cmd_cache_len(cmd);
    uint8_t found_flags = 0;

    for (uint8_t i = 0; i < cache_entries; i++) {
        cmd_cache_t cache = _cmd_read_cache(cmd, i);

        if (
            cache.name < cmd->buf.cap &&
            cache.value < cmd->buf.cap &&
            cache.name == cache.value && // name = value indicates ordered flag
            found_flags++ == idx
        ) {
            return i;
        }
    }

    return 0xFF;
}

// Look for any unordered flag cache entries starting from the given index
// Return the first whose `name` points to a '-' character
uint8_t _cmd_find_fcache(cmd_t* cmd, uint8_t start) {
    uint8_t cache_entries = _cmd_cache_len(cmd);

    for (uint16_t i = start; i < cache_entries; i++) {
        cmd_cache_t cache = _cmd_read_cache(cmd, i);

        if (cache.name < cmd->buf.cap && cmd->buf.buf[cache.name] == '-') {
            return i;
        }
    }

    return 0xFF; // No matching cache entry found
}


cmd_cache_t _cmd_read_cache(cmd_t* cmd, uint16_t cache_idx) {
    // Cache index out of bounds; Return empty cache
    if (cache_idx >= _cmd_cache_len(cmd))
        return (cmd_cache_t) { 0x00, 0x00 };

    cmd_cache_t cache;
    memcpy(&cache, &cmd->buf.buf[cmd->buf.cap - (cache_idx + 1) * sizeof(cmd_cache_t)], sizeof(cmd_cache_t));

    return cache;
}

uint8_t _cmd_cache_len(cmd_t* cmd) {
    return (cmd->buf.cap - cmd->buf.cch_idx) / sizeof(cmd_cache_t) - 1;
}
