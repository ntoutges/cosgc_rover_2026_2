#include "cli.h"

cli_state_t _cli_state = CLI_S_INIT;
cmd_t _cli_cmd;

cmd_entry_t _cli_entry_buf[CLI_ENTRY_BUF_SIZE];
uint8_t _cli_buf_buf[CLI_BUF_BUF_SIZE];

void cli_csch_tick() {
    switch (_cli_state) {
        case CLI_S_INIT:
            _cli_cmd = cmd(_cli_entry_buf, sizeof(_cli_entry_buf) / sizeof(cmd_entry_t), _cli_buf_buf, sizeof(_cli_buf_buf), '!');

            cmd_attach(&_cli_cmd, "mpu", _cli_mpu_cmd, NULL);
            cmd_attach(&_cli_cmd, "dir", _cli_dir_cmd, NULL);
            cmd_attach(&_cli_cmd, "steps", _cli_steps_cmd, NULL);
            cmd_attach(&_cli_cmd, "mot", _cli_mot_cmd, NULL);


            csch_cqueue(CLI_CSCH_TK_PASSIVE); // Queue next tick in passive mode
            _cli_state = CLI_S_PASSIVE;
            break;

        case CLI_S_PASSIVE:
            if (Serial.available()) {
                _cli_state = CLI_S_ACTIVE; // Input available, read and process it
                csch_cqueue(CLI_CSCH_TK_ACTIVE); // Queue next tick in active mode
            }
            else {
                // Slowly poll for input in passive mode
                csch_cqueue(CLI_CSCH_TK_PASSIVE);
            }
            break;

        case CLI_S_ACTIVE:
            int parse_ct = Serial.available();

            if (!parse_ct) {
                _cli_state = CLI_S_PASSIVE; // No more input, go back to passive mode
                csch_cqueue(CLI_CSCH_TK_PASSIVE); // Queue next tick in passive mode
                break;
            }

            // Quickly poll for input in active mode
            csch_cqueue(CLI_CSCH_TK_ACTIVE);

            if (parse_ct > CLI_MAX_TICK_PARSE) {
                parse_ct = CLI_MAX_TICK_PARSE; // Limit number of characters parsed each tick to avoid blocking for too long
            }

            // Run command receive state machine on incoming characters
            for (int i = 0; i < parse_ct; i++) {
                char recv = Serial.read();
                
                if (cmd_recv(&_cli_cmd, recv)) continue; // Consume all characters that are part of valid commands
                if (mot_user_recv(recv)) continue;       // Consume all characters that are used for user motor control
            }
            break;
    }
}

void _cli_mpu_cmd(void*) {
    const char* subcmd = cmd_ogets(&_cli_cmd, 1, "");

    if (strcmp(subcmd, "acal") == 0) {
        int samples = cmd_ogeti(&_cli_cmd, 2, 0);
        int delay = cmd_ogeti(&_cli_cmd, 3, 0);

        if (samples <= 0 || samples > 255 || delay <= 0 || delay > 65535) {
            Serial.println("~acal requires arguments samples, delay");
            return;
        }

        mpu_autocal((uint8_t) samples, (uint16_t) delay);
        Serial.println("=acal");
        return;
    }

    if (strcmp(subcmd, "cal") == 0) {
        mpu_cal_t cal = mpu_cal_get();

        // Return result as space-separated values in the order ax ay az gx gy gz
        Serial.print("=cal ");
        Serial.print(cal.ax);
        Serial.print(" ");
        Serial.print(cal.ay);
        Serial.print(" ");
        Serial.print(cal.az);
        Serial.print(" ");
        Serial.print(cal.gx);
        Serial.print(" ");
        Serial.print(cal.gy);
        Serial.print(" ");
        Serial.println(cal.gz);
        return;
    }

    if (strcmp(subcmd, "!cal") == 0) {
        int16_t ax = (int16_t) cmd_ogeti(&_cli_cmd, 2, 0);
        int16_t ay = (int16_t) cmd_ogeti(&_cli_cmd, 3, 0);
        int16_t az = (int16_t) cmd_ogeti(&_cli_cmd, 4, 0);
        int16_t gx = (int16_t) cmd_ogeti(&_cli_cmd, 5, 0);
        int16_t gy = (int16_t) cmd_ogeti(&_cli_cmd, 6, 0);
        int16_t gz = (int16_t) cmd_ogeti(&_cli_cmd, 7, 0);

        mpu_cal((mpu_cal_t){
            .ax = ax,
            .ay = ay,
            .az = az,
            .gx = gx,
            .gy = gy,
            .gz = gz
        });
        Serial.println("=!cal");
        return;
    }

    if (strcmp(subcmd, "acc") == 0) {
        int16_t ax, ay, az;
        mpu_acc(&ax, &ay, &az);

        Serial.print("=acc ");
        Serial.print(ax);
        Serial.print(" ");
        Serial.print(ay);
        Serial.print(" ");
        Serial.println(az);
        return;
    }

    if (strcmp(subcmd, "accmax") == 0) {
        uint16_t ax_max, ay_max, az_max;
        mpu_acc_max(&ax_max, &ay_max, &az_max);

        Serial.print("=accmax ");
        Serial.print(ax_max);
        Serial.print(" ");
        Serial.print(ay_max);
        Serial.print(" ");
        Serial.println(az_max);
        return;
    }

    if (strcmp(subcmd, "accreset") == 0) {
        bool x = cmd_ugetb(&_cli_cmd, "x", false);
        bool y = cmd_ugetb(&_cli_cmd, "y", false);
        bool z = cmd_ugetb(&_cli_cmd, "z", false);

        // Default to all axes if no axes specified
        if (!x && !y && !z) {
            x = true;
            y = true;
            z = true;
        }

        mpu_acc_reset(x, y, z);
        Serial.print("=accreset ");
        if (x) Serial.print("x");
        if (y) Serial.print("y");
        if (z) Serial.print("z");
        Serial.println();
        return;
    }

    if (strcmp(subcmd, "rot") == 0) {
        float rx, ry, rz;
        mpu_rot_ac(&rx, &ry, &rz);

        Serial.print("=rot ");
        Serial.print(rx);
        Serial.print(" ");
        Serial.print(ry);
        Serial.print(" ");
        Serial.println(rz);
        return;
    }

    if (strcmp(subcmd, "rotreset") == 0) {
        bool x = cmd_ugetb(&_cli_cmd, "x", false);
        bool y = cmd_ugetb(&_cli_cmd, "y", false);
        bool z = cmd_ugetb(&_cli_cmd, "z", false);

        // Default to all axes if no axes specified
        if (!x && !y && !z) {
            x = true;
            y = true;
            z = true;
        }

        mpu_rot_reset(x, y, z);
        Serial.print("=rotreset ");
        if (x) Serial.print("x");
        if (y) Serial.print("y");
        if (z) Serial.print("z");
        Serial.println();
        return;
    }

    if (strcmp(subcmd, "ready") == 0) {
        Serial.print("=ready ");
        Serial.println(mpu_ready() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "state") == 0) {
        Serial.print("=state ");
        Serial.println(mpu_state);
        return;
    }

    if (strcmp(subcmd, "!state") == 0) {
        int state = cmd_ogeti(&_cli_cmd, 2, 0);

        if (state < MPU_S_INIT || state > MPU_S_AUTOCAL_2) {
            Serial.println("~Invalid state argument");
            return;
        }

        mpu_state = (mpu_state_t) state;
        Serial.println("=!state");
        return;
    }

    if (strcmp(subcmd, "track") == 0) {
        mpu_track();
        Serial.println("=!track");
        return;
    }

    if (strcmp(subcmd, "!track") == 0) {
        mpu_track();
        Serial.println("=!track");
        return;
    }

    if (strcmp(subcmd, "!untrack") == 0) {
        mpu_untrack();

        Serial.println("=!untrack");
        return;
    }

    if (strcmp(subcmd, "pid") == 0) {
        Serial.print("=pid ");
        Serial.println(mpu_proc.pid);
        return;
    }

    Serial.println("~Unknown mpu subcommand");
}

void _cli_dir_cmd(void*) {
    const char* subcmd = cmd_ogets(&_cli_cmd, 1, "");

    if (strcmp(subcmd, "acal") == 0) {
        const char* state = cmd_ogets(&_cli_cmd, 2, "");

        if (strcmp(state, "begin") == 0) {
            dir_autocal_begin();
            Serial.println("=acal begin");
            return;
        }
        else if (strcmp(state, "end") == 0) {
            bool success = dir_autocal_end();
            Serial.print("=acal end");
            Serial.println(success ? "" : " fail");
            return;
        }
        else {
            Serial.println("~Invalid acal argument (begin/end)");
            return;
        }
    }

    if (strcmp(subcmd, "cal") == 0) {
        dir_cal_t cal = dir_cal_get();

        // Return result as space-separated values in the order x_min x_max y_min y_max z_min z_max
        Serial.print("=cal ");
        Serial.print(cal.x_min);
        Serial.print(" ");
        Serial.print(cal.x_max);
        Serial.print(" ");
        Serial.print(cal.y_min);
        Serial.print(" ");
        Serial.print(cal.y_max);
        Serial.print(" ");
        Serial.print(cal.z_min);
        Serial.print(" ");
        Serial.println(cal.z_max);
        return;
    }

    if (strcmp(subcmd, "!cal") == 0) {
        int16_t x_min = (int16_t) cmd_ogeti(&_cli_cmd, 2, 0);
        int16_t x_max = (int16_t) cmd_ogeti(&_cli_cmd, 3, 0);
        int16_t y_min = (int16_t) cmd_ogeti(&_cli_cmd, 4, 0);
        int16_t y_max = (int16_t) cmd_ogeti(&_cli_cmd, 5, 0);
        int16_t z_min = (int16_t) cmd_ogeti(&_cli_cmd, 6, 0);
        int16_t z_max = (int16_t) cmd_ogeti(&_cli_cmd, 7, 0);

        dir_cal((dir_cal_t){
            .x_min = x_min,
            .x_max = x_max,
            .y_min = y_min,
            .y_max = y_max,
            .z_min = z_min,
            .z_max = z_max
        });
        Serial.println("=!cal");
        return;
    }

    if (strcmp(subcmd, "heading") == 0) {
        Serial.print("=heading ");
        Serial.println(dir_heading());
        return;
    }

    if (strcmp(subcmd, "ref") == 0) {
        float ref = dir_ref_get();

        Serial.print("=ref ");
        Serial.println(ref);
        return;
    }

    if (strcmp(subcmd, "!ref") == 0) {
        float ref = cmd_ogetf(&_cli_cmd, 2, 0);

        dir_ref(ref);
        Serial.println("=!ref");
        return;
    }

    if (strcmp(subcmd, "orient") == 0) {
        dir_orient_t orient = dir_orient_get();

        Serial.print("=orient ");
        switch (orient) {
            case DIR_O_YZ:
                Serial.println("yz");
                break;
            case DIR_O_XZ:
                Serial.println("xz");
                break;
            case DIR_O_XY:
            default:
                Serial.println("xy");
                break;
        }
    }

    if (strcmp(subcmd, "!orient") == 0) {
        const char* orient_str = cmd_ogets(&_cli_cmd, 2, "");

        dir_orient_t orient;

        if (strcmp(orient_str, "yz") == 0) {
            orient = DIR_O_YZ;
        }
        else if (strcmp(orient_str, "xz") == 0) {
            orient = DIR_O_XZ;
        }
        else if (strcmp(orient_str, "xy") == 0) {
            orient = DIR_O_XY;
        }
        else {
            Serial.println("~Invalid orient argument (yz/xz/xy)");
            return;
        }

        dir_orient(orient);
        Serial.println("=!orient");
        return;
    }

    if (strcmp(subcmd, "magnet") == 0) {
        bool safe = dir_magnet_safe_get();

        Serial.print("=magnet ");
        Serial.println(safe ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "!magnet") == 0) {
        const char* safe_str = cmd_ogets(&_cli_cmd, 2, "");

        bool safe;
        if (strcmp(safe_str, "true") == 0) {
            safe = true;
        }
        else if (strcmp(safe_str, "false") == 0) {
            safe = false;
        }
        else {
            Serial.println("~Invalid magnet argument (true/false)");
            return;
        }

        dir_magnet_safe(safe);
        Serial.println("=!magnet");
        return;
    }

    if (strcmp(subcmd, "ready") == 0) {
        Serial.print("=ready ");
        Serial.println(dir_ready() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "state") == 0) {
        Serial.print("=state ");
        Serial.println(dir_state);
        return;
    }

    if (strcmp(subcmd, "!state") == 0) {
        int state = cmd_ogeti(&_cli_cmd, 2, 0);

        if (state < DIR_S_INIT || state > DIR_S_AUTOCAL_2) {
            Serial.println("~Invalid state argument");
            return;
        }

        dir_state = (dir_state_t) state;
        Serial.println("=!state");
        return;
    }

    if (strcmp(subcmd, "track") == 0) {
        dir_track();
        Serial.println("=track");
        return;
    }

    if (strcmp(subcmd, "!track") == 0) {
        dir_track();
        Serial.println("=!track");
        return;
    }

    if (strcmp(subcmd, "!untrack") == 0) {
        dir_untrack();
        Serial.println("=!untrack");
        return;
    }

    if (strcmp(subcmd, "pid") == 0) {
        Serial.print("=pid ");
        Serial.println(dir_proc.pid);
        return;
    }

    Serial.println("~Unknown dir subcommand");
}
