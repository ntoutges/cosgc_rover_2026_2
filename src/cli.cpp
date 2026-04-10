#include "cli.h"

cli_state_t _cli_state = CLI_S_INIT;
cmd_t _cli_cmd;

cmd_entry_t _cli_entry_buf[CLI_ENTRY_BUF_SIZE];
uint8_t _cli_buf_buf[CLI_BUF_BUF_SIZE];

#if CLI_SERIAL1_ENABLE
#define CLI_SERIAL Serial1
#define CLI_BAUD CLI_SERIAL1_BAUD
#else
#define CLI_SERIAL Serial
#define CLI_BAUD 115200
#endif

void cli_csch_tick() {
    switch (_cli_state) {
        case CLI_S_INIT:
            _cli_cmd = cmd(_cli_entry_buf, sizeof(_cli_entry_buf) / sizeof(cmd_entry_t), _cli_buf_buf, sizeof(_cli_buf_buf), '!');

            cmd_attach(&_cli_cmd, "mpu", _cli_mpu_cmd, NULL);
            cmd_attach(&_cli_cmd, "dir", _cli_dir_cmd, NULL);
            cmd_attach(&_cli_cmd, "steps", _cli_steps_cmd, NULL);
            cmd_attach(&_cli_cmd, "mot", _cli_mot_cmd, NULL);
            cmd_attach(&_cli_cmd, "pos", _cli_pos_cmd, NULL);
            cmd_attach(&_cli_cmd, "move", _cli_move_cmd, NULL);

            // Startup serial for CLI input/output
            CLI_SERIAL.begin(CLI_BAUD);

            csch_cqueue(CLI_CSCH_TK_PASSIVE); // Queue next tick in passive mode
            _cli_state = CLI_S_PASSIVE;
            break;

        case CLI_S_PASSIVE:
            if (CLI_SERIAL.available()) {
                _cli_state = CLI_S_ACTIVE; // Input available, read and process it
                csch_cqueue(CLI_CSCH_TK_ACTIVE); // Queue next tick in active mode
            }
            else {
                // Slowly poll for input in passive mode
                csch_cqueue(CLI_CSCH_TK_PASSIVE);
            }
            break;

        case CLI_S_ACTIVE:
            int parse_ct = CLI_SERIAL.available();

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
                char recv = CLI_SERIAL.read();
                
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
            CLI_SERIAL.println("~acal requires arguments samples, delay");
            return;
        }

        mpu_autocal((uint8_t) samples, (uint16_t) delay);
        CLI_SERIAL.println("=acal");
        return;
    }

    if (strcmp(subcmd, "cal") == 0) {
        mpu_cal_t cal = mpu_cal_get();

        // Return result as space-separated values in the order ax ay az gx gy gz
        CLI_SERIAL.print("=cal ");
        CLI_SERIAL.print(cal.ax);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.print(cal.ay);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.print(cal.az);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.print(cal.gx);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.print(cal.gy);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.println(cal.gz);
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
        CLI_SERIAL.println("=!cal");
        return;
    }

    if (strcmp(subcmd, "acc") == 0) {
        int16_t ax, ay, az;
        mpu_acc(&ax, &ay, &az);

        CLI_SERIAL.print("=acc ");
        CLI_SERIAL.print(ax);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.print(ay);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.println(az);
        return;
    }

    if (strcmp(subcmd, "accmax") == 0) {
        uint16_t ax_max, ay_max, az_max;
        mpu_acc_max(&ax_max, &ay_max, &az_max);

        CLI_SERIAL.print("=accmax ");
        CLI_SERIAL.print(ax_max);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.print(ay_max);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.println(az_max);
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
        CLI_SERIAL.print("=accreset ");
        if (x) CLI_SERIAL.print("x");
        if (y) CLI_SERIAL.print("y");
        if (z) CLI_SERIAL.print("z");
        CLI_SERIAL.println();
        return;
    }

    if (strcmp(subcmd, "rot") == 0) {
        float rx, ry, rz;
        mpu_rot_ac(&rx, &ry, &rz);

        CLI_SERIAL.print("=rot ");
        CLI_SERIAL.print(rx);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.print(ry);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.println(rz);
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
        CLI_SERIAL.print("=rotreset ");
        if (x) CLI_SERIAL.print("x");
        if (y) CLI_SERIAL.print("y");
        if (z) CLI_SERIAL.print("z");
        CLI_SERIAL.println();
        return;
    }

    if (strcmp(subcmd, "ready") == 0) {
        CLI_SERIAL.print("=ready ");
        CLI_SERIAL.println(mpu_ready() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "state") == 0) {
        CLI_SERIAL.print("=state ");
        CLI_SERIAL.println(mpu_state);
        return;
    }

    if (strcmp(subcmd, "!state") == 0) {
        int state = cmd_ogeti(&_cli_cmd, 2, 0);

        if (state < MPU_S_MIN || state > MPU_S_MAX) {
            CLI_SERIAL.println("~Invalid state argument");
            return;
        }

        mpu_state = (mpu_state_t) state;
        CLI_SERIAL.println("=!state");
        return;
    }

    if (strcmp(subcmd, "track") == 0) {
        CLI_SERIAL.print("=!track ");
        CLI_SERIAL.println(mpu_trackers);
        return;
    }

    if (strcmp(subcmd, "!track") == 0) {
        mpu_track();
        CLI_SERIAL.println("=!track");
        return;
    }

    if (strcmp(subcmd, "!untrack") == 0) {
        mpu_untrack();

        CLI_SERIAL.println("=!untrack");
        return;
    }

    if (strcmp(subcmd, "pid") == 0) {
        CLI_SERIAL.print("=pid ");
        CLI_SERIAL.println(mpu_proc.pid);
        return;
    }

    CLI_SERIAL.println("~Unknown mpu subcommand");
}

void _cli_dir_cmd(void*) {
    const char* subcmd = cmd_ogets(&_cli_cmd, 1, "");

    if (strcmp(subcmd, "acal") == 0) {
        const char* state = cmd_ogets(&_cli_cmd, 2, "");

        if (strcmp(state, "begin") == 0) {
            dir_autocal_begin();
            CLI_SERIAL.println("=acal begin");
            return;
        }
        else if (strcmp(state, "end") == 0) {
            bool success = dir_autocal_end();
            CLI_SERIAL.print("=acal end");
            CLI_SERIAL.println(success ? "" : " fail");
            return;
        }
        else {
            CLI_SERIAL.println("~Invalid acal argument (begin/end)");
            return;
        }
    }

    if (strcmp(subcmd, "cal") == 0) {
        dir_cal_t cal = dir_cal_get();

        // Return result as space-separated values in the order x_min x_max y_min y_max z_min z_max
        CLI_SERIAL.print("=cal ");
        CLI_SERIAL.print(cal.x_min);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.print(cal.x_max);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.print(cal.y_min);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.print(cal.y_max);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.print(cal.z_min);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.println(cal.z_max);
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
        CLI_SERIAL.println("=!cal");
        return;
    }

    if (strcmp(subcmd, "heading") == 0) {
        CLI_SERIAL.print("=heading ");
        CLI_SERIAL.println(dir_heading());
        return;
    }

    if (strcmp(subcmd, "raw") == 0) {
        int16_t x, y, z;
        dir_raw(&x, &y, &z);

        CLI_SERIAL.print("=raw ");
        CLI_SERIAL.print(x);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.print(y);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.println(z);
        return;
    }

    if (strcmp(subcmd, "ref") == 0) {
        float ref = dir_ref_get();

        CLI_SERIAL.print("=ref ");
        CLI_SERIAL.println(ref);
        return;
    }

    if (strcmp(subcmd, "!ref") == 0) {
        float ref = cmd_ogetf(&_cli_cmd, 2, 0);

        dir_ref(ref);
        CLI_SERIAL.println("=!ref");
        return;
    }

    if (strcmp(subcmd, "orient") == 0) {
        dir_orient_t orient = dir_orient_get();

        CLI_SERIAL.print("=orient ");
        switch (orient) {
            case DIR_O_YZ:
                CLI_SERIAL.println("yz");
                break;
            case DIR_O_XZ:
                CLI_SERIAL.println("xz");
                break;
            case DIR_O_XY:
            default:
                CLI_SERIAL.println("xy");
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
            CLI_SERIAL.println("~Invalid orient argument (yz/xz/xy)");
            return;
        }

        dir_orient(orient);
        CLI_SERIAL.println("=!orient");
        return;
    }

    if (strcmp(subcmd, "magnet") == 0) {
        bool safe = dir_magnet_safe_get();

        CLI_SERIAL.print("=magnet ");
        CLI_SERIAL.println(safe ? "true" : "false");
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
            CLI_SERIAL.println("~Invalid magnet argument (true/false)");
            return;
        }

        dir_magnet_safe(safe);
        CLI_SERIAL.println("=!magnet");
        return;
    }

    if (strcmp(subcmd, "ready") == 0) {
        CLI_SERIAL.print("=ready ");
        CLI_SERIAL.println(dir_ready() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "state") == 0) {
        CLI_SERIAL.print("=state ");
        CLI_SERIAL.println(dir_state);
        return;
    }

    if (strcmp(subcmd, "!state") == 0) {
        int state = cmd_ogeti(&_cli_cmd, 2, 0);

        if (state < DIR_S_MIN || state > DIR_S_MAX) {
            CLI_SERIAL.println("~Invalid state argument");
            return;
        }

        dir_state = (dir_state_t) state;
        CLI_SERIAL.println("=!state");
        return;
    }

    if (strcmp(subcmd, "track") == 0) {
        CLI_SERIAL.print("=track ");
        CLI_SERIAL.println(dir_trackers);
        return;
    }

    if (strcmp(subcmd, "!track") == 0) {
        dir_track();
        CLI_SERIAL.println("=!track");
        return;
    }

    if (strcmp(subcmd, "!untrack") == 0) {
        dir_untrack();
        CLI_SERIAL.println("=!untrack");
        return;
    }

    if (strcmp(subcmd, "pid") == 0) {
        CLI_SERIAL.print("=pid ");
        CLI_SERIAL.println(dir_proc.pid);
        return;
    }

    CLI_SERIAL.println("~Unknown dir subcommand");
}

void _cli_steps_cmd(void*) {
    const char* subcmd = cmd_ogets(&_cli_cmd, 1, "");

    if (strcmp(subcmd, "acal") == 0) {
        const char* state = cmd_ogets(&_cli_cmd, 2, "");

        if (strcmp(state, "begin") == 0) {
            steps_autocal_begin();
            CLI_SERIAL.println("=acal begin");
            return;
        }
        else if (strcmp(state, "end") == 0) {
            bool success = steps_autocal_end();
            CLI_SERIAL.print("=acal end");
            CLI_SERIAL.println(success ? "" : " fail");
            return;
        }
        else {
            CLI_SERIAL.println("~Invalid acal argument (begin/end)");
            return;
        }
    }

    if (strcmp(subcmd, "cal") == 0) {
        steps_cal_t cal = steps_cal_get();

        // Return result as space-separated values in the order left_min, left_max, right_min, right_max
        CLI_SERIAL.print("=cal ");
        CLI_SERIAL.print(cal.left_min);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.print(cal.left_max);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.print(cal.right_min);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.println(cal.right_max);
        return;
    }

    if (strcmp(subcmd, "!cal") == 0) {
        uint16_t left_min = (uint16_t) cmd_ogeti(&_cli_cmd, 2, 0);
        uint16_t left_max = (uint16_t) cmd_ogeti(&_cli_cmd, 3, 0);
        uint16_t right_min = (uint16_t) cmd_ogeti(&_cli_cmd, 4, 0);
        uint16_t right_max = (uint16_t) cmd_ogeti(&_cli_cmd, 5, 0);

        steps_cal((steps_cal_t){
            .left_min = left_min,
            .left_max = left_max,
            .right_min = right_min,
            .right_max = right_max
        });
        CLI_SERIAL.println("=!cal");
        return;
    }

    if (strcmp(subcmd, "read") == 0) {
        int32_t left, right;
        steps_read(&left, &right);

        CLI_SERIAL.print("=read ");
        CLI_SERIAL.print(left);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.println(right);
        return;
    }

    if (strcmp(subcmd, "raw") == 0) {
        int16_t left, right;
        steps_raw(&left, &right);

        CLI_SERIAL.print("=raw ");
        CLI_SERIAL.print(left);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.println(right);
        return;
    }

    if (strcmp(subcmd, "reset") == 0) {
        bool left = cmd_ugetb(&_cli_cmd, "l", false);
        bool right = cmd_ugetb(&_cli_cmd, "r", false);

        // Default to both if no sides specified
        if (!left && !right) {
            left = true;
            right = true;
        }

        steps_reset(left, right);
        CLI_SERIAL.print("=reset ");
        if (left) CLI_SERIAL.print("l");
        if (right) CLI_SERIAL.print("r");
        CLI_SERIAL.println();
        return;
    }

    if (strcmp(subcmd, "coef") == 0) {
        float coef = steps_coef_get();

        CLI_SERIAL.print("=coef ");
        CLI_SERIAL.println(coef);
        return;
    }

    if (strcmp(subcmd, "!coef") == 0) {
        float coef = cmd_ogetf(&_cli_cmd, 2, 0);

        steps_coef(coef);
        CLI_SERIAL.println("=!coef");
        return;
    }

    if (strcmp(subcmd, "ready") == 0) {
        CLI_SERIAL.print("=ready ");
        CLI_SERIAL.println(steps_ready() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "state") == 0) {
        CLI_SERIAL.print("=state ");
        CLI_SERIAL.println(steps_state);
        return;
    }

    if (strcmp(subcmd, "!state") == 0) {
        int state = cmd_ogeti(&_cli_cmd, 2, 0);

        if (state < STEPS_S_MIN || state > STEPS_S_MAX) {
            CLI_SERIAL.println("~Invalid state argument");
            return;
        }

        steps_state = (steps_state_t) state;
        CLI_SERIAL.println("=!state");
        return;
    }

    if (strcmp(subcmd, "track") == 0) {
        CLI_SERIAL.print("=track ");
        CLI_SERIAL.println(steps_trackers);
        return;
    }

    if (strcmp(subcmd, "!track") == 0) {
        steps_track();
        CLI_SERIAL.println("=!track");
        return;
    }

    if (strcmp(subcmd, "!untrack") == 0) {
        steps_untrack();
        CLI_SERIAL.println("=!untrack");
        return;
    }

    if (strcmp(subcmd, "pid") == 0) {
        CLI_SERIAL.print("=pid ");
        CLI_SERIAL.println(steps_proc.pid);
        return;
    }

    CLI_SERIAL.println("~Unknown steps subcommand");
}

void _cli_mot_cmd(void*) {
    const char* subcmd = cmd_ogets(&_cli_cmd, 1, "");

    if (strcmp(subcmd, "get") == 0) {
        int16_t left, right;
        mot_power_get(&left, &right);

        CLI_SERIAL.print("=get ");
        CLI_SERIAL.print(left);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.println(right);
        return;
    }

    if (strcmp(subcmd, "set") == 0) {
        int16_t left = (int16_t) cmd_ogeti(&_cli_cmd, 2, 0);
        int16_t right = (int16_t) cmd_ogeti(&_cli_cmd, 3, 0);

        mot_power_set(left, right);
        CLI_SERIAL.println("=!set");
        return;
    }

    if (strcmp(subcmd, "stop") == 0) {
        mot_power_set(0, 0);
        CLI_SERIAL.println("=!stop");
        return;
    }

    if (strcmp(subcmd, "override") == 0) {
        CLI_SERIAL.print("=override ");
        CLI_SERIAL.println(mot_user_get() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "!override") == 0) {
        const char* override_str = cmd_ogets(&_cli_cmd, 2, "");

        bool override;
        if (strcmp(override_str, "true") == 0) {
            override = true;
        }
        else if (strcmp(override_str, "false") == 0) {
            override = false;
        }
        else {
            CLI_SERIAL.println("~Invalid override argument (true/false)");
            return;
        }

        mot_user(override);
        CLI_SERIAL.println("=!override");
        return;
    }

    if (strcmp(subcmd, "ready") == 0) {
        CLI_SERIAL.print("=ready ");
        CLI_SERIAL.println(mot_ready() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "state") == 0) {
        CLI_SERIAL.print("=state ");
        CLI_SERIAL.println(mot_state);
        return;
    }

    if (strcmp(subcmd, "!state") == 0) {
        int state = cmd_ogeti(&_cli_cmd, 2, 0);

        if (state < MOT_S_MIN || state > MOT_S_MAX) {
            CLI_SERIAL.println("~Invalid state argument");
            return;
        }

        mot_state = (mot_state_t) state;
        CLI_SERIAL.println("=!state");
        return;
    }

    if (strcmp(subcmd, "pid") == 0) {
        CLI_SERIAL.print("=pid ");
        CLI_SERIAL.println(mot_proc.pid);
        return;
    }

    CLI_SERIAL.println("~Unknown mot subcommand");
}

void _cli_pos_cmd(void*) {
    const char* subcmd = cmd_ogets(&_cli_cmd, 1, "");

    if (strcmp(subcmd, "get") == 0) {
        float x, y;
        pos_get(&x, &y);

        CLI_SERIAL.print("=get ");
        CLI_SERIAL.print(x);
        CLI_SERIAL.print(" ");
        CLI_SERIAL.println(y);
        return;
    }

    if (strcmp(subcmd, "set") == 0) {
        float x = cmd_ogetf(&_cli_cmd, 2, 0);
        float y = cmd_ogetf(&_cli_cmd, 3, 0);

        pos_set(x, y);
        CLI_SERIAL.println("=!set");
        return;
    }

    if (strcmp(subcmd, "ready") == 0) {
        CLI_SERIAL.print("=ready ");
        CLI_SERIAL.println(pos_ready() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "state") == 0) {
        CLI_SERIAL.print("=state ");
        CLI_SERIAL.println(pos_state);
        return;
    }

    if (strcmp(subcmd, "!state") == 0) {
        int state = cmd_ogeti(&_cli_cmd, 2, 0);

        if (state < POS_S_MIN || state > POS_S_MAX) {
            CLI_SERIAL.println("~Invalid state argument");
            return;
        }

        pos_state = (pos_state_t) state;
        CLI_SERIAL.println("=!state");
        return;
    }

    if (strcmp(subcmd, "track") == 0) {
        CLI_SERIAL.print("=track ");
        CLI_SERIAL.println(pos_trackers);
        return;
    }

    if (strcmp(subcmd, "!track") == 0) {
        pos_track();
        CLI_SERIAL.println("=!track");
        return;
    }

    if (strcmp(subcmd, "!untrack") == 0) {
        pos_untrack();
        CLI_SERIAL.println("=!untrack");
        return;
    }

     if (strcmp(subcmd, "pid") == 0) {
        CLI_SERIAL.print("=pid ");
        CLI_SERIAL.println(pos_proc.pid);
        return;
    }

    if (strcmp(subcmd, "pid") == 0) {
        CLI_SERIAL.print("=pid ");
        CLI_SERIAL.println(pos_proc.pid);
        return;
    }

    CLI_SERIAL.println("~Unknown pos subcommand");
}

void _cli_move_cmd(void*) {
    const char* subcmd = cmd_ogets(&_cli_cmd, 1, "");

    if (strcmp(subcmd, "busy") == 0) {
        CLI_SERIAL.print("=busy ");
        CLI_SERIAL.println(move_busy() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "drive") == 0) {
        int16_t power = (int16_t) cmd_ogeti(&_cli_cmd, 2, 0);
        move_drive(power);
        CLI_SERIAL.println("=!drive");
        return;
    }

    if (strcmp(subcmd, "driveby") == 0) {
        int16_t power = (int16_t) cmd_ogeti(&_cli_cmd, 2, 0);
        int32_t distance = cmd_ogeti(&_cli_cmd, 3, 0);
        move_drive_by(power, distance);
        CLI_SERIAL.println("=!driveby");
        return;
    }

    if (strcmp(subcmd, "rotate") == 0) {
        int16_t power = (int16_t) cmd_ogeti(&_cli_cmd, 2, 0);
        move_rotate(power);
        CLI_SERIAL.println("=!rotate");
        return;
    }

    if (strcmp(subcmd, "rotateto") == 0) {
        uint8_t power = (uint8_t) cmd_ogeti(&_cli_cmd, 2, 0);
        float heading = cmd_ogetf(&_cli_cmd, 3, 0);
        move_rotate_to(power, heading);
        CLI_SERIAL.println("=!rotateto");
        return;
    }

    if (strcmp(subcmd, "stop") == 0) {
        move_stop();
        CLI_SERIAL.println("=!stop");
        return;
    }

    if (strcmp(subcmd, "ready") == 0) {
        CLI_SERIAL.print("=ready ");
        CLI_SERIAL.println(move_ready() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "state") == 0) {
        CLI_SERIAL.print("=state ");
        CLI_SERIAL.println(move_state);
        return;
    }

    if (strcmp(subcmd, "!state") == 0) {
        int state = cmd_ogeti(&_cli_cmd, 2, 0);

        if (state < MOVE_S_MIN || state > MOVE_S_MAX) {
            CLI_SERIAL.println("~Invalid state argument");
            return;
        }

        move_state = (move_state_t) state;
        CLI_SERIAL.println("=!state");
        return;
    }

    if (strcmp(subcmd, "pid") == 0) {
        CLI_SERIAL.print("=pid ");
        CLI_SERIAL.println(move_proc.pid);
        return;
    }

    CLI_SERIAL.println("~Unknown move subcommand");
}
