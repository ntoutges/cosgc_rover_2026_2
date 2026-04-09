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
            cmd_attach(&_cli_cmd, "pos", _cli_pos_cmd, NULL);

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

        if (state < MPU_S_MIN || state > MPU_S_MAX) {
            Serial.println("~Invalid state argument");
            return;
        }

        mpu_state = (mpu_state_t) state;
        Serial.println("=!state");
        return;
    }

    if (strcmp(subcmd, "track") == 0) {
        Serial.print("=!track ");
        Serial.println(mpu_trackers);
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

    if (strcmp(subcmd, "raw") == 0) {
        int16_t x, y, z;
        dir_raw(&x, &y, &z);

        Serial.print("=raw ");
        Serial.print(x);
        Serial.print(" ");
        Serial.print(y);
        Serial.print(" ");
        Serial.println(z);
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

        if (state < DIR_S_MIN || state > DIR_S_MAX) {
            Serial.println("~Invalid state argument");
            return;
        }

        dir_state = (dir_state_t) state;
        Serial.println("=!state");
        return;
    }

    if (strcmp(subcmd, "track") == 0) {
        Serial.print("=track ");
        Serial.println(dir_trackers);
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

void _cli_steps_cmd(void*) {
    const char* subcmd = cmd_ogets(&_cli_cmd, 1, "");

    if (strcmp(subcmd, "acal") == 0) {
        const char* state = cmd_ogets(&_cli_cmd, 2, "");

        if (strcmp(state, "begin") == 0) {
            steps_autocal_begin();
            Serial.println("=acal begin");
            return;
        }
        else if (strcmp(state, "end") == 0) {
            bool success = steps_autocal_end();
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
        steps_cal_t cal = steps_cal_get();

        // Return result as space-separated values in the order left_min, left_max, right_min, right_max
        Serial.print("=cal ");
        Serial.print(cal.left_min);
        Serial.print(" ");
        Serial.print(cal.left_max);
        Serial.print(" ");
        Serial.print(cal.right_min);
        Serial.print(" ");
        Serial.println(cal.right_max);
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
        Serial.println("=!cal");
        return;
    }

    if (strcmp(subcmd, "read") == 0) {
        int32_t left, right;
        steps_read(&left, &right);

        Serial.print("=read ");
        Serial.print(left);
        Serial.print(" ");
        Serial.println(right);
        return;
    }

    if (strcmp(subcmd, "raw") == 0) {
        int16_t left, right;
        steps_raw(&left, &right);

        Serial.print("=raw ");
        Serial.print(left);
        Serial.print(" ");
        Serial.println(right);
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
        Serial.print("=reset ");
        if (left) Serial.print("l");
        if (right) Serial.print("r");
        Serial.println();
        return;
    }

    if (strcmp(subcmd, "coef") == 0) {
        float coef = steps_coef_get();

        Serial.print("=coef ");
        Serial.println(coef);
        return;
    }

    if (strcmp(subcmd, "!coef") == 0) {
        float coef = cmd_ogetf(&_cli_cmd, 2, 0);

        steps_coef(coef);
        Serial.println("=!coef");
        return;
    }

    if (strcmp(subcmd, "ready") == 0) {
        Serial.print("=ready ");
        Serial.println(steps_ready() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "state") == 0) {
        Serial.print("=state ");
        Serial.println(steps_state);
        return;
    }

    if (strcmp(subcmd, "!state") == 0) {
        int state = cmd_ogeti(&_cli_cmd, 2, 0);

        if (state < STEPS_S_MIN || state > STEPS_S_MAX) {
            Serial.println("~Invalid state argument");
            return;
        }

        steps_state = (steps_state_t) state;
        Serial.println("=!state");
        return;
    }

    if (strcmp(subcmd, "track") == 0) {
        Serial.print("=track ");
        Serial.println(steps_trackers);
        return;
    }

    if (strcmp(subcmd, "!track") == 0) {
        steps_track();
        Serial.println("=!track");
        return;
    }

    if (strcmp(subcmd, "!untrack") == 0) {
        steps_untrack();
        Serial.println("=!untrack");
        return;
    }

    if (strcmp(subcmd, "pid") == 0) {
        Serial.print("=pid ");
        Serial.println(steps_proc.pid);
        return;
    }

    Serial.println("~Unknown steps subcommand");
}

void _cli_mot_cmd(void*) {
    const char* subcmd = cmd_ogets(&_cli_cmd, 1, "");

    if (strcmp(subcmd, "get") == 0) {
        int16_t left, right;
        mot_power_get(&left, &right);

        Serial.print("=get ");
        Serial.print(left);
        Serial.print(" ");
        Serial.println(right);
        return;
    }

    if (strcmp(subcmd, "set") == 0) {
        int16_t left = (int16_t) cmd_ogeti(&_cli_cmd, 2, 0);
        int16_t right = (int16_t) cmd_ogeti(&_cli_cmd, 3, 0);

        mot_power_set(left, right);
        Serial.println("=!set");
        return;
    }

    if (strcmp(subcmd, "stop") == 0) {
        mot_power_set(0, 0);
        Serial.println("=!stop");
        return;
    }

    if (strcmp(subcmd, "override") == 0) {
        Serial.print("=override ");
        Serial.println(mot_user_get() ? "true" : "false");
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
            Serial.println("~Invalid override argument (true/false)");
            return;
        }

        mot_user(override);
        Serial.println("=!override");
        return;
    }

    if (strcmp(subcmd, "ready") == 0) {
        Serial.print("=ready ");
        Serial.println(mot_ready() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "state") == 0) {
        Serial.print("=state ");
        Serial.println(mot_state);
        return;
    }

    if (strcmp(subcmd, "!state") == 0) {
        int state = cmd_ogeti(&_cli_cmd, 2, 0);

        if (state < MOT_S_MIN || state > MOT_S_MAX) {
            Serial.println("~Invalid state argument");
            return;
        }

        mot_state = (mot_state_t) state;
        Serial.println("=!state");
        return;
    }

    if (strcmp(subcmd, "pid") == 0) {
        Serial.print("=pid ");
        Serial.println(mot_proc.pid);
        return;
    }

    Serial.println("~Unknown mot subcommand");
}

void _cli_pos_cmd(void*) {
    const char* subcmd = cmd_ogets(&_cli_cmd, 1, "");

    if (strcmp(subcmd, "get") == 0) {
        float x, y;
        pos_get(&x, &y);

        Serial.print("=get ");
        Serial.print(x);
        Serial.print(" ");
        Serial.println(y);
        return;
    }

    if (strcmp(subcmd, "set") == 0) {
        float x = cmd_ogetf(&_cli_cmd, 2, 0);
        float y = cmd_ogetf(&_cli_cmd, 3, 0);

        pos_set(x, y);
        Serial.println("=!set");
        return;
    }

    if (strcmp(subcmd, "ready") == 0) {
        Serial.print("=ready ");
        Serial.println(pos_ready() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "state") == 0) {
        Serial.print("=state ");
        Serial.println(pos_state);
        return;
    }

    if (strcmp(subcmd, "!state") == 0) {
        int state = cmd_ogeti(&_cli_cmd, 2, 0);

        if (state < POS_S_MIN || state > POS_S_MAX) {
            Serial.println("~Invalid state argument");
            return;
        }

        pos_state = (pos_state_t) state;
        Serial.println("=!state");
        return;
    }

    if (strcmp(subcmd, "track") == 0) {
        Serial.print("=track ");
        Serial.println(pos_trackers);
        return;
    }

    if (strcmp(subcmd, "!track") == 0) {
        pos_track();
        Serial.println("=!track");
        return;
    }

    if (strcmp(subcmd, "!untrack") == 0) {
        pos_untrack();
        Serial.println("=!untrack");
        return;
    }

     if (strcmp(subcmd, "pid") == 0) {
        Serial.print("=pid ");
        Serial.println(pos_proc.pid);
        return;
    }

    if (strcmp(subcmd, "pid") == 0) {
        Serial.print("=pid ");
        Serial.println(pos_proc.pid);
        return;
    }

    Serial.println("~Unknown pos subcommand");
}

void _cli_move_cmd(void*) {
    const char* subcmd = cmd_ogets(&_cli_cmd, 1, "");

    if (strcmp(subcmd, "busy") == 0) {
        Serial.print("=busy ");
        Serial.println(move_busy() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "drive") == 0) {
        float speed = cmd_ogetf(&_cli_cmd, 2, 0);
        move_drive(speed);
        Serial.println("=!drive");
        return;
    }

    if (strcmp(subcmd, "driveby") == 0) {
        int distance = cmd_ogeti(&_cli_cmd, 2, 0);
        float speed = cmd_ogetf(&_cli_cmd, 3, 0);
        move_drive_by(distance, speed);
        Serial.println("=!driveby");
        return;
    }

    if (strcmp(subcmd, "rotate") == 0) {
        float speed = cmd_ogetf(&_cli_cmd, 2, 0);
        move_rotate(speed);
        Serial.println("=!rotate");
        return;
    }

    if (strcmp(subcmd, "rotateto") == 0) {
        float heading = cmd_ogetf(&_cli_cmd, 2, 0);
        float speed = cmd_ogetf(&_cli_cmd, 3, 0);
        move_rotate_to(heading, speed);
        Serial.println("=!rotateto");
        return;
    }

    if (strcmp(subcmd, "stop") == 0) {
        bool force = cmd_ugetb(&_cli_cmd, "f", false);
        move_stop(force);
        Serial.println("=!stop");
        return;
    }

    if (strcmp(subcmd, "ramp") == 0) {
        float ramp = move_ramp_get();

        Serial.print("=ramp ");
        Serial.println(ramp);
        return;
    }

    if (strcmp(subcmd, "!ramp") == 0) {
        float accel = cmd_ogetf(&_cli_cmd, 2, 0);

        move_ramp(accel);
        Serial.println("=!ramp");
        return;
    }

    if (strcmp(subcmd, "plan") == 0) {
        const char* plan_str = cmd_ogets(&_cli_cmd, 2, "");

        bool plan;
        if (strcmp(plan_str, "true") == 0) {
            plan = true;
        }
        else if (strcmp(plan_str, "false") == 0) {
            plan = false;
        }
        else {
            Serial.println("~Invalid plan argument (true/false)");
            return;
        }

        move_plan(plan);
        Serial.println("=!plan");
        return;
    }

    if (strcmp(subcmd, "motors") == 0) {
        bool left = cmd_ugetb(&_cli_cmd, "l", false);
        bool right = cmd_ugetb(&_cli_cmd, "r", false);

        if (!left && !right) {
            left = true;
            right = true;
        }

        move_mstate_t left_mstate;
        move_mstate_t right_mstate;
        move_motors_get(&left_mstate, &right_mstate);

        // Print format: "<side>: (<initial>) - 0 - <acc_pos> - (max_speed) - <dec_pos> - <final_pos> - (0)"
        // Print example (l: (0) - 0 - 100 - (100) - 300 - 400 - (0))

        if (left) {
            Serial.print("=motors l: (");
            Serial.print(left_mstate.plan.initial_speed);
            Serial.print(") - 0 - ");
            Serial.print(left_mstate.plan.acc_pos);
            Serial.print(" - (");
            Serial.print(left_mstate.target.speed);
            Serial.print(") - ");
            Serial.print(left_mstate.plan.dec_pos);
            Serial.print(" - ");
            Serial.print(left_mstate.target.pos);
            Serial.println(" - (0)");
        }

        if (right) {
            Serial.print("=motors r: (");
            Serial.print(right_mstate.plan.initial_speed);
            Serial.print(") - 0 - ");
            Serial.print(right_mstate.plan.acc_pos);
            Serial.print(" - (");
            Serial.print(right_mstate.target.speed);
            Serial.print(") - ");
            Serial.print(right_mstate.plan.dec_pos);
            Serial.print(" - ");
            Serial.print(right_mstate.target.pos);
            Serial.println(" - (0)");
        }

        return;
    }

    if (strcmp(subcmd, "kp") == 0) {
        int32_t kp, ki, kd;
        move_pid_get(&kp, &ki, &kd);
        Serial.print("=kp ");
        Serial.print(kp);
        Serial.print(" ");
        Serial.print(ki);
        Serial.print(" ");
        Serial.println(kd);
        return;
    }

    if (strcmp(subcmd, "!kp") == 0) {
        int32_t kp = cmd_ogeti(&_cli_cmd, 2, 0);
        int32_t ki = cmd_ogeti(&_cli_cmd, 3, 0);
        int32_t kd = cmd_ogeti(&_cli_cmd, 4, 0);
        move_pid(kp, ki, kd);
        Serial.println("=!kp");
        return;
    }

    if (strcmp(subcmd, "minpwm") == 0) {
        Serial.print("=minpwm ");
        Serial.println(move_min_pwm_get());
        return;
    }

    if (strcmp(subcmd, "!minpwm") == 0) {
        uint8_t val = (uint8_t) cmd_ogeti(&_cli_cmd, 2, 0);
        move_min_pwm(val);
        Serial.println("=!minpwm");
        return;
    }

    if (strcmp(subcmd, "sync") == 0) {
        Serial.print("=sync ");
        Serial.println(move_sync_get());
        return;
    }

    if (strcmp(subcmd, "!sync") == 0) {
        int32_t kp = cmd_ogeti(&_cli_cmd, 2, 0);
        move_sync(kp);
        Serial.println("=!sync");
        return;
    }

    if (strcmp(subcmd, "headingtol") == 0) {
        Serial.print("=headingtol ");
        Serial.println(move_heading_tol_get());
        return;
    }

    if (strcmp(subcmd, "!headingtol") == 0) {
        float tol = cmd_ogetf(&_cli_cmd, 2, 0);
        move_heading_tol(tol);
        Serial.println("=!headingtol");
        return;
    }

    if (strcmp(subcmd, "ready") == 0) {
        Serial.print("=ready ");
        Serial.println(move_ready() ? "true" : "false");
        return;
    }

    if (strcmp(subcmd, "state") == 0) {
        Serial.print("=state ");
        Serial.println(move_state);
        return;
    }

    if (strcmp(subcmd, "!state") == 0) {
        int state = cmd_ogeti(&_cli_cmd, 2, 0);

        if (state < MOVE_S_MIN || state > MOVE_S_MAX) {
            Serial.println("~Invalid state argument");
            return;
        }

        move_state = (move_state_t) state;
        Serial.println("=!state");
        return;
    }

    if (strcmp(subcmd, "pid") == 0) {
        Serial.print("=pid ");
        Serial.println(move_proc.pid);
        return;
    }

    Serial.println("~Unknown move subcommand");
}
