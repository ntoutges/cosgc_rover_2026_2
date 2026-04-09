#include <Arduino.h>
#include "csch.h"

#include "mpu.h"
#include "dir.h"
#include "steps.h"
#include "mot.h"
#include "pos.h"
#include "move.h"

#include "cli.h"

csch_t sched;
csch_proc_t proc_buf[8];

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setTimeout(250); // Set I2C timeout to 250ms
    Wire.setWireTimeout(3000, true);

    sched = csch_create(1, millis, proc_buf, sizeof(proc_buf) / sizeof(*proc_buf));

    csch_task_fork(&sched, mpu_csch_tick);
    csch_task_fork(&sched, dir_csch_tick);
    csch_task_fork(&sched, cli_csch_tick);
    csch_task_fork(&sched, steps_csch_tick);
    csch_task_fork(&sched, mot_csch_tick);
    csch_task_fork(&sched, pos_csch_tick);
    csch_task_fork(&sched, move_csch_tick);

    mpu_cal((mpu_cal_t){ 6797, -4173, -32768, -1155, 77, -60 });

    dir_cal((dir_cal_t){ -4155, 480, -6150, -1725, 5018, 6332 });
    dir_orient(DIR_O_XY);

    steps_cal((steps_cal_t){ 0, 884, 0, 884 });

    move_ramp(100); // Set movement acceleration to 100 steps (mm) per second squared

    int32_t next = 0;

    // Run the scheduler
    while (1) {
        csch_tick(&sched);

        // if (millis() > next) {
        //     next += 1000;

        //     // Print out csch process list
        //     Serial.print("0xFF");

        //     uint8_t pid = sched.proc_start;
        //     while (pid != 0xFF) {
        //         csch_proc_t* p = &(sched.proc_buf[pid]);

        //         Serial.print(" -> ");
        //         Serial.print(pid);
        //         Serial.print(" (");
        //         Serial.print(p->tk_queue);
        //         Serial.print(")");

        //         pid = p->next;
        //     }
        //     Serial.println();
        // }
    }
}

void loop() {}