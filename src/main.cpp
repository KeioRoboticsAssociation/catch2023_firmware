#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>
#include "amt232.h"
#include "encoderBase.h"
#include "gpio.h"
#include "gpioex.h"
#include "motor.h"
#include "pico/multicore.h"
#include "pwm.h"
#include "qenc.h"
#include "stepper.h"
#include  "servo.h"

const uint CS_PIN = 25;
AMT232 amt232(CS_PIN, 3, 0, 2);
Qenc enc(12);
Gpio cs0(25, OUTPUT);
Gpio dum0(1, OUTPUT);
Gpio dum1(6, OUTPUT);
Pwm pwm0(19, 10000, 1);
Pwm pwm1(16, 10000, 1);
Servo servo0(22);
Gpio dir0(20, OUTPUT);
Gpio dir1(17, OUTPUT);
Motor motor[2] = {
    Motor(dir0, pwm0, amt232, dum0),
    Motor(dir1, pwm1, enc, dum1)};
Gpio stp(8, OUTPUT);
Gpio slp(28, OUTPUT);
Gpio stp_dir(1, OUTPUT, EX);
Stepper stepper(stp, slp, stp_dir);
float degpos[2] = {0, 0};
int stepperState = 0;
char buf[255] = "";
static repeating_timer_t timer;
static repeating_timer_t timer1;
static repeating_timer_t timer2;

bool timer_cb(repeating_timer_t* rt) {
    motor[0].timer_cb();
    motor[1].timer_cb();
    return true;
}

bool timer_cb_pos(repeating_timer_t* rt) {
    motor[0].timer_cb_pos();
    motor[1].timer_cb_pos();
    return true;
}

bool timer_cb_stp(repeating_timer_t* rt) {
    stepper.timer_cb();
    return true;
}

void initTimer() {
    // add_repeating_timer_ms(-10, timer_cb, NULL, &timer);
    // add_repeating_timer_ms(-100, timer_cb_pos, NULL, &timer1);
    add_repeating_timer_us(-500, timer_cb_stp, NULL, &timer2);
}

void serial_read() {
    while (1) {
        int cnt = 0;
        while (1) {
            char c = getchar_timeout_us(100000000000);
            buf[cnt] = c;
            if (c == '\n') {
                cnt = 0;
                sscanf(buf, "%1d %f %f\n", &stepperState, &degpos[0], &degpos[1]);
                memset(buf, '\0', 255);
                break;
            }
            cnt++;
        }
    }
}

int main() {
    stdio_init_all();
    multicore_launch_core1(serial_read);
    sleep_ms(10);
    cs0.init();
    dum0.init();
    dum1.init();
    cs0.write(1);
    dum0.write(1);
    dum1.write(1);
    slp.init();
    stp.init();
    slp.write(1);
    ex.init();
    slp.write(1);
    servo0.init();
    // // ex.mode(0, OUTPUT);
    ex.mode(1, OUTPUT);
    // // ex.mode(2, OUTPUT);
    // // ex.mode(3, OUTPUT);
    // // ex.mode(4, OUTPUT);
    // // ex.mode(5, INPUT_PU);
    // // ex.mode(6, INPUT_PU);
    // // ex.mode(7, INPUT_PU);

    ex.set();
    // // ex.write(0, 1);
    ex.write(1, 1);
    // // slp.write(1);

    motor[0].init();
    motor[0].setPosGain(3.8, 0.12, 1.2);
    // // motor[0].disablePosPid();
    sleep_ms(10);
    motor[1].init(1);
    motor[1].setPosGain(4.0, 0, 0);
    motor[1].setMaxSpeed(2000);
    // // motor[1].disablePosPid();
    sleep_ms(10);
    stepper.init();
    stepper.sleep(1);
    stepper.setPeriod(2);
    amt232.init();
    // initTimer();
    servo0.write(0);
    while(1);
    while (1) {
        servo0.write(38);
        // servo0.write(38);
        sleep_ms(1000);
        servo0.write(90);
        // servo0.write(90);
        sleep_ms(1000);
        // printf("%d, %f, %f\n", stepperState, degpos[0], degpos[1]);
        // switch (stepperSu2tate) {
        //     case 0:
        //         stepper.disable();
        //         stepper.setTargetMillimeter(0);
        //         stepper.enable();
        //         break;
        //     case 1:
        //         stepper.disable();
        //         stepper.setTargetMillimeter(150);
        //         stepper.enable();
        //         break;
        //     case 2:
        //         stepper.disable();
        //         stepper.setTargetMillimeter(300);
        //         stepper.enable();
        //         break;
        //     default:
        //         break;
        // }

        // motor[0].setPos(-degpos[0]);
        // motor[1].setPos(-degpos[1] * 360 / 72);
        // sleep_ms(1);
        // printf("%d, %d, %d\n",amt232.get(),amt232.getRaw(),amt232.get()%4096);
        // printf("%d\n", enc.get());
    }
}