#include <hardware/watchdog.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>
#include "Adafruit_PWMServoDriver.h"
#include "amt232.h"
#include "encoderBase.h"
#include "gpio.h"
#include "gpioex.h"
#include "motor.h"
#include "pico/multicore.h"
#include "pwm.h"
#include "qenc.h"
#include "servo.h"
#include "stepper.h"

const uint CS_PIN = 25;
AMT232 amt232(CS_PIN, 3, 0, 2);
Qenc enc(12);
Gpio cs0(25, OUTPUT);
Gpio dum0(1, OUTPUT);
Gpio dum1(6, OUTPUT);
Gpio sw0(22, INPUT_PD);
Gpio sw1(23, INPUT_PD);
Gpio sw2(7, INPUT_PD);
Gpio sw3(15, INPUT_PD);
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
Gpio stp_dir(14, OUTPUT);
Stepper stepper(stp, slp, stp_dir);
int servoState = 0;
int armtheta = 0;
int hand = 0;
float degpos[2] = {0, 0};
int stepperState = 0;
char buf[255] = "";
static repeating_timer_t timer;
static repeating_timer_t timer1;
static repeating_timer_t timer2;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600   // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600      // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400     // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

void sw0_cb(uint gpio, uint32_t events) {
    switch (gpio) {
        case 22:
            motor[1].reset();
            break;
        case 23:
            printf("SW1 Interrupt\n");
            motor[1].disablePosPid();
            motor[1].reset();
            motor[1].enc.set(61667);
            motor[1].setPos(542);
    }
}

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

void setServoAngle(int servoNum, float deg) {
    if (deg < 0)
        deg = 0;
    if (deg > 180)
        deg = 180;
    pwm.writeMicroseconds(servoNum, int((deg * (2600 - 560) / 180.0) + 560));
    pwm.writeMicroseconds(servoNum, int((deg * (2600 - 560) / 180.0) + 560));
    pwm.writeMicroseconds(servoNum, int((deg * (2600 - 560) / 180.0) + 560));
}

void reset() {
    watchdog_enable(1, 1);
    while (1)
        ;
}

void initTimer() {
    add_repeating_timer_ms(-10, timer_cb, NULL, &timer);
    add_repeating_timer_ms(-100, timer_cb_pos, NULL, &timer1);
    add_repeating_timer_ms(-2, timer_cb_stp, NULL, &timer2);
}

void serial_read() {
    while (1) {
        int cnt = 0;
        while (1) {
            char c = getchar_timeout_us(100000000000);
            buf[cnt] = c;
            if (c == '\n') {
                cnt = 0;
                sscanf(buf, "%d %d %d %d %f %f\n", &stepperState, &servoState, &hand, &armtheta, &degpos[0], &degpos[1]);
                memset(buf, '\0', 255);
                break;
            }
            if (c == 'r') {
                // reset microcontroller
                reset();
            }
            cnt++;
        }
    }
}

void catchObject() {
    setServoAngle(1, 55);
    sleep_ms(10);
    setServoAngle(2, 50);
    sleep_ms(10);
    setServoAngle(3, 40);
}

void releaseObject() {
    setServoAngle(1, 115);
    sleep_ms(10);
    setServoAngle(2, 110);
    sleep_ms(10);
    setServoAngle(3, 100);
}

void homing() {
    printf("homing\n");
    motor[1].duty(-0.1);
    while (1) {
        if (!sw0.read()) {
            motor[1].reset();
            break;
        }
    }
    stp_dir.write(1);
    while (1) {
        stp.write(1);
        sleep_ms(2);
        stp.write(0);
        sleep_ms(2);
        if (!sw2.read()) {
            break;
        }
    }
}

int main() {
    stdio_init_all();
    while (1) {
        static char buf[255];
        int cnt = 0;
        while (1) {
            char c = getchar_timeout_us(100000000000);
            buf[cnt] = c;
            if (c == '\n') {
                printf("%s", buf);
                cnt = 0;
                // sscanf(buf, "%c\n", &c);
                // memset(buf, '\0', 255);
                break;
            }
            cnt++;
        }
        if (buf[0] == 's') {
            break;
        }
    }
    multicore_launch_core1(serial_read);
    sleep_ms(10);
    sw0.init();
    sw1.init();
    sw2.init();
    sw3.init();
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
    stp_dir.init();
    slp.write(1);
    servo0.init();
    sw0.init();
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
    motor[0].setPos(0);
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

    // motor[0].duty(0.5);
    // motor[1].duty(0.1);

    homing();
    gpio_set_irq_enabled_with_callback(22, GPIO_IRQ_EDGE_FALL, true, &sw0_cb);
    gpio_set_irq_enabled_with_callback(23, GPIO_IRQ_EDGE_FALL, true, &sw0_cb);
    timer_cb(nullptr);
    timer_cb_pos(nullptr);
    initTimer();

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

    servo0.write(0);

    // setServoAngle(0, 45s);

    while (1) {
        static int cnt = 0;
        motor[0].setPos(-degpos[0]);
        if (degpos[1] < 0) {
            degpos[1] = 0;
        } else if (degpos[1] > 542) {
            degpos[1] = 542;
        }
        motor[1].setPos(degpos[1] * 360 / 72 < 0 ? 0 : degpos[1] * 360 / 72);
        printf("%f, %f, %d, %d, %d, %d, %f, %f，%d, %d, %d, %d, %d, %d\n",
               -motor[0].getCurrent(),
               motor[1].getCurrent() * 72 / 360.0,
               servoState, 
               hand, 
               armtheta, 
               degpos[0], 
               degpos[1], 
               amt232.getRaw(), 
               enc.get(), 
               sw0.read(), 
               sw1.read(), 
               sw2.read(), 
               sw3.read()
            );
        switch (stepperState) {
            case 0:
                stepper.setTargetMillimeter(0);
                break;
            case 1:
                stepper.setTargetMillimeter(150);
                break;
            case 2:
                stepper.setTargetMillimeter(250);
                break;
            default:
                break;
        }
        switch (servoState) {
            case 0:
                catchObject();
                break;

            case 1:
                releaseObject();
                break;
            default:
                break;
        }
        switch (hand) {
            case 0:
                setServoAngle(0, 40);
                break;
            case 1:
                setServoAngle(0, 0);
                break;
            default:
                break;
        }
        setServoAngle(7, armtheta);
        // setServoAngle(7, 0);
        cnt++;
        sleep_ms(1);
        // while (1) {
        //     setServoAngle(1, 30);
        //     sleep_ms(10);
        //     setServoAngle(2, 35);
        //     sleep_ms(10);
        //     setServoAngle(3, 85);
        //     sleep_ms(1000);
        //     setServoAngle(1, 90);
        //     sleep_ms(10);
        //     setServoAngle(2, 90);
        //     sleep_ms(10);
        //     setServoAngle(3, 40);
        //     sleep_ms(2000);
        // }
    }
}