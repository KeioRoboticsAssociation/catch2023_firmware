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
Gpio sw3(15, OUTPUT);
static repeating_timer_t timer;
static repeating_timer_t timer1;
static repeating_timer_t timer2;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600   // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600      // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400     // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

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

void setServoMs(int servoNum, float ms) {
    pwm.writeMicroseconds(servoNum, int(ms));
    pwm.writeMicroseconds(servoNum, int(ms));
    pwm.writeMicroseconds(servoNum, int(ms));
}

void mg996r(int servoNum, float deg) {
    static int prevDeg;
    pwm.writeMicroseconds(servoNum, 0);
    pwm.writeMicroseconds(servoNum, 0);
    pwm.writeMicroseconds(servoNum, 0);
    if (deg < 0)
        deg = 0;
    if (deg > 180)
        deg = 180;

    if (deg - prevDeg > 0) {
        pwm.writeMicroseconds(servoNum, int((deg * 9.822 + 600)));
        pwm.writeMicroseconds(servoNum, int((deg * 9.822 + 600)));
        pwm.writeMicroseconds(servoNum, int((deg * 9.822 + 600)));
    } else {
        pwm.writeMicroseconds(servoNum, int((deg * 10 + 550)));
        pwm.writeMicroseconds(servoNum, int((deg * 10 + 550)));
        pwm.writeMicroseconds(servoNum, int((deg * 10 + 550)));
    }
    prevDeg = deg;
}

void deg45(int servoNum, float deg) {
    if (deg < 0)
        deg = 0;
    if (deg > 180)
        deg = 180;
    pwm.writeMicroseconds(servoNum, int((deg * (2600 - 560) / 180.0) + 560));
    pwm.writeMicroseconds(servoNum, int((deg * (2600 - 560) / 180.0) + 560));
    pwm.writeMicroseconds(servoNum, int((deg * (2600 - 560) / 180.0) + 560));
}

void initTimer() {
    // add_repeating_timer_ms(-10, timer_cb, NULL, &timer);
    // add_repeating_timer_ms(-100, timer_cb_pos, NULL, &timer1);
    add_repeating_timer_us(-500, timer_cb_stp, NULL, &timer2);
}
int servoPulseLength = 0;

void serial_read() {
    while (1) {
        int cnt = 0;
        while (1) {
            char c = getchar_timeout_us(100000000000);
            buf[cnt] = c;
            if (c == '\n') {
                cnt = 0;
                sscanf(buf, "%d\n", &servoPulseLength);
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
    sw3.init();
    sw3.write(1);
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
    
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

    servo0.write(0);

    // setServoAngle(0, 45s);

    while (1) {
        int servoNum = 1;
        // setServoAngle(1, servoPulseLength);
        // setServoAngle(2, servoPulseLength);
        // setServoAngle(3, servoPulseLength);
        // deg45(servoNum, servoPulseLength);
        setServoMs(5, servoPulseLength);
        printf("%d\n", servoPulseLength);
        sleep_ms(1000);
    }

    while (1) {
        setServoAngle(1, 30);
        sleep_ms(10);
        setServoAngle(2, 35);
        sleep_ms(10);
        setServoAngle(3, 85);
        sleep_ms(1000);
        setServoAngle(1, 90);
        sleep_ms(10);
        setServoAngle(2, 90);
        sleep_ms(10);
        setServoAngle(3, 40);
        sleep_ms(2000);
    }
}