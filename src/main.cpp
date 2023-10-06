#include <hardware/flash.h>
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
// const int STP_POS_0 = 0;
// const int STP_POS_1 = 40;
// const int STP_POS_2 = 205;
// const int STP_POS_3 = 220;
// const int STP_POS_4 = 250;
// const int STP_POS_5 = 270;
uint8_t config[FLASH_PAGE_SIZE] = {0};

AMT232 amt232(CS_PIN, 3, 0, 2);
Qenc enc(12);
Gpio cs0(25, OUTPUT);
Gpio dum0(1, OUTPUT);
Gpio dum1(6, OUTPUT);
Gpio sw0(22, INPUT_PD);
Gpio sw1(23, INPUT_PD);
Gpio sw2(7, INPUT_PD);
Gpio sw3(15, OUTPUT);
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
int stepperState, currentStepperState = 0;
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

static void writeFlash(uint8_t* data, uint32_t size = FLASH_PAGE_SIZE) {
    const uint32_t FLASH_TARGET_OFFSET = 0x1F0000;
    uint8_t write_data[FLASH_PAGE_SIZE];
    memcpy(write_data, data, size);

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, write_data, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

void loadFlash(uint8_t* data, uint32_t size = FLASH_PAGE_SIZE) {
    const uint32_t FLASH_TARGET_OFFSET = 0x1F0000;
    const uint8_t* flash_target_contents = (const uint8_t*)(XIP_BASE + FLASH_TARGET_OFFSET);
    memcpy(data, flash_target_contents, size);
}

void mg996r(int servoNum, float deg) {
    static int prevDeg;
    // pwm.writeMicroseconds(servoNum, 0);
    // pwm.writeMicroseconds(servoNum, 0);
    // pwm.writeMicroseconds(servoNum, 0);
    if (deg < 0)
        deg = 0;
    if (deg > 180)
        deg = 180;
    // deg*=1.01;
    // if (deg - prevDeg > 0) {
    //     pwm.writeMicroseconds(servoNum, int((deg * 9.822 + 600)));
    //     pwm.writeMicroseconds(servoNum, int((deg * 9.822 + 600)));
    //     pwm.writeMicroseconds(servoNum, int((deg * 9.822 + 600)));
    // } else {
    // int width = int(-0.0002*deg*deg*deg+0.0494*deg*deg+7.037*deg+550);
    // int width = int(deg * 10.02 + 550);
    int width = int(9.9333 * deg + 510);
    pwm.writeMicroseconds(servoNum, width);
    pwm.writeMicroseconds(servoNum, width);
    pwm.writeMicroseconds(servoNum, width);
    // }
    // prevDeg = deg;
}

void reset() {
    watchdog_enable(1, 1);
    while (1)
        ;
}

void initTimer() {
    add_repeating_timer_ms(-10, timer_cb, NULL, &timer);
    add_repeating_timer_ms(-100, timer_cb_pos, NULL, &timer1);
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
                sscanf(buf, "%d %d %d %d %f %f\n", &stepperState, &servoState, &hand, &armtheta, &degpos[0], &degpos[1]);
                memset(buf, '\0', 255);
                break;
            }
            if (c == 'r') {
                // reset microcontroller
                sw3.write(0);
                reset();
            }
            cnt++;
        }
    }
}

void catchObject(int state) {
    if (state >> 2 & 0b1) {
        setServoAngle(1, 78);
    } else {
        setServoAngle(1, 8);
    }
    sleep_ms(10);
    if (state >> 1 & 0b1) {
        setServoAngle(2, 60);
    } else {
        setServoAngle(2, 0);
    }
    sleep_ms(10);
    if (state >> 0 & 0b1) {
        setServoAngle(3, 60);
    } else {
        setServoAngle(3, 0);
    }
}

void releaseObject(int state) {
    if (state >> 2 & 0b1) {
        setServoAngle(1, 8);
    }
    sleep_ms(10);
    if (state >> 1 & 0b1) {
        setServoAngle(2, 0);
    }
    sleep_ms(10);
    if (state >> 0 & 0b1) {
        setServoAngle(3, 0);
    }
}

void homing() {
    printf("homing\n");
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
    printf("Stepper homing done\n");
    motor[1].duty(-0.1);
    while (1) {
        if (!sw0.read()) {
            motor[1].reset();
            break;
        }
    }
    printf("Motor[1] homing done\n");
}

int main() {
    stdio_init_all();

    sw3.init();

    sleep_ms(10);
    sw0.init();
    sw1.init();
    sw2.init();
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
    ex.mode(1, OUTPUT);

    ex.set();
    ex.write(1, 1);
    loadFlash(config);
    amt232.setOffset(config[0] | (config[1] << 8));

    if (!sw1.read()) {
        for (int i = 0; i < 30; i++) {
            printf(".");
            sleep_ms(100);
        }
        printf("\nCalib mode\n");

        printf("Prev offset: %4d\n", config[0] | (config[1] << 8));
        amt232.init();
        int offset = amt232.getRaw();
        amt232.setOffset(offset);
        config[0] = offset & 0xff;
        config[1] = (offset >> 8) & 0xff;
        writeFlash(config);
        printf("New  offset: %4d\n", offset);
        printf("Calib done. Please reset.\n");
        while (1)
            ;
    }

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
    sw3.write(1);

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

    sleep_ms(100);
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
    amt232.init();

    // motor[0].duty(0.5);
    // motor[1].duty(0.1);

    // if(!sw1.read()){
    //     printf("Calib\n");
    //     int offset = amt232.getRaw();
    //     amt232.setOffset(offset);
    //     printf("offset: %d\n", offset);
    //     printf("Calib done. Please reset.\n");
    //     while(1);
    // }

    homing();
    gpio_set_irq_enabled_with_callback(22, GPIO_IRQ_EDGE_FALL, true, &sw0_cb);
    gpio_set_irq_enabled_with_callback(23, GPIO_IRQ_EDGE_FALL, true, &sw0_cb);
    timer_cb(nullptr);
    timer_cb_pos(nullptr);
    initTimer();

    servo0.write(0);

    // setServoAngle(0, 45s);

    while (1) {
        static int cnt = 0;
        motor[0].setPos(-degpos[0]);
        if (degpos[1] < 0) {
            degpos[1] = 0;
        } else if (degpos[1] > 540) {
            degpos[1] = 540;
        }
        motor[1].setPos(degpos[1] * 360 / 72 < 0 ? 0 : degpos[1] * 360 / 72);
        printf("%f, %f, %d, %d, %d, %d, %d, %f, %f，%d, %d, %d, %d, %d\n",
               -motor[0].getCurrent(),
               motor[1].getCurrent() * 72 / 360.0,
               currentStepperState,
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
               sw3.read());
        if (currentStepperState - stepperState < 0) {
            stepper.setPeriod(4);
        } else {
            stepper.setPeriod(8);
        }
        stepper.setTargetMillimeter(stepperState);
        // switch (stepperState) {
        //     case 0:
        //         stepper.setTargetMillimeter(STP_POS_0);
        //         break;
        //     case 1:
        //         stepper.setTargetMillimeter(STP_POS_1);
        //         break;
        //     case 2:
        //         stepper.setTargetMillimeter(STP_POS_2);
        //         break;
        //     case 3:
        //         stepper.setTargetMillimeter(STP_POS_3);
        //         break;
        //     case 4:
        //         stepper.setTargetMillimeter(STP_POS_4);
        //         break;
        //     case 5:
        //         stepper.setTargetMillimeter(STP_POS_5);
        //         break;
        //     case 8:
        //         while (1) {
        //             stp.write(1);
        //             sleep_ms(2);
        //             stp.write(0);
        //             sleep_ms(2);
        //             if (!sw2.read()) {
        //                 break;
        //             }
        //         }
        //         stepper.reset();
        //     default:
        //         break;
        // }
        catchObject(servoState);
        switch (hand) {
            case 0:
                setServoAngle(0, 55);
                break;
            case 1:
                setServoAngle(0, 0);
                break;
            default:
                break;
        }
        // switch (stepper.getPos()) {
        //     case int(STP_POS_0 * 200.0 / (30 * 3.1415)):
        //         currentStepperState = 0;
        //         break;
        //     case int(STP_POS_1 * 200.0 / (30 * 3.1415)):
        //         currentStepperState = 1;
        //         break;
        //     case int(STP_POS_2 * 200.0 / (30 * 3.1415)):
        //         currentStepperState = 2;
        //         break;
        //     case int(STP_POS_3 * 200.0 / (30 * 3.1415)):
        //         currentStepperState = 3;
        //         break;
        //     case int(STP_POS_4 * 200.0 / (30 * 3.1415)):
        //         currentStepperState = 4;
        //         break;
        //     case int(STP_POS_5 * 200.0 / (30 * 3.1415)):
        //         currentStepperState = 5;
        //         break;
        //     default:
        //         break;
        // }
        currentStepperState = stepper.getPos() * (30 * 3.1415) / 200.0;
        mg996r(5, armtheta);
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