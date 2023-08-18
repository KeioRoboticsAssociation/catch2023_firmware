#include <pico/stdlib.h>
#include <stdio.h>
#include "amt232.h"
#include "encoderBase.h"
#include "gpio.h"
#include "gpioex.h"
#include "motor.h"
#include "pwm.h"
#include "qenc.h"

const uint CS_PIN = 25;
AMT232 amt232(CS_PIN, 3, 0, 2);
Qenc enc(12);
Gpio cs0(25, OUTPUT);
Gpio dum0(1, OUTPUT);
Gpio dum1(6, OUTPUT);
Pwm pwm0(19, 10000, 1);
Pwm pwm1(16, 10000, 1);
Gpio dir0(20, OUTPUT);
Gpio dir1(17, OUTPUT);
Motor motor[2] = {
    Motor(dir0, pwm0, amt232, dum0),
    Motor(dir1, pwm1, enc, dum1)};
Gpio stp(8, OUTPUT);
Gpio slp(28, OUTPUT);

bool timer_cb(repeating_timer_t* rt) {
    // motor[0].timer_cb();
    motor[1].timer_cb();
    return true;
}

bool timer_cb_pos(repeating_timer_t* rt) {
    // motor[0].timer_cb_pos();
    motor[1].timer_cb_pos();
    return true;
}

void initTimer() {
    static repeating_timer_t timer;
    static repeating_timer_t timer1;
    static repeating_timer_t timer2;
    add_repeating_timer_ms(-10, timer_cb, NULL, &timer);
    add_repeating_timer_ms(-100, timer_cb_pos, NULL, &timer1);
}

int main() {
    stdio_init_all();

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

    ex.mode(0, OUTPUT);
    ex.mode(1, OUTPUT);
    ex.mode(2, OUTPUT);
    ex.mode(3, OUTPUT);
    ex.mode(4, OUTPUT);
    ex.mode(5, INPUT_PU);
    ex.mode(6, INPUT_PU);
    ex.mode(7, INPUT_PU);

    ex.set();
    ex.write(0, 1);
    ex.write(1, 1);
    slp.write(1);

    while (1) {
        for (int i = 0; i < 250; i++) {
            ex.write(1, 1);
            stp.write(1);
            sleep_us(10);
            stp.write(0);
            sleep_us(2000);
        }
        sleep_ms(1000);
        for (int i = 0; i < 250; i++) {
            ex.write(1, 0);
            stp.write(1);
            sleep_us(10);
            stp.write(0);
            sleep_us(2000);
        }
        sleep_ms(1000);
    }

    motor[0].init();
    motor[0].setPosGain(3.8, 0.12, 1.2);
    // motor[0].disablePosPid();
    sleep_ms(10);
    motor[1].init(1);
    motor[1].setPosGain(4.0, 0, 0);
    motor[1].setMaxSpeed(2000);
    // motor[1].disablePosPid();
    // motor[0].duty(0.1);
    // motor[1].duty(0.1);
    sleep_ms(10);
    amt232.init();
    initTimer();
    while (1) {
        // motor[0].setVel(90);
        motor[0].setPos(90);
        // motor[1].setVel(-50);
        motor[1].setPos(-1125);
        // tx_data[0] = 0x0c;
        // tx_data[1] = 0x00;

        // dum0.write(0);
        // spi_write_read_blocking(spi0, tx_data, rx_data, 2);
        // dum0.write(1);
        // printf("data: %x, %x\n", rx_data[0], rx_data[1]);
        // printf("%d, %d, %d\n",amt232.get(),amt232.getRaw(),amt232.get()%4096);
        // printf("%d\n", enc.get());
        // sleep_ms(6000);

        // sleep_ms(6000);
        sleep_ms(10000);
        motor[0].setPos(0);
        motor[1].setPos(0);
        sleep_ms(10000);
    }
}