#include <hardware/flash.h>
#include <hardware/watchdog.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

uint8_t config[FLASH_PAGE_SIZE];

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

int main() {
    stdio_init_all();
    uint8_t data[FLASH_PAGE_SIZE];
    printf("Hello, world!\n");
    data[0] = 132;
    // writeFlash(config);
    loadFlash(data);
    while (1) {
        printf("%d\n", data[0]);
        sleep_ms(1000);
    }
}