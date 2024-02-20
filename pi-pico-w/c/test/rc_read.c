////////////////// Includes //////////////////

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/cyw43_arch.h" // WiFi chip on Pico W
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "hardware/clocks.h"


////////////////// Settings //////////////////

#define RC_THROTTLE 2
#define RC_ROLL     0
#define RC_PITCH    1
#define RC_YAW      3
#define RC_SWA      4
#define RC_SWB      5

/* Pin placement */
#define PIN_LED     0 // The LED pin is tied to CYW43
#define PIN_RC_RX   5
#define PIN_RC_TX   4


////////////////// Global variables //////////////////

float normalised_rc_values[6];


////////////////// Defines //////////////////

void rc_read() {
    /* 
    An iBus packet comprises 32 bytes:
        - 2 header bytes (first header is 0x20, second is 0x40)
        - 28 channel bytes (2 bytes per channel value)
        - 2 checksum bytes (1 checksum value)

    The protocol data rate is 115200 baud. A packet is transmitted every 7 ms.
    Each packet takes 32*8/115200 seconds to transmit (about 2.22 ms).
    Checksum = 0xFFFF - sum of first 30 bytes
    The channels' values are transmitted sequentially in little endian byte order.
    */
   for (int attempt = 0; attempt < 6; ++attempt) {
        if (uart_is_readable(uart1)) {
            uint8_t buffer[30];

            // Read start bytes
            uint8_t char1 = uart_getc(uart1);
            uint8_t char2 = uart_getc(uart1);

            // Validate start bytes
            if (char1 == 0x20 && char2 == 0x40) {
                // Read the rest of the data into the buffer
                uart_read_blocking(UART_ID, buffer, RC_BUFFER);

                uint16_t checksum = 0xFF9F; // 0xFFFF - 0x20 - 0x40

                // Validate checksum
                for (int byte_index = 0; byte_index < 28; ++byte_index) {
                    checksum -= buffer[byte_index];
                }

                if (checksum == (buffer[29] << 8) | buffer[28]) {
                    int raw_rc_values[6];
                    for (int channel = 0; channel < 6; ++channel) {
                        raw_rc_values[channel] = (buffer[channel * 2 + 1] << 8) + buffer[channel * 2];
                    }

                    // Normalise data
                    normalised_rc_values[RC_THROTTLE] = (float)(raw_rc_values[RC_THROTTLE] * 0.001 - 1); // Normalize from 1000-2000 to 0.0-1.0
                    normalised_rc_values[RC_ROLL] = (float)((raw_rc_values[RC_ROLL] - 1500) * 0.002);    // Normalize from 1000-2000 to -1-1
                    normalised_rc_values[RC_PITCH] = (float)((raw_rc_values[RC_PITCH] - 1500) * 0.002);  // Normalize from 1000-2000 to -1-1
                    normalised_rc_values[RC_YAW] = (float)-((raw_rc_values[RC_YAW] - 1500) * 0.002);      // Normalize from 1000-2000 to -1-1
                    normalised_rc_values[RC_SWA] = (float)(raw_rc_values[RC_SWA] * 0.001 - 1);    // Normalize from 1000-2000 to 0-1
                    normalised_rc_values[RC_SWB] = (float)(raw_rc_values[RC_SWB] * 0.001 - 1);    // Normalize from 1000-2000 to 0-1

                    break;
                }
            }
        }
   }
}


////////////////// Setup //////////////////

int setup() {
    int fail_flag = 0;
    uint64_t setup_start = time_us_64();
    uint64_t setup_delay = setup_start + 5000000;
    stdio_init_all();

    while (time_us_64()  < setup_delay);

    // Overclock RP2040
    set_sys_clock_khz(250000, true);
    if (clock_get_hz(clk_sys) == 250000000) {
        printf("INFO  >>>>   Overclock RP2040 to 250 MHz --> SUCCESS\n\n");
    }
    else {
        fail_flag = 1;
        printf("ERROR >>>>   Overclock RP2040 to 250 MHz --> FAIL\n\n");
    }

    // Initialise WiFi chip for LED
    if (cyw43_arch_init() == 0) {
        printf("INFO  >>>>   Initialise WiFi chip --> SUCCESS\n\n");
    }
    else {
        fail_flag = 1;
        printf("ERROR >>>>   Initialise WiFi chip --> FAIL\n\n");
    }

    // Initialise UART for RC
    int baud = uart_init(uart1, 115200);
    if (baud > 115100 && baud < 115300) {
        gpio_set_function(PIN_RC_RX, GPIO_FUNC_UART);
        gpio_set_function(PIN_RC_TX, GPIO_FUNC_UART);
        printf("INFO  >>>>   Initialise UART --> SUCCESS\n\n");
    }
    else {
        fail_flag = 1;
        printf("ERROR >>>>   Initialise UART --> FAIL\n\n");
    }

    return fail_flag;
}


////////////////// Main //////////////////

int main() {
    uint64_t start_timestamp = time_us_64();

    printf("INFO  >>>>   Executing setup sequence.\n\n");
    if (setup() == 0) {
        printf("INFO  >>>>   Setup completed in %f seconds, looping.\n\n", ((double)(time_us_64() - start_timestamp)/1000000 - 5));

        ////////////////// Loop //////////////////
        while (true) {
            start_timestamp = time_us_64();

            rc_read();
            printf("%d %d %d %d %d %d\n", )

            while (time_us_64() - start_timestamp < 4000); // Do nothing until 4 ms has passed since loop start
        }

    }
}
