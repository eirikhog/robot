
#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "common.h"
#include "remote_lcd.h"
#include "remote_terminal.h"
#include "remote_menu.h"
#include "remote_nrf24.h"
#include "nrf24.h"

// Global program state, accessible from ISR
static volatile InputState input_state; 

// Unity build
// TODO: Consider if we want to keep this...
#include "remote_lcd.c"
#include "remote_input.c"
#include "remote_mode_manual.c"

/**
 * Remote controller code.
 *
 * The purpuse of this code is to control the user input on the
 * control board, and transmit the data to the robot.
 *
 * Hardware configuration:
 * PB0: Button0
 * PD7: Button1
 * PD6: Button2
 * PD5: Button3
 * PD0-1: UART (XBee radio)
 * PC6: Reset
 * PC0: Joystick (analog)
 * PC1: Joystick (analog)
 * PC2: Joystick (button)
 * LCD:
 * PB1: SCE (Chip select)
 * PB2: RST (Reset)
 * PB7: D/C (Mode select)
 * PB5-3: SPI
 *
 * Radio: (Software SPI)
 * PD0: MOSI
 * PD1: MISO
 * PD2: CE
 * PD3: CSN
 * PD4: SCK
 *
 */

typedef enum {
    MODE_MANUAL,
    MODE_AUTO,
    MODE_MENU
} RemoteMode;

void debug_print_byte(char *prefix, uint8_t byte) {
    char buff[64];
    sprintf(buff, "%s: 0x%02x\n", prefix, byte);
    printf(buff);
}

int main(void) {
    // LCD Screen
    lcd_init();
    terminal_init();

    // Input
    input_init();
    
    // Set power LED
    DDRB |= (1 << PINB6);
    clear_bit(PORTB, PINB6);
    
    // Radio
    uint8_t local_addr[] = { 0x1B, 0x1B, 0x1B, 0x1B, 0x1B };
    uint8_t robot_addr[] = { 0x0A, 0x0A, 0x0A, 0x0A, 0x0A };
    spi_init();
    nrf24_init();
    nrf24_config(47, 2); // Channel 7, 2 bytes
    nrf24_set_rx_addr(local_addr);
    nrf24_set_tx_addr(robot_addr);

    // Ready to go!
    sei();

    RemoteMode mode = MODE_MANUAL;

    while (1) 
    {
        input_update(&input_state);

        if (!(input_state.buttons & BUTTON_LEFT)) {
            // Send some data
            printf("Sending data.\n");
            uint16_t data = 0xDD;
            nrf24_send(&data, sizeof(data));
            int i = 10;
            while (nrf24_is_sending() && --i) {
                printf("Waiting...\n");
            }
            debug_print_byte("R", nrf24_retransmissions());
            debug_print_byte("L", nrf24_last());
            debug_print_byte("C", nrf24_read_register(0x0A));
            debug_print_byte("S", nrf24_status());
            printf("Done!\n");
        }

        switch (mode) {
            case MODE_MANUAL:
                UpdateManualMode(&input_state);
                break;
            case MODE_AUTO:
                asm("nop"::);
                break;
            case MODE_MENU:
                asm("nop"::);
                break;
        }
    }
}


