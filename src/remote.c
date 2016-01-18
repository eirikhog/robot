
#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "common.h"
#include "remote_lcd.h"
#include "remote_terminal.h"
#include "remote_menu.h"

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
 * PB5-3: SPI (ISP connection, Display)
 * LCD:
 * PB1: SCE (Chip select)
 * PB2: RST (Reset)
 * PB7: D/C (Mode select)
 * PB5-3: SPI
 */

typedef enum {
    MODE_MANUAL,
    MODE_AUTO,
    MODE_MENU
} RemoteMode;

int main(void) {
    // UART for radio.
    init_uart(9600, F_CPU);

    // LCD Screen
    lcd_init();
    terminal_init();

    // Input
    input_init();
    
    // Set power LED
    DDRB |= (1 << PINB6);
    clear_bit(PORTB, PINB6);

    // Ready to go!
    sei();

    RemoteMode mode = MODE_MANUAL;

    while (1) 
    {
        input_update(&input_state);

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


