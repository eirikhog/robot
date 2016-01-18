
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
    MODE_MENU,
    MODE_AUTO
} RemoteMode;


void UpdateRunMode() {

    static uint8_t buttons_prev = 0xFF;
    uint8_t buttons = input_state.buttons;

    if (input_state.joystick.sw) {
        printf("Joystick pressed!\n");
    }

    //char buffer[32];
    //sprintf(buffer, "%u,%u\n", joystick.x, joystick.y);
    //printf(buffer);

    if (button_pressed(BUTTON_UP, buttons, buttons_prev)) {
        printf("BUTTON_UP\n");
        uart_send(RADIO_CMD_FORWARD);
    }
    if (button_pressed(BUTTON_DOWN, buttons, buttons_prev)) {
        printf("BUTTON_DOWN\n");
        uart_send(RADIO_CMD_BACKWARD);
    }
    if (button_pressed(BUTTON_LEFT, buttons, buttons_prev)) {
        printf("BUTTON_LEFT\n");
        uart_send(RADIO_CMD_TURN_LEFT);
    }
    if (button_pressed(BUTTON_RIGHT, buttons, buttons_prev)) {
        printf("BUTTON_RIGHT\n");
        uart_send(RADIO_CMD_TURN_RIGHT);
    }

    buttons_prev = buttons;
}

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

    while (1) 
    {
        input_update(&input_state);
        printf("X: %u Y: %u\n", input_state.joystick.x, input_state.joystick.y);
    }
}


