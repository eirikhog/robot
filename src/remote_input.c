#include "remote_input.h"

#include <avr/interrupt.h>

ISR(ADC_vect) {
    if (ADMUX & (1 << MUX0)) { // ADC Channel 1
        ADMUX &= ~(1 << MUX0);
        input_state.joystick.x = ADC;
    } else { // ADC Channel 0
        ADMUX |= (1 << MUX0);
        input_state.joystick.y = ADC;
    }
}

void buttons_init() {
    // 3 Buttons on PORT D
    uint8_t portd_mask = (1 << PIND7) | (1 << PIND6) | (1 << PIND5);
    DDRD &= ~portd_mask;
    PORTD |= portd_mask;

    // 1 Button on Port B
    uint8_t portb_mask = (1 << PORTB0);
    DDRB &= ~portb_mask;
    PORTB |= portb_mask;

    // Initialize button state
    input_state.buttons = 0xFF;
}

void adc_init(void) {
    DDRC &= ~((1 << PINC2) | (1 << PINC1) | (1 << PINC0));
    PORTC &= ~((1 << PINC2) | (1 << PINC1) | (1 << PINC0));
    
    ADMUX = (0 << ADLAR) | (1 << REFS0) | (1 << MUX0);
    ADCSRA = (1 << ADATE) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1);
    ADCSRB = 0;

    ADCSRA |= (1 << ADEN);
    ADCSRA |= (1 << ADSC);
}

void joystick_init(void) {
    DDRC |= (1 << PINC2);
    PORTC |= (1 << PINC2);
}

void joystick_update(volatile JoyState *joystick) {
    // x and y are set with interrupt.
    if (PINC & (1 << PINC2)) {
        joystick->sw = 0;
    } else {
        joystick->sw = 1;
    }
}

uint8_t buttons_get_state() {

    uint8_t state = 0;
    if (bit_set(PINB, 0)) {
        state |= BUTTON_UP;
    }
    if (bit_set(PIND, 7)) {
        state |= BUTTON_DOWN;
    }
    if (bit_set(PIND, 6)) {
        state |= BUTTON_LEFT;
    }
    if (bit_set(PIND, 5)) {
        state |= BUTTON_RIGHT;
    }

    return state;
}

uint8_t button_pressed(uint8_t button, uint8_t buttons_state, uint8_t buttons_prev_state) {
    if (buttons_state & button && !(buttons_prev_state & button)) {
        return 1;
    }
    return 0;
}

void input_init(void) {
    buttons_init();
    joystick_init();
    adc_init();
}

void input_update(volatile InputState *state) {
    state->buttons = buttons_get_state();
    joystick_update(&state->joystick);
}

