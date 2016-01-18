#ifndef _REMOTE_INPUT_H
#define _REMOTE_INPUT_H

#include <stdint.h>
#include "common.h"

typedef struct {
    uint16_t x;
    uint16_t y;
    bool sw;
} JoyState;

enum Buttons {
    BUTTON_UP = 0x1,
    BUTTON_DOWN = 0x2,
    BUTTON_LEFT = 0x4,
    BUTTON_RIGHT = 0x8
};

typedef struct {
    JoyState joystick;
    uint8_t buttons;
} InputState;

uint8_t pressed(uint8_t button, uint8_t buttons_state, uint8_t buttons_prev_state);
void input_init(void);
void input_update(volatile InputState *state);

#endif

