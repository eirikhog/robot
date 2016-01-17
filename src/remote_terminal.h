#ifndef _REMOTE_TERMINAL_H
#define _REMOTE_TERMINAL_H

// Adapt these to the display
#define TERMINAL_LINE_HEIGHT 6
#define TERMINAL_LINE_LENGTH 14

#define TERMINAL_WIDTH 84
#define TERMINAL_HEIGHT 48

#include "remote_lcd.h"

#include <stdio.h>

#define TERMINAL_BUFFER_SIZE (TERMINAL_LINE_HEIGHT * TERMINAL_LINE_LENGTH)
static char terminal_buffer[TERMINAL_BUFFER_SIZE];

typedef struct {
    uint8_t size; // buffer count
    char *pos;
    char *end;
} TerminalState;

static TerminalState term_state;
void terminal_clear(void);
int terminal_printf(char c, FILE *fp);
void terminal_write_char(char c);
void terminal_push_to_lcd(void);

void terminal_init() {
    term_state.size = TERMINAL_BUFFER_SIZE;
    term_state.pos = terminal_buffer;
    term_state.end = terminal_buffer + TERMINAL_BUFFER_SIZE;

    terminal_clear();

    // Hook up printf functionality
    fdevopen(terminal_printf, NULL);
}

int terminal_printf(char c, FILE *fp) {
    if (c == '\n') {
        // Fill rest of the line
        int spaces = TERMINAL_LINE_LENGTH - ((term_state.pos - terminal_buffer) % TERMINAL_LINE_LENGTH);
        for (int i = 0; i < spaces; ++i) {
            terminal_write_char(' ');
        }
        terminal_push_to_lcd();
    } else {
        terminal_write_char(c);
    }

    return 1;
}

void terminal_move_up() {
    // Remove first line, and move rest of buffer!
    char *src = terminal_buffer + TERMINAL_LINE_LENGTH;
    char *dst = terminal_buffer;

    for (int i = 0; i < TERMINAL_BUFFER_SIZE; ++i) {
        if (i >= (TERMINAL_LINE_HEIGHT - 1) * TERMINAL_LINE_LENGTH) {
            *dst = ' ';
        } else {
            *dst = *src;
        }
        ++src;
        ++dst;
    }
}

void terminal_write_char(char c) {
    *term_state.pos = c;

    term_state.pos++;
    if (term_state.pos >= term_state.end) {
        term_state.pos = terminal_buffer + TERMINAL_LINE_LENGTH * (TERMINAL_LINE_HEIGHT - 1);
        terminal_move_up();
    }
}

void terminal_write(char *string) {
    uint8_t i = 0;
    char* src = string;
    while (*src) {
        terminal_write_char(*src);
        ++i;
        ++src;
    }

    // Fill the rest of the line
    while (i < TERMINAL_LINE_LENGTH) {
        terminal_write_char(' ');
        ++i;
    }

    terminal_push_to_lcd();
}

void terminal_push_to_lcd() {
    // TODO: We dont always have to redraw everything.
    //lcd_clear();
    lcd_setpos(0, 0);
    for (int y = 0; y < TERMINAL_LINE_HEIGHT; ++y) {
        for (int x = 0; x < TERMINAL_LINE_LENGTH; ++x) {
            lcd_putchar(terminal_buffer[y * TERMINAL_LINE_LENGTH + x], x * 6, y * 8);
        }
    }

    lcd_update();
}

void terminal_clear() {
    for (int i = 0; i < TERMINAL_BUFFER_SIZE; ++i) {
        terminal_buffer[i] = ' ';
    }
}

#endif


