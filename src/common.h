#ifndef _COMMON_H
#define _COMMON_H

#include <stdint.h>

typedef uint8_t byte;
typedef uint8_t bool;

#define set_bit(reg, n) ((reg) |= (1 << (n)))
#define clear_bit(reg, n) ((reg) &= ~(1 << (n)))
#define bit_set(reg, n) ((reg) & (1 << (n)))

typedef enum {
    FORWARD,
    BACKWARD
} Direction;

typedef enum {
    RADIO_CMD_RESERVED = 0,
	RADIO_CMD_LIGHT_ON,
	RADIO_CMD_LIGHT_OFF,
    RADIO_CMD_STOP,
    RADIO_CMD_SET_MOTOR,
    RADIO_CMD_COUNT
} RadioCommand;

typedef struct {
    RadioCommand cmd;
    uint8_t data[4];
} RemoteCommand;

#endif

