#ifndef _COMMON_H
#define _COMMON_H

#include <stdint.h>

typedef uint8_t byte;
typedef uint8_t bool;

#define set_bit(reg, n) ((reg) |= (1 << (n)))
#define clear_bit(reg, n) ((reg) &= ~(1 << (n)))
#define bit_set(reg, n) ((reg) & (1 << (n)))

// Note: Not safe in edge cases
// #define abs(x) ((x > 0) ? x : -x)
#define max(x,y) ((x < y) ? y : x)

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

typedef enum {
    RADIO_NACK = 0,
    RADIO_ACK = 1
} RadioProtocol;

typedef struct {
    RadioCommand cmd;
    uint8_t data[4];
} RemoteCommand;

#endif

