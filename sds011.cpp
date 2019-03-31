#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "sds011.h"

// magic header/footer bytes
#define MAGIC1 0xAA
#define MAGIC2 0xAB

// parsing state
typedef enum {
    HEAD = 0,
    COMMAND,
    DATA,
    CHECK,
    TAIL
} EState;

typedef struct {
    EState  state;
    uint8_t cmd;
    uint8_t buf[32];
    int     size;
    int     idx, len;
    uint8_t sum;
} TState;

static TState state;

/**
    Initializes the measurement data state machine.
 */
void SdsInit(void)
{
    state.state = HEAD;
    state.size = sizeof(state.buf);
    state.idx = state.len = 0;
    state.sum = 0;
}

/**
    Processes one byte in the measurement data state machine.
    @param[in] b the byte
    @return true if a full message was received
 */
bool SdsProcess(uint8_t b)
{
    switch (state.state) {
    // wait for header byte
    case HEAD:
        if (b == MAGIC1) {
            state.state = COMMAND;
        }
        break;
    // receive COMMAND byte
    case COMMAND:
        state.sum = 0;
        state.cmd = b;
        state.idx = 0;
        state.len = 6;
        state.state = DATA;
        break;
    // store data
    case DATA:
        state.sum += b;
        if (state.idx < state.len) {
            state.buf[state.idx++] = b;
        }
        if (state.idx == state.len) {
            state.state = CHECK;
        }
        break;
    // store checksum
    case CHECK:
        if (b == state.sum) {
            state.state = TAIL;
        } else {
            state.state = HEAD;
            SdsProcess(b);
        }
        break;
    // wait for tail byte
    case TAIL:
        state.state = HEAD;
        return (b == MAGIC2);
    default:
        state.state = HEAD;
        break;
    }
    return false;
}

static uint16_t get_le(const uint8_t *buf, int idx)
{
    uint16_t word;
    word = buf[idx++];
    word += buf[idx++] << 8;
    return word;
}

static uint16_t get_be(const uint8_t *buf, int idx)
{
    uint16_t word;
    word = buf[idx++];
    word = (word << 8) | buf[idx++];
    return word;
}

/**
    Parses a complete measurement data frame into a structure.
    @param[out] meas the parsed measurement data
 */
void SdsParse(sds_meas_t *meas)
{
    meas->pm2_5 = get_le(state.buf, 0) / 10.0;
    meas->pm10 = get_le(state.buf, 2) / 10.0;
    meas->id = get_be(state.buf, 4);
}

/**
    Creates a command buffer to send.
    @param[out] buf the command buffer
    @param[in] size the size of the command buffer
    @param[in] cmd_data the command data byte array
    @param[in] cmd_data_len the length of the byte array
    @return the length of the command buffer, or 0 if no command could be constructed
*/
int SdsCreateCmd(uint8_t *buf, int size, const uint8_t *cmd_data, int cmd_data_len)
{
    // verify arguments
    if (size < 19) {
        return 0;
    }
    if (cmd_data_len > 13) {
        return 0;
    }

    // fill buffer
    memset(buf, 0, size);
    buf[0] = MAGIC1;
    buf[1] = 0xB4;
    memcpy(&buf[2], cmd_data, cmd_data_len);
    buf[15] = 0xFF;
    buf[16] = 0xFF;

    // calculate check
    uint8_t sum = 0;
    for (int i = 2; i < 17; i++) {
        sum += buf[i];
    }
    buf[17] = sum;
    buf[18] = MAGIC2;
    return 19;
}

