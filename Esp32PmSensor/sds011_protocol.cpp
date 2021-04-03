#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "sds011_protocol.h"

// magic header/footer bytes
#define MAGIC1 0xAA
#define MAGIC2 0xAB

/**
    Initializes the measurement data state machine.
 */
SDS011Protocol::SDS011Protocol(void)
{
    _state = HEAD;
    _size = sizeof(_buf);
    _idx = 0;
    _len = 0;
    _sum = 0;
}

/**
    Processes one byte in the measurement data state machine.
    @param[in] b the byte
    @return true if a full message was received
 */
bool SDS011Protocol::process_rx(uint8_t b, uint8_t rsp_id)
{
    switch (_state) {
    // wait for header byte
    case HEAD:
        if (b == MAGIC1) {
            _state = COMMAND;
        }
        break;
    // receive COMMAND byte
    case COMMAND:
        if (b == rsp_id) {
            _sum = 0;
            _idx = 0;
            _len = 6;
            _state = DATA;
        } else {
            _state = HEAD;
        }
        break;
    // store data
    case DATA:
        _sum += b;
        if (_idx < _len) {
            _buf[_idx++] = b;
        }
        if (_idx == _len) {
            _state = CHECK;
        }
        break;
    // store checksum
    case CHECK:
        if (b == _sum) {
            _state = TAIL;
        } else {
            _state = HEAD;
            process_rx(b, rsp_id);
        }
        break;
    // wait for tail byte
    case TAIL:
        _state = HEAD;
        return (b == MAGIC2);
    default:
        _state = HEAD;
        break;
    }
    return false;
}

/**
    Creates a command buffer to send.
    @param[out] buf the command buffer, should be at least 19 bytes
    @param[in] cmd the command code
    @param[in] cmd_len the length of the byte array
    @param[in] cmd_data the command data byte array
    @return the length of the command buffer, or 0 if no command could be constructed
*/
int SDS011Protocol::build_tx(uint8_t *buf, uint8_t cmd, size_t cmd_len, const uint8_t *cmd_data)
{
    int idx = 0;

    buf[idx++] = MAGIC1;
    buf[idx++] = 0xB4;
    buf[idx++] = cmd;
    for (int i = 0; i < 12; i++) {
        buf[idx++] = (i < cmd_len) ? cmd_data[i] : 0;
    }
    buf[idx++] = 0xFF;
    buf[idx++] = 0xFF;

    // calculate check
    uint8_t sum = 0;
    for (int i = 2; i < 17; i++) {
        sum += buf[i];
    }
    buf[idx++] = sum;
    buf[idx++] = MAGIC2;
    return idx;
}

size_t SDS011Protocol::get_data(uint8_t *rsp)
{
    memcpy(rsp, _buf, 6);
    return 6;
}

