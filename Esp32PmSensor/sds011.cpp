#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "sds011.h"

// magic header/footer bytes
#define MAGIC1 0xAA
#define MAGIC2 0xAB

/**
    Initializes the measurement data state machine.
 */
SDS011::SDS011(void)
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
bool SDS011::process(uint8_t b, uint8_t cmd_id)
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
        if (b == cmd_id) {
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
            process(b, cmd_id);
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
void SDS011::getMeasurement(sds_meas_t *measurement)
{
    measurement->pm2_5 = get_le(_buf, 0) / 10.0;
    measurement->pm10 = get_le(_buf, 2) / 10.0;
    measurement->id = get_be(_buf, 4);
}

/**
    Creates a command buffer to send.
    @param[out] buf the command buffer
    @param[in] size the size of the command buffer
    @param[in] cmd_data the command data byte array
    @param[in] cmd_data_len the length of the byte array
    @return the length of the command buffer, or 0 if no command could be constructed
*/
int SDS011::createCommand(uint8_t *buf, int size, const uint8_t *cmd_data, int cmd_data_len)
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

int SDS011::getBuffer(uint8_t *rsp, int rsp_size)
{
    int len = 10;
    if (len > rsp_size) {
        len = rsp_size;
    }

    memcpy(rsp, &_buf, len);
    return len;
}

