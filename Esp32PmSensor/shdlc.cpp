
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "shdlc.h"

SHDLC::SHDLC(void)
{
    _state = SHDLC_START;
    _index = 0;
    _length = 0;
    _miso_state = 0;
}

// true if *pc contains a valid value
bool SHDLC::unescape(uint8_t *pc)
{
    if (_escape) {
        _escape = false;
        *pc ^= 0x20;
        return true;
    }
    if (*pc == 0x7D) {
        _escape = true;
        return false;
    }
    return true;
}

static int add_byte(uint8_t *buf, int &idx, uint8_t b)
{
    switch (b) {
    case 0x7E:
    case 0x7D:
    case 0x11:
    case 0x13:
        buf[idx++] = 0x7D;
        buf[idx++] = b ^ 0x20;
        break;
    default:
        buf[idx++] = b;
        break;
    }
    return b;
}

size_t SHDLC::build_tx(uint8_t *buf, uint8_t cmd, size_t data_len, const uint8_t *data)
{
    uint8_t sum = 0;
    int index = 0;

    // reset state machine
    _cmd = cmd;
    _state = SHDLC_START;
    _escape = false;

    // build command
    buf[index++] = 0x7E;
    sum += add_byte(buf, index, 0);
    sum += add_byte(buf, index, cmd);
    sum += add_byte(buf, index, data_len);
    for (int i = 0; i < data_len; i++) {
        sum += add_byte(buf, index, data[i]);
    }
    add_byte(buf, index, sum ^ 0xFF);
    buf[index++] = 0x7E;
    return index;
}

size_t SHDLC::get_data(uint8_t *data)
{
    memcpy(data, _data, _length);
    return _length; 
}

uint8_t SHDLC::get_state(void)
{
    return _state;
}

bool SHDLC::process_rx(uint8_t c, uint8_t cmd)
{
    switch (_state) {
    case SHDLC_START:
        if (c == 0x7E) {
            _state = SHDLC_ADR;
            _escape = false;
        }
        break;
    case SHDLC_ADR:
        if (unescape(&c)) {
            _check = c;
            _state = SHDLC_CMD;
        }
        break;
    case SHDLC_CMD:
        if (unescape(&c)) {
            if (c == _cmd) {
                _check += c;
                _state = SHDLC_STATE;
            } else {
                _state = SHDLC_START;
            }
        }
        break;
    case SHDLC_STATE:
        if (unescape(&c)) {
            _check += c;
            _miso_state = c;
            _state = SHDLC_L;
        }
        break;
    case SHDLC_L:
        if (unescape(&c)) {
            _check += c;
            _index = 0;
            _length = c;
            if (_length > 0) {
                _state = SHDLC_DATA;
            } else {
                _state = SHDLC_CHK;
            }
        }
        break;
    case SHDLC_DATA:
        if (unescape(&c)) {
            _check += c;
            _data[_index++] = c;
            if (_index == _length) {
                _state = SHDLC_CHK;
            }    
        }
        break;
    case SHDLC_CHK:
        if (unescape(&c)) {
            _check ^= 0xFF;
            if (_check == c) {
                _state = SHDLC_STOP;
            } else {
                printf("check=0x%02X\n", _check);
                _state = SHDLC_START;
            }
        }
        break;
    case SHDLC_STOP:
        _state = SHDLC_START;
        return (c == 0x7E);
    default:
        _state = SHDLC_START;
        break;
    }

    return false;    
}


