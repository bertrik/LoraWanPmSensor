#include <Arduino.h>

#include <stdint.h>
#include <stdbool.h>

#include "sds011_protocol.h"
#include "sds011.h"

void SDS011::_printhex(const char *prefix, const uint8_t * buf, int len)
{
    if (_debug) {
        printf(prefix);
        for (int i = 0; i < len; i++) {
            printf("%02X", buf[i]);
        }
        printf("\n");
    }
}

SDS011::SDS011(Stream *serial, bool debug)
{
    _serial = serial;
    _debug = debug;
}

int SDS011::_exchange(uint8_t * cmd, int cmd_len, uint8_t * rsp, int rsp_size, int timeout)
{
    uint8_t data[19];
    int rsp_len;

    // send cmd
    int len = _protocol.createCommand(data, sizeof(data), cmd, cmd_len);
    _printhex("> ", data, len);
    _serial->write(data, len);

    // wait for response
    unsigned long start = millis();
    while ((millis() - start) < timeout) {
        if (_serial->available()) {
            char c = _serial->read();
            if (_protocol.process(c, 0xC5)) {
                rsp_len = _protocol.getBuffer(rsp, rsp_size);
                _printhex("< ", rsp, rsp_len);
                return rsp_len;
            }
        }
        yield();
    }
    printf("\n");
    return 0;
}

bool SDS011::version(char *serial, char *date)
{
    uint8_t cmd = 7;
    uint8_t rsp[10];
    int rsp_len = _exchange(&cmd, 1, rsp, sizeof(rsp), 1000);

    // parse it, example response 07 12 0A 1E 3A B7 00 00 00 00
    if ((rsp_len > 5) && (rsp[0] == 7)) {
        int year = 2000 + rsp[1];
        int month = rsp[2];
        int day = rsp[3];
        int id = (rsp[4] << 8) | rsp[5];
        sprintf(serial, "%04X", id);
        sprintf(date, "%4d-%2d-%2d", year, month, day);
        return true;
    }

    return false;
}

bool SDS011::fan(bool on)
{
    uint8_t cmd[3];
    uint8_t rsp[10];

    cmd[0] = 6;
    cmd[1] = 1;
    cmd[2] = on ? 1 : 0;
    int rsp_len = _exchange(cmd, sizeof(cmd), rsp, sizeof(rsp), 1000);
    if (rsp_len > 2) {
        return rsp[2] == cmd[2];
    }
    return false;
}

bool SDS011::poll(sds_meas_t *measurement)
{
    uint8_t buf[20];

    while (_serial->available()) {
        uint8_t c = _serial->read();
        if (_protocol.process(c, 0xC0)) {
            int len = _protocol.getBuffer(buf, sizeof(buf));
            if (len > 4) {
                measurement->pm2_5 = ((buf[1] << 8) + buf[0]) / 10.0;
                measurement->pm10 = ((buf[3] << 8) + buf[2]) / 10.0;
                measurement->id = (buf[4] << 8) + buf[5];
                return true;
            }
        }
    }
    return false;
}


