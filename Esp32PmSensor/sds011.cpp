#include <Arduino.h>

#include <stdint.h>
#include <stdbool.h>

#include "sds011_protocol.h"
#include "sds011.h"

#define DEFAULT_TIMEOUT 1000

#define CMD_SET_DATA_REPORTING_MODE 0x02
#define CMD_QUERY_DATA              0x04
#define CMD_SET_DEVICE_ID           0x05
#define CMD_SET_SLEEP_AND_WORK      0x06
#define CMD_CHECK_FIRMWARE_VERSION  0x07
#define CMD_SET_WORKING_PERIOD      0x08


SDS011::SDS011(Stream *serial, bool debug)
{
    _serial = serial;
    _debug = debug;
}

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

// exchanges command data, returns length of response data (or < 0) if no data available
int SDS011::_exchange(uint8_t cmd, size_t cmd_len, const uint8_t * cmd_data)
{
    // send cmd
    int len = _protocol.build_tx(_cmd_buf, cmd, cmd_len, cmd_data);
    _printhex("> ", _cmd_buf, len);
    _serial->write(_cmd_buf, len);

    // wait for response
    unsigned long start = millis();
    while ((millis() - start) < DEFAULT_TIMEOUT) {
        if (_serial->available()) {
            char c = _serial->read();
            if (_protocol.process_rx(c, 0xC5)) {
                int rsp_len = _protocol.get_data(_rsp_buf);
                _printhex("< ", _rsp_buf, rsp_len);
                if (_rsp_buf[0] == cmd) {
                    return rsp_len;
                }
                return rsp_len;
            }
        }
        yield();
    }
    printf("\n");
    return -1;
}

bool SDS011::fan(bool on)
{
    uint8_t cmd[2];

    cmd[0] = 1; // 0 = query mode, 1 = set mode
    cmd[1] = on ? 1 : 0;
    int rsp_len = _exchange(CMD_SET_SLEEP_AND_WORK, sizeof(cmd), cmd);
    return (rsp_len > 0);
}

bool SDS011::version(char *serial, char *date)
{
    // example response 07 12 0A 1E 3A B7 00 00 00 00
    int rsp_len = _exchange(CMD_CHECK_FIRMWARE_VERSION, 0, NULL);
    if (rsp_len > 5) {
        int year = 2000 + _rsp_buf[1];
        int month = _rsp_buf[2];
        int day = _rsp_buf[3];
        int id = (_rsp_buf[4] << 8) | _rsp_buf[5];
        sprintf(serial, "%04X", id);
        sprintf(date, "%4d-%02d-%02d", year, month, day);
        return true;
    }

    return false;
}

bool SDS011::poll(sds_meas_t *measurement)
{
    while (_serial->available()) {
        uint8_t c = _serial->read();
        if (_protocol.process_rx(c, 0xC0)) {
            int len = _protocol.get_data(_rsp_buf);
            if (len > 4) {
                measurement->pm2_5 = ((_rsp_buf[1] << 8) + _rsp_buf[0]) / 10.0;
                measurement->pm10 = ((_rsp_buf[3] << 8) + _rsp_buf[2]) / 10.0;
                measurement->id = (_rsp_buf[4] << 8) + _rsp_buf[5];
                return true;
            }
        }
    }
    return false;
}


