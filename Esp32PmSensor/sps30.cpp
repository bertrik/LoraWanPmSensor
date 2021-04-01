
#include <stdbool.h>
#include <stdio.h>

#include "shdlc.h"
#include "sps30.h"

#define DEFAULT_TIMEOUT 100

static SHDLC shdlc;

void SPS30::printhex(const char *prefix, const uint8_t * buf, int len)
{
    if (_debug) {
        printf(prefix);
        for (int i = 0; i < len; i++) {
            printf("%02X", buf[i]);
        }
        printf("\n");
    }
}

SPS30::SPS30(Stream *serial, bool debug)
{
    _serial = serial;
    _debug = debug;
}

int SPS30::exchange(uint8_t cmd, size_t out_len)
{
    uint8_t buf[256];
    size_t in_len;

    // build command and send it
    int len = shdlc.build_tx(buf, cmd, out_len, _mosi);
    printhex("MOSI: ", buf, len);
    _serial->write(buf, len);

    // wait for result
    unsigned long start = millis();
    while ((millis() - start) < DEFAULT_TIMEOUT) {
        while (_serial->available()) {
            char c = _serial->read();
            if (shdlc.process_rx(c, cmd)) {
                in_len = shdlc.get_data(_miso);
                printhex("MISO: ", _miso, in_len);
                return in_len;
            }
        }
    }
    return -1;
}  

bool SPS30::start()
{
    _mosi[0] = 1;   // sub-command
    _mosi[1] = 5;   // 5 = use integer (not float)
    return exchange(0x00, 2) == 0;
}

bool SPS30::stop(void)
{
    return exchange(0x01, 0) == 0;
}

bool SPS30::read_measurement(uint16_t *pm1_0, uint16_t *pm2_5, uint16_t *pm4_0, uint16_t *pm10, uint16_t *ps)
{
    int len = exchange(0x03, 0);
    if (len < 20) {
        return false;
    }
    *pm1_0 = (_miso[0] << 8) + _miso[1];
    *pm2_5 = (_miso[2] << 8) + _miso[3];
    *pm4_0 = (_miso[4] << 8) + _miso[5];
    *pm10 = (_miso[6] << 8) + _miso[7];
    *ps = (_miso[18] << 8) + _miso[19];
    return true;
}

bool SPS30::sleep(void)
{
    return (exchange(0x10, 0) == 0);
}

bool SPS30::wakeup(void)
{
    _serial->write(0xFF);
    return (exchange(0x11, 0) == 0);
}

bool SPS30::clean_fan(void)
{
    int len = exchange(0x56, 0);
    return (len == 0);
}

bool SPS30::read_autoclean_interval(uint32_t *interval)
{
    int len = 0;

    _mosi[len++] = 0;
    len = exchange(0x80, len);
    if (len < 4) {
        return false;
    }
    *interval = (_miso[0] << 24) |
                (_miso[1] << 16) |
                (_miso[2] << 8) |
                (_miso[3] << 0);
    return true;
}

bool SPS30::write_autoclean_interval(uint32_t interval)
{
    int len = 0;
    _mosi[len++] = 0;
    _mosi[len++] = (interval >> 24) & 0xFF;
    _mosi[len++] = (interval >> 16) & 0xFF;
    _mosi[len++] = (interval >> 8) & 0xFF;
    _mosi[len++] = (interval >> 0) & 0xFF;
    return (exchange(0x80, len) == 0);
}

bool SPS30::device_info(char *product_type, char *serial_number)
{
    int len;

    // product type
    _mosi[0] = 0;
    len = exchange(0xd0, 1);
    if (len <= 0) {
        return false;
    }
    strlcpy(product_type, (const char *)_miso, len);

    // serial number
    _mosi[0] = 3;
    len = exchange(0xd0, 1);
    if (len <= 0) {
        return false;
    }
    strlcpy(serial_number, (const char *)_miso, len);

    return true;
}

bool SPS30::read_version(uint16_t *fw_version, uint16_t *hw_version, uint16_t *shdlc_version)
{
    int len = exchange(0xd1, 0);
    if (len < 7) {
        return false;
    }
    *fw_version = (_miso[0] << 8) | _miso[1];
    *hw_version = _miso[3];
    *shdlc_version = (_miso[5] << 8) | _miso[6];
    return true;
}

bool SPS30::reset(void)
{
    return (exchange(0xD3, 0) == 0);
}

