
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
        yield();
    }
    return -1;
}  

bool SPS30::start()
{
    _mosi[0] = 1;   // sub-command
    _mosi[1] = 3;   // format: 3 = IEEE754 float, 5 = big-endian integer
    return exchange(0x00, 2) == 0;
}

bool SPS30::stop(void)
{
    return exchange(0x01, 0) == 0;
}

// performs machine-dependent conversion from big-endian IEEE754 bytes to float
static float bytes_to_float(uint8_t *data)
{
    float f;
    uint8_t *b = (uint8_t *)&f;
    b[0] = data[3];
    b[1] = data[2];
    b[2] = data[1];
    b[3] = data[0];
    return f;
}

bool SPS30::read_measurement(sps_meas_t *meas)
{
    int len = exchange(0x03, 0);
    if (len < 40) {
        return false;
    }
    meas->pm1_0 = bytes_to_float(_miso + 0);
    meas->pm2_5 = bytes_to_float(_miso + 4);
    meas->pm4_0 = bytes_to_float(_miso + 8);
    meas->pm10 = bytes_to_float(_miso + 12);
    meas->n0_5 = bytes_to_float(_miso + 16);
    meas->n1_0 = bytes_to_float(_miso + 20);
    meas->n2_5 = bytes_to_float(_miso + 24);
    meas->n4_0 = bytes_to_float(_miso + 28);
    meas->n10 = bytes_to_float(_miso + 32);
    meas->tps = bytes_to_float(_miso + 36);
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

