#include <Arduino.h>

#include <stdint.h>
#include <stdbool.h>

#include "sds011_protocol.h"

typedef struct {
    float pm2_5;
    float pm10;
    uint16_t id;
} sds_meas_t;

class SDS011 {

private:
    Stream *_serial;
    bool _debug;
    uint8_t _cmd_buf[32];
    uint8_t _rsp_buf[8];

    SDS011Protocol _protocol;
    int _exchange(uint8_t cmd, size_t cmd_len, const uint8_t * cmd_data);
    void _printhex(const char *prefix, const uint8_t * buf, int len);

public:
    explicit SDS011(Stream *serial, bool debug = false);

    bool version(char *serial, char *date);
    bool fan(bool on);
    bool poll(sds_meas_t *measuremnt);

};

