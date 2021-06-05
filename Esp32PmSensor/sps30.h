#include <Arduino.h>

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float pm1_0;
    float pm2_5;
    float pm4_0;
    float pm10;
    float n0_5;
    float n1_0;
    float n2_5;
    float n4_0;
    float n10;
    float tps;
} sps_meas_t;

class SPS30 {

private:
    Stream *_serial;
    bool _debug;
    uint8_t _mosi[256];
    uint8_t _miso[256];

    void printhex(const char *prefix, const uint8_t * buf, int len);
    int exchange(uint8_t cmd, size_t out_len);

public:
    static const int BIT_RATE = 115200;

    /**
     * Constructor.
     *
     * @param serial the serial port, NOTE: the serial port has to be configured for a bit rate of SPS30::BIT_RATE !
     */
    explicit SPS30(Stream *serial, bool debug = false);

    bool start(void);
    bool stop(void);
    bool read_measurement(sps_meas_t *meas);
    bool sleep(void);
    bool wakeup(void);
    bool clean_fan(void);
    bool read_autoclean_interval(uint32_t *interval);
    bool write_autoclean_interval(uint32_t interval);
    bool device_info(char *product_type, char *serial_number);
    bool read_version(uint16_t *fw_version, uint16_t *hw_version, uint16_t *shdlc_version);
    bool reset(void);
};

