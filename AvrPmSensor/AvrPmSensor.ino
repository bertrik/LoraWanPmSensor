// MIT License
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

#include <stdio.h>
#include <stdint.h>

#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <SPI.h>
#include <ArduinoUniqueID.h>
#include "SoftwareSerial.h"
#include "EEPROM.h"

#include "sds011.h"

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x9B, 0xA0, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] =
    { 0xAA, 0x9F, 0x12, 0x45, 0x7F, 0x06, 0x64, 0xDF, 0x4C, 0x1E, 0x9F,
    0xC9, 0x5E, 0xDA, 0x1A, 0x8A
};

const unsigned TX_INTERVAL = 10;

#define PIN_BUTTON 0

#define OTAA_MAGIC 0xCAFEBABE

typedef struct {
    u4_t netid = 0;
    devaddr_t devaddr = 0;
    u1_t nwkKey[16];
    u1_t artKey[16];
    uint32_t magic;
} otaa_data_t;

// stored in "little endian" format
static uint8_t deveui[8];
static otaa_data_t otaa_data;
static SoftwareSerial sdsSerial(1, 1);
static SDS011 sds;

// This should also be in little endian format, see above.
void os_getDevEui(u1_t * buf)
{
    memcpy_P(buf, deveui, 8);
}

void os_getArtEui(u1_t * buf)
{
    memcpy_P(buf, APPEUI, 8);
}

void os_getDevKey(u1_t * buf)
{
    memcpy_P(buf, APPKEY, 16);
}

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 0,
    .dio = { 4, 5, 7 }
};

static void print_keys(void)
{
    char hex[8];

    Serial.print(F("netid: "));
    Serial.println(otaa_data.netid, DEC);
    Serial.print(F("devaddr: "));
    Serial.println(otaa_data.devaddr, HEX);
    Serial.print(F("artKey: "));
    for (size_t i = 0; i < sizeof(otaa_data.artKey); ++i) {
        snprintf(hex, sizeof(hex), " %02X", otaa_data.artKey[i]);
        Serial.print(hex);
    }
    Serial.println("");
    Serial.print(F("nwkKey: "));
    for (size_t i = 0; i < sizeof(otaa_data.nwkKey); ++i) {
        snprintf(hex, sizeof(hex), " %02X", otaa_data.nwkKey[i]);
        Serial.print(hex);
    }
    Serial.println("");
}

static void setLoraStatus(const char *fmt, ...)
{
    char line[64];

    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    Serial.println(line);
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");

    switch (ev) {
    case EV_JOINING:
        setLoraStatus("OTAA JOIN...");
        break;
    case EV_JOINED:
        LMIC_getSessionKeys(&otaa_data.netid, &otaa_data.devaddr, otaa_data.nwkKey,
                            otaa_data.artKey);
        otaa_data.magic = OTAA_MAGIC;
        EEPROM.put(0, otaa_data);
        print_keys();

        setLoraStatus("JOIN OK!");
        break;
    case EV_JOIN_FAILED:
        setLoraStatus("JOIN failed!");
        break;
    case EV_TXCOMPLETE:
        setLoraStatus("%08X-%d", LMIC.devaddr, LMIC.seqnoUp);
        break;
    case EV_TXSTART:
        setLoraStatus("Transmitting");
        break;
    case EV_JOIN_TXCOMPLETE:
        setLoraStatus("JOIN sent");
        break;
    default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned) ev);
        break;
    }
}

static int sds_exchange(uint8_t * cmd, int cmd_len, uint8_t * rsp, int rsp_size,
                        unsigned int timeout)
{
    uint8_t data[19];
    int rsp_len;

    // send cmd
    int len = sds.createCommand(data, sizeof(data), cmd, cmd_len);
    sdsSerial.write(data, len);

    // wait for response
    unsigned long start = millis();
    while ((millis() - start) < timeout) {
        if (sdsSerial.available()) {
            char c = sdsSerial.read();
            if (sds.process(c, 0xC5)) {
                rsp_len = sds.getBuffer(rsp, rsp_size);
                return rsp_len;
            }
        }
    }
    return 0;
}

static bool sds_version(char *version, int size)
{
    uint8_t cmd = 7;
    uint8_t rsp[10];
    int rsp_len;
    rsp_len = sds_exchange(&cmd, 1, rsp, sizeof(rsp), 1000);

    // parse it, example response 07 12 0A 1E 3A B7 00 00 00 00
    if ((rsp_len > 5) && (rsp[0] == 7)) {
        int year = rsp[1];
        int month = rsp[2];
        int day = rsp[3];
        int id = (rsp[4] << 8) | rsp[5];
        snprintf(version, size, "%04X/%2d-%2d-%2d", id, year, month, day);
        return true;
    }

    return false;
}

void setup(void)
{
    Serial.begin(115200);
    Serial.println(F("Starting..."));
    UniqueIDdump(Serial);

    // initialize the SDS011
    sdsSerial.begin(9600);

    // setup of unique ids
    memcpy(deveui, _UniqueID.id, 8);
    Serial.print(F("DEV EUI:"));
    char hex[8];
    for (int i = 0; i < 8; i++) {
        snprintf(hex, sizeof(hex), " %02X", deveui[i]);
        Serial.print(hex);
    }
    Serial.println();

#if 0   // needs actual hardware
    Serial.println(F("os_init()"));
    // LMIC init
    os_init();

    Serial.println(F("LMIC_reset()"));
    LMIC_reset();
#endif

    EEPROM.begin();
    EEPROM.get(0, otaa_data);
    if (otaa_data.magic == OTAA_MAGIC) {
        setLoraStatus("Resume OTAA");
        LMIC_setSession(otaa_data.netid, otaa_data.devaddr, otaa_data.nwkKey, otaa_data.artKey);
        print_keys();
    } else {
        LMIC_startJoining();
    }

    char version[32];
    if (sds_version(version, sizeof(version))) {
        Serial.print(F("SDS version and date: "));
        Serial.println(version);
    }
    Serial.println(F("setup() done"));
}

static void send_dust(sds_meas_t * meas)
{
    uint8_t buf[20];

    if ((LMIC.opmode & (OP_TXDATA | OP_TXRXPEND)) == 0) {
        // encode it
        int idx = 0;
        int pm10 = 10.0 * meas->pm10;
        buf[idx++] = (pm10 >> 8) & 0xFF;
        buf[idx++] = (pm10 >> 0) & 0xFF;
        int pm2_5 = 10.0 * meas->pm2_5;
        buf[idx++] = (pm2_5 >> 8) & 0xFF;
        buf[idx++] = (pm2_5 >> 0) & 0xFF;

        buf[idx++] = 0xFF;
        buf[idx++] = 0xFF;

        buf[idx++] = 0xFF;
        buf[idx++] = 0xFF;

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, buf, idx, 0);
        Serial.println(F("Sending uplink packet..."));
    }
}

void loop(void)
{
    static sds_meas_t sds_meas;
    static unsigned long last_sent = 0;
    static bool have_data = false;

    // check for incoming measurement data
    while (sdsSerial.available()) {
        uint8_t c = sdsSerial.read();
        if (sds.process(c, 0xC0)) {
            // parse it
            sds.getMeasurement(&sds_meas);
            have_data = true;
        }
    }
    
    // hack, REMOVE
    have_data = true;

    // time to send a dust measurement?
    unsigned long now = millis() / 1000;
    if ((now - last_sent) > TX_INTERVAL) {
        last_sent = now;
        if (have_data) {
            send_dust(&sds_meas);
            have_data = false;
        }
    }

    // run LMIC periodic stuff
    os_runloop_once();
}

