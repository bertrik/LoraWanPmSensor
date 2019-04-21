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
#include <SSD1306.h>
#include "soc/efuse_reg.h"
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

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

#define PIN_BUTTON 0

#define PIN_RX  34
#define PIN_TX  25

#define OTAA_MAGIC 0xCAFEBABE

typedef struct {
    u4_t netid = 0;
    devaddr_t devaddr = 0;
    u1_t nwkKey[16];
    u1_t artKey[16];
    uint32_t magic;
} otaa_data_t;

typedef struct {
    bool update;
    // 1st line: LoRa address
    char loraDevEui[32];
    // 2nd line: LoRa status
    char loraStatus[32];
    // 3rd line: PM10 + temperature
    char pm10[16];
    char temp[16];
    // 4th line: PM2.5 + humidity
    char pm2_5[16];
    char humi[16];
} screen_t;

static otaa_data_t otaa_data;
static uint64_t chipid;
static SSD1306 display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL);
static SoftwareSerial sds011(PIN_RX, PIN_TX);
static screen_t screen;

// This should also be in little endian format, see above.
void os_getDevEui(u1_t * buf)
{
    memcpy_P(buf, &chipid, 8);
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
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = { 26, 33, 32 }       // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

static void print_keys(void)
{
    Serial.print("netid: ");
    Serial.println(otaa_data.netid, DEC);
    Serial.print("devaddr: ");
    Serial.println(otaa_data.devaddr, HEX);
    Serial.print("artKey: ");
    for (int i = 0; i < sizeof(otaa_data.artKey); ++i) {
        Serial.print(otaa_data.artKey[i], HEX);
    }
    Serial.println("");
    Serial.print("nwkKey: ");
    for (int i = 0; i < sizeof(otaa_data.nwkKey); ++i) {
        Serial.print(otaa_data.nwkKey[i], HEX);
    }
    Serial.println("");
}

static void setLoraStatus(const char *status)
{
    snprintf(screen.loraStatus, sizeof(screen.loraStatus), status);
    screen.update = true;
}

static void showLoraDevAddr(void)
{
    char devaddr[16];
    snprintf(devaddr, sizeof(devaddr), "%08X", LMIC.devaddr);
    setLoraStatus(devaddr);
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        setLoraStatus("OTAA JOIN...");
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        LMIC_getSessionKeys(&otaa_data.netid, &otaa_data.devaddr, otaa_data.nwkKey,
                            otaa_data.artKey);
        otaa_data.magic = OTAA_MAGIC;
        EEPROM.put(0, otaa_data);
        EEPROM.commit();

        print_keys();

        setLoraStatus("JOIN OK!");
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        setLoraStatus("JOIN failed!");
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        setLoraStatus("REJOIN failed!");
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen) {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        showLoraDevAddr();
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        setLoraStatus("TX start");
        break;
    case EV_TXCANCELED:
        Serial.println(F("EV_TXCANCELED"));
        break;
    case EV_RXSTART:
        Serial.println(F("EV_RXSTART"));
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_JOIN_TXCOMPLETE"));
        setLoraStatus("JOIN sent");
        break;
    default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned) ev);
        break;
    }
}

static void dump(uint8_t * buf, int len)
{
    int i;
    char hex[8];
    for (i = 0; i < len; i++) {
        sprintf(hex, " %02X", buf[i]);
        Serial.print(hex);
    }
    Serial.println();
}

static int sds_exchange(uint8_t * cmd, int cmd_len, uint8_t * rsp, int timeout)
{
    uint8_t data[19];
    int rsp_len;

    // send cmd
    int len = SdsCreateCmd(data, sizeof(data), cmd, cmd_len);
    sds011.write(data, len);
    dump(data, len);

    // wait for response
    unsigned long start = millis();
    while ((millis() - start) < timeout) {
        if (sds011.available()) {
            char c = sds011.read();
            if (SdsProcess(c, 0xC5)) {
                rsp_len = SdsGetBuffer(rsp);
                return rsp_len;
            }
        }
    }
    return 0;
}

static bool sds_version(void)
{
    uint8_t cmd = 7;
    uint8_t rsp[10];
    int rsp_len;
    rsp_len = sds_exchange(&cmd, 1, rsp, 1000);
    if (rsp_len > 0) {
        dump(rsp, rsp_len);
    }

    return (rsp_len > 0);
}

void setup(void)
{
    Serial.begin(115200);
    Serial.println(F("Starting..."));

    // button config
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    // reset the OLED
    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(50);
    digitalWrite(OLED_RESET, HIGH);

    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    display.drawString(0, 0, "Init!");
    display.display();

    chipid = ESP.getEfuseMac();

    // initialize the SDS011
    sds011.begin(9600);
    SdsInit();

    EEPROM.begin(512);
    EEPROM.get(0, otaa_data);

    // LMIC init
    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    if (otaa_data.magic == OTAA_MAGIC) {
        LMIC_setSession(otaa_data.netid, otaa_data.devaddr, otaa_data.nwkKey, otaa_data.artKey);
        print_keys();
    } else {
        LMIC_startJoining();
    }

    // screen
    uint8_t id[8];
    memcpy(id, &chipid, sizeof(chipid));
    memset(&screen, 0, sizeof(screen));
    for (int i = 0; i < 8; i++) {
        char buf[8];
        if (i == 0) {
            strcpy(screen.loraDevEui, "");
        } else {
            strcat(screen.loraDevEui, ":");
        }
        snprintf(buf, sizeof(buf), "%02X", id[7 - i]);
        strcat(screen.loraDevEui, buf);
    }
    screen.update = true;

    sds_version();
    sds_version();
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

static void screen_update(sds_meas_t * meas)
{
    char value[16];

    if (screen.update) {
        display.clear();

        // 1st line
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 0, screen.loraDevEui);

        // 2nd line
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 12, screen.loraStatus);

        // 3rd
        snprintf(value, sizeof(value), "PM 10: %3d ug/m3", (int) (meas->pm10));
        display.drawString(0, 30, value);

        // 4th line
        snprintf(value, sizeof(value), "PM2.5: %3d ug/m3", (int) (meas->pm2_5));
        display.drawString(0, 46, value);

        display.display();
        screen.update = false;
    }
}


void loop(void)
{
    static sds_meas_t sds_meas;
    static unsigned long last_sent = 0;
    static bool have_data = false;
    static u2_t opmode;
    static unsigned long button_ts = 0;

    // check for long button press to restart OTAA
    unsigned long ms = millis();
    if (digitalRead(PIN_BUTTON) == 0) {
        if ((ms - button_ts) > 2000) {
            Serial.println("Resetting OTAA");
            LMIC_reset();
            LMIC_startJoining();
            button_ts = ms;
        }
    } else {
        button_ts = ms;
    }

    // check for incoming measurement data
    while (sds011.available()) {
        uint8_t c = sds011.read();
        if (SdsProcess(c, 0xC0)) {
            // parse it
            SdsParse(&sds_meas);
            have_data = true;
            screen.update = true;
        }
    }

    // time to send a dust measurement?
    unsigned long now = millis() / 1000;
    if ((now - last_sent) > TX_INTERVAL) {
        last_sent = now;
        if (have_data) {
            send_dust(&sds_meas);
            have_data = false;
        }
    }
    // log LMIC state changes
    if (LMIC.opmode != opmode) {
        opmode = LMIC.opmode;
        Serial.print("New opmode: ");
        Serial.println(opmode, HEX);
    }
    // update screen
    screen_update(&sds_meas);

    // ?
    os_runloop_once();
}
