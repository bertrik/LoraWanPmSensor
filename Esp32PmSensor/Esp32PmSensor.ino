// MIT License
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

#include <stdio.h>
#include <stdint.h>

#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include "arduino_lmic_hal_boards.h"

#include <SPI.h>
#include <SSD1306.h>
#include <SparkFunBME280.h>
#include "soc/efuse_reg.h"
#include "HardwareSerial.h"
#include "EEPROM.h"

#include "sds011.h"

// This EUI must be in BIG-ENDIAN format, so least-significant-byte first.
// For TTN issued EUIs the first bytes should be 0x70, 0xB3, 0xD5.
static const u1_t APPEUI[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x01, 0xA0, 0x9B };

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t APPKEY[] = {
    0xAA, 0x9F, 0x12, 0x45, 0x7F, 0x06, 0x64, 0xDF, 0x4C, 0x1E, 0x9F, 0xC9, 0x5E, 0xDA, 0x1A, 0x8A
};

#define OLED_I2C_ADDR 0x3C

#define PIN_OLED_RESET  16
#define PIN_OLED_SDA    4
#define PIN_OLED_SCL    15
#define PIN_BUTTON      0
#define PIN_SDS_RX      35
#define PIN_SDS_TX      25
#define PIN_LED         2

#define OTAA_MAGIC      "MAGIC"
#define UG_PER_M3       "\u00B5g/m\u00B3"

// total measurement cycle time (seconds)
#define TIME_CYCLE      145
// time to show version info
#define TIME_VERSION    5
// duration of warmup (seconds)
#define TIME_WARMUP     20
// duration of measurement (seconds)
#define TIME_MEASURE    10

typedef struct {
    u4_t netid = 0;
    devaddr_t devaddr = 0;
    u1_t nwkKey[16];
    u1_t artKey[16];
    u1_t dn2Dr;
    u1_t rx1DrOffset;
    u1_t rxDelay;
    u4_t channelFreq[MAX_CHANNELS];
    u2_t channelDrMap[MAX_CHANNELS];
    u4_t channelDlFreq[MAX_CHANNELS];
    band_t bands[MAX_BANDS];
    u2_t channelMap;

    char magic[8];
} otaa_data_t;

typedef struct {
    float humidity;
    float temperature;
    float pressure;
} bme_meas_t;

typedef struct {
    bool update;
    // 1st line: LoRa address
    char loraDevEui[32];
    // 2nd line: LoRa status
    char loraStatus[32];
    // 3rd line: PM10
    String dust1;
    // 4th line: PM2.5
    String dust2;
} screen_t;

// main state machine
typedef enum {
    E_INIT,
    E_IDLE,
    E_WARMUP,
    E_MEASURE
} fsm_state_t;

// Pin mapping
const lmic_pinmap lmic_pins = *Arduino_LMIC::GetPinmap_ttgo_lora32_v1();

static fsm_state_t main_state;

// stored in "little endian" format
static uint8_t deveui[8];
static SSD1306 display(OLED_I2C_ADDR, PIN_OLED_SDA, PIN_OLED_SCL);
static BME280 bme280;
static bool bmeFound = false;
static char bmeVersion[8] = "FAIL!";
static HardwareSerial sdsSerial(1);
static bool sdsFound = false;
static char sdsVersion[8] = "FAIL!";
static SDS011 sds;
static screen_t screen;

// average dust measument
static sds_meas_t avg;

// This should also be in little endian format, see above.
void os_getDevEui(u1_t * buf)
{
    for (int i = 0; i < 8; i++) {
        buf[i] = deveui[7 - i];
    }
}

void os_getArtEui(u1_t * buf)
{
    for (int i = 0; i < 8; i++) {
        buf[i] = APPEUI[7 - i];
    }
}

void os_getDevKey(u1_t * buf)
{
    memcpy(buf, APPKEY, 16);
}

static void setLoraStatus(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(screen.loraStatus, sizeof(screen.loraStatus), fmt, args);
    va_end(args);

    screen.update = true;
}

const char *event_names[] = { LMIC_EVENT_NAME_TABLE__INIT };

static void otaa_save(void)
{
    otaa_data_t otaa_data;

    LMIC_getSessionKeys(&otaa_data.netid, &otaa_data.devaddr, otaa_data.nwkKey, otaa_data.artKey);
    otaa_data.dn2Dr = LMIC.dn2Dr;
    otaa_data.rx1DrOffset = LMIC.rx1DrOffset;
    otaa_data.rxDelay = LMIC.rxDelay;

    memcpy(otaa_data.channelFreq, LMIC.channelFreq, sizeof(otaa_data.channelFreq));
    memcpy(otaa_data.channelDrMap, LMIC.channelDrMap, sizeof(otaa_data.channelDrMap));
    memcpy(otaa_data.channelDlFreq, LMIC.channelDlFreq, sizeof(otaa_data.channelDlFreq));
    memcpy(otaa_data.bands, LMIC.bands, sizeof(otaa_data.bands));
    otaa_data.channelMap = LMIC.channelMap;

    strcpy(otaa_data.magic, OTAA_MAGIC);
    EEPROM.put(0, otaa_data);
    EEPROM.commit();
}

static bool otaa_restore(void)
{
    otaa_data_t otaa_data;

    EEPROM.get(0, otaa_data);
    if (strcmp(otaa_data.magic, OTAA_MAGIC) != 0) {
        return false;
    }
    LMIC_setSession(otaa_data.netid, otaa_data.devaddr, otaa_data.nwkKey, otaa_data.artKey);
    LMIC.dn2Dr = otaa_data.dn2Dr;
    LMIC.rx1DrOffset = otaa_data.rx1DrOffset;
    LMIC.rxDelay = otaa_data.rxDelay;

    memcpy(LMIC.channelFreq, otaa_data.channelFreq, sizeof(LMIC.channelFreq));
    memcpy(LMIC.channelDrMap, otaa_data.channelDrMap, sizeof(LMIC.channelDrMap));
    memcpy(LMIC.channelDlFreq, otaa_data.channelDlFreq, sizeof(LMIC.channelDlFreq));
    memcpy(LMIC.bands, otaa_data.bands, sizeof(LMIC.bands));
    LMIC.channelMap = otaa_data.channelMap;

    return true;
}

static void onEventCallback(void *user, ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    Serial.println(event_names[ev]);

    switch (ev) {
    case EV_JOINING:
        setLoraStatus("OTAA JOIN...");
        break;
    case EV_JOINED:
        otaa_save();
        setLoraStatus("JOIN OK!");
        break;
    case EV_JOIN_FAILED:
        setLoraStatus("JOIN failed!");
        break;
    case EV_REJOIN_FAILED:
        setLoraStatus("REJOIN failed!");
        break;
    case EV_TXCOMPLETE:
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println("Received ack");
        if (LMIC.dataLen) {
            Serial.print("Received ");
            Serial.print(LMIC.dataLen);
            Serial.println(" bytes of payload");
        }
        setLoraStatus("%08X-%d", LMIC.devaddr, LMIC.seqnoUp);
        break;
    case EV_TXSTART:
        setLoraStatus("Transmit SF%d", getSf(LMIC.rps) + 6);
        break;
    case EV_RXSTART:
        setLoraStatus("Receive SF%d", getSf(LMIC.rps) + 6);
        break;
    case EV_JOIN_TXCOMPLETE:
        setLoraStatus("JOIN sent");
        break;
    default:
        Serial.print("Unknown event: ");
        Serial.println((unsigned) ev);
        break;
    }
}

static int sds_exchange(uint8_t * cmd, int cmd_len, uint8_t * rsp, int rsp_size, int timeout)
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
        yield();
    }
    return 0;
}

static bool sds_version(char *serial, char *date)
{
    uint8_t cmd = 7;
    uint8_t rsp[10];
    int rsp_len = sds_exchange(&cmd, 1, rsp, sizeof(rsp), 1000);

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

static bool sds_fan(bool on)
{
    uint8_t cmd[3];
    uint8_t rsp[10];

    cmd[0] = 6;
    cmd[1] = 1;
    cmd[2] = on ? 1 : 0;
    int rsp_len = sds_exchange(cmd, sizeof(cmd), rsp, sizeof(rsp), 1000);
    if (rsp_len > 2) {
        return rsp[2] == cmd[2];
    }
    return false;
}

static void send_dust(sds_meas_t * meas, bme_meas_t * bme, bool bmeValid)
{
    uint8_t buf[20];

    if ((LMIC.opmode & (OP_TXDATA | OP_TXRXPEND)) == 0) {
        // encode it as Cayenne
        int idx = 0;

        // PM10
        buf[idx++] = 1;
        buf[idx++] = 2;
        int pm10 = meas->pm10 / 0.01;
        if (pm10 > 32767) {
            pm10 = 32767;
        }
        buf[idx++] = (pm10 >> 8) & 0xFF;
        buf[idx++] = (pm10 >> 0) & 0xFF;

        // PM2.5
        buf[idx++] = 2;
        buf[idx++] = 2;
        int pm2_5 = meas->pm2_5 / 0.01;
        if (pm2_5 > 32767) {
            pm2_5 = 32767;
        }
        buf[idx++] = (pm2_5 >> 8) & 0xFF;
        buf[idx++] = (pm2_5 >> 0) & 0xFF;

        if (bmeValid) {
            // temperature
            int tempInt = bme->temperature / 0.1;
            buf[idx++] = 3;
            buf[idx++] = 103;
            buf[idx++] = highByte(tempInt);
            buf[idx++] = lowByte(tempInt);

            // humidity
            int humiInt = bme->humidity / 0.5;
            buf[idx++] = 4;
            buf[idx++] = 104;
            buf[idx++] = humiInt;

            // pressure
            int presInt = bme->pressure / 10.0;
            buf[idx++] = 5;
            buf[idx++] = 115;
            buf[idx++] = highByte(presInt);
            buf[idx++] = lowByte(presInt);
        }
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, buf, idx, 0);
    }
}

static void screen_update(void)
{
    if (screen.update) {
        display.clear();

        // 1st line
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 0, screen.loraDevEui);

        // 2nd line
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 12, screen.loraStatus);

        // 3rd
        display.drawString(0, 30, screen.dust1);

        // 4th line
        display.drawString(0, 46, screen.dust2);

        display.display();
        screen.update = false;
    }
}

static void screen_format_dust(sds_meas_t * meas)
{
    char value[16];

    snprintf(value, sizeof(value), "PM 10:%3d ", (int) round(meas->pm10));
    screen.dust1 = String(value) + UG_PER_M3;

    snprintf(value, sizeof(value), "PM2.5:%3d ", (int) round(meas->pm2_5));
    screen.dust2 = String(value) + UG_PER_M3;

    screen.update = true;
}

static void screen_format_version(const char *sdsVersion, const char *bmeVersion)
{
    screen.dust1 = String("SDS011: ") + sdsVersion;
    screen.dust2 = String("BME280: ") + bmeVersion;
    screen.update = true;
}

static void set_fsm_state(fsm_state_t newstate)
{
    printf(">>> ");
    switch (newstate) {
    case E_INIT:
        printf("E_INIT");
        break;
    case E_IDLE:
        printf("E_IDLE");
        break;
    case E_WARMUP:
        printf("E_WARMUP");
        break;
    case E_MEASURE:
        printf("E_MEASURE");
        break;
    default:
        break;
    }
    printf("\n");
    main_state = newstate;
}

static void fsm_run(void)
{
    static sds_meas_t sum;
    static int sum_num = 0;
    sds_meas_t sds_meas;

    int sec = (millis() / 1000) % TIME_CYCLE;

    switch (main_state) {
    case E_INIT:
        // send command to get software version
        if (!bmeFound) {
            Serial.printf("Detecting BME280 ...\n");
            bmeFound = findBME280(bmeVersion);
            Serial.printf("Found BME280, i2c=%s\n", bmeVersion);
            screen_format_version(sdsVersion, bmeVersion);
        }
        if (!sdsFound) {
            char sdsDate[16];
            Serial.printf("Detecting SDS011 ...\n");
            sdsFound = sds_version(sdsVersion, sdsDate);
            Serial.printf("Found SDS011, serial=%s, date=%s\n", sdsVersion, sdsDate);
            screen_format_version(sdsVersion, bmeVersion);
        }
        if (sdsFound && (sec > TIME_VERSION)) {
            set_fsm_state(E_IDLE);
        }
        break;

    case E_IDLE:
        if (sec < TIME_WARMUP) {
            // turn fan on
            sds_fan(true);
            set_fsm_state(E_WARMUP);
        }
        break;

    case E_WARMUP:
        if (sec < TIME_WARMUP) {
            // read/show measurements while warming up
            while (sdsSerial.available()) {
                uint8_t c = sdsSerial.read();
                if (sds.process(c, 0xC0)) {
                    sds.getMeasurement(&sds_meas);
                    screen_format_dust(&sds_meas);
                }
            }
        } else {
            // reset sum
            sum.pm2_5 = 0.0;
            sum.pm10 = 0.0;
            sum_num = 0;
            set_fsm_state(E_MEASURE);
        }
        break;

    case E_MEASURE:
        if (sec < (TIME_WARMUP + TIME_MEASURE)) {
            // process measurement
            while (sdsSerial.available()) {
                uint8_t c = sdsSerial.read();
                if (sds.process(c, 0xC0)) {
                    // show individual measurement
                    sds.getMeasurement(&sds_meas);
                    screen_format_dust(&sds_meas);
                    // average it
                    sum.pm2_5 += sds_meas.pm2_5;
                    sum.pm10 += sds_meas.pm10;
                    sum_num++;
                }
            }
        } else {
            // turn the fan off
            sds_fan(false);

            // calculate average particulate matter
            if (sum_num > 0) {
                avg.pm2_5 = sum.pm2_5 / sum_num;
                avg.pm10 = sum.pm10 / sum_num;
                // take temperature/humidity sample
                bme_meas_t bme = {0.0, 0.0, 0.0};
                if (bmeFound) {
                    bme.temperature = bme280.readTempC();
                    bme.humidity = bme280.readFloatHumidity();
                    bme.pressure = bme280.readFloatPressure();
                }
                // show averaged particulate matter and send it
                screen_format_dust(&avg);
                send_dust(&avg, &bme, bmeFound);
            }
            set_fsm_state(E_IDLE);
        }
        break;

    default:
        set_fsm_state(E_INIT);
        break;
    }

    // when measuring, light the LED
    digitalWrite(PIN_LED, main_state == E_MEASURE);
}

static bool findBME280(char *version)
{
    bme280.setI2CAddress(0x76);
    if (bme280.beginI2C()) {
        strcpy(version, "0x76");
        return true;
    }
    bme280.setI2CAddress(0x77);
    if (bme280.beginI2C()) {
        strcpy(version, "0x77");
        return true;
    }
    return false;
}

void setup(void)
{
    Serial.begin(115200);
    Serial.println("Starting...");

    // LED config
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, 1);

    // button config
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    // init the OLED
    pinMode(PIN_OLED_RESET, OUTPUT);
    digitalWrite(PIN_OLED_RESET, LOW);
    delay(50);
    digitalWrite(PIN_OLED_RESET, HIGH);

    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    // initialize the SDS011 serial
    sdsSerial.begin(9600, SERIAL_8N1, PIN_SDS_RX, PIN_SDS_TX, false);

    // setup of unique ids
    uint64_t chipid = ESP.getEfuseMac();
    deveui[0] = (chipid >> 56) & 0xFF;
    deveui[1] = (chipid >> 48) & 0xFF;
    deveui[2] = (chipid >> 40) & 0xFF;
    deveui[3] = (chipid >> 32) & 0xFF;
    deveui[4] = (chipid >> 24) & 0xFF;
    deveui[5] = (chipid >> 16) & 0xFF;
    deveui[6] = (chipid >> 8) & 0xFF;
    deveui[7] = (chipid >> 0) & 0xFF;
    snprintf(screen.loraDevEui, sizeof(screen.loraDevEui),
             "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", deveui[0], deveui[1], deveui[2], deveui[3],
             deveui[4], deveui[5], deveui[6], deveui[7]);

    // LMIC init
    os_init();
    LMIC_reset();
    LMIC_registerEventCb(onEventCallback, NULL);

    EEPROM.begin(512);
    if (otaa_restore()) {
        setLoraStatus("Resume OTAA");
    } else {
        LMIC_startJoining();
    }

    // fan on, this makes the SDS011 respond to version commands
    sds_fan(true);
}

void loop(void)
{
    static unsigned long button_ts = 0;

    // check for long button press to restart OTAA
    unsigned long ms = millis();
    if (digitalRead(PIN_BUTTON) == 0) {
        if ((ms - button_ts) > 2000) {
            LMIC_reset();
            LMIC_startJoining();
            button_ts = ms;
        }
    } else {
        button_ts = ms;
    }

    // run the measurement state machine
    fsm_run();

    // update screen
    screen_update();

    // run LoRa process
    os_runloop_once();
}
