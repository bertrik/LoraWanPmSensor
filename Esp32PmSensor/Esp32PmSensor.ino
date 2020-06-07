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
#include "soc/efuse_reg.h"
#include "HardwareSerial.h"
#include "EEPROM.h"

#include "sds011.h"

// This EUI must be in little-endian format, so least-significant-byte first.
// For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
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

#define PIN_OLED_RESET  16
#define PIN_OLED_SDA    4
#define PIN_OLED_SCL    15
#define PIN_BUTTON      0
#define PIN_SDS_RX      35
#define PIN_SDS_TX      25

#define OTAA_MAGIC 0xCAFEBABE
#define UG_PER_M3  "\u00B5g/m\u00B3"

// total measurement cycle time (seconds)
#define TIME_CYCLE      60
// duration of warmup (seconds)
#define TIME_WARMUP     20
// duration of measurement (seconds)
#define TIME_MEASURE    10

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

// main state machine
typedef enum {
    E_INIT,
    E_IDLE,
    E_WARMUP,
    E_MEASURE
} fsm_state_t;

static fsm_state_t main_state;

// stored in "little endian" format
static uint8_t deveui[8];
static otaa_data_t otaa_data;
static SSD1306 display(OLED_I2C_ADDR, PIN_OLED_SDA, PIN_OLED_SCL);
static HardwareSerial sdsSerial(1);
static SDS011 sds;
static screen_t screen;

// average dust measument
static sds_meas_t avg;

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
const lmic_pinmap lmic_pins = *Arduino_LMIC::GetPinmap_ttgo_lora32_v1();

static void print_keys(void)
{
    Serial.print("netid: ");
    Serial.println(otaa_data.netid, DEC);
    Serial.print("devaddr: ");
    Serial.println(otaa_data.devaddr, HEX);
    Serial.print("artKey: ");
    for (int i = 0; i < sizeof(otaa_data.artKey); ++i) {
        Serial.printf("%02X", otaa_data.artKey[i]);
    }
    Serial.println("");
    Serial.print("nwkKey: ");
    for (int i = 0; i < sizeof(otaa_data.nwkKey); ++i) {
        Serial.printf("%02X", otaa_data.nwkKey[i]);
    }
    Serial.println("");
}

static void setLoraStatus(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(screen.loraStatus, sizeof(screen.loraStatus), fmt, args);
    va_end(args);

    screen.update = true;
}

const char *event_names[] = {LMIC_EVENT_NAME_TABLE__INIT};

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    Serial.println(event_names[ev]);

    switch (ev) {
    case EV_JOINING:
        setLoraStatus("OTAA JOIN...");
        break;
    case EV_JOINED:
        LMIC_getSessionKeys(&otaa_data.netid, &otaa_data.devaddr, otaa_data.nwkKey,
                            otaa_data.artKey);
        otaa_data.magic = OTAA_MAGIC;
        EEPROM.put(0, otaa_data);
        EEPROM.commit();

        print_keys();

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
            Serial.println(F("Received ack"));
        if (LMIC.dataLen) {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
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

static bool sds_version(char *version, int size)
{
    uint8_t cmd = 7;
    uint8_t rsp[10];
    int rsp_len = sds_exchange(&cmd, 1, rsp, sizeof(rsp), 1000);

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

static void send_dust(sds_meas_t * meas)
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

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, buf, idx, 0);
        Serial.println(F("Sending uplink packet..."));
    }
}

static void screen_update(sds_meas_t * meas, bool valid)
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
        if (valid) {
            snprintf(value, sizeof(value), "PM 10:%3d ", (int) round(meas->pm10));
        } else {
            snprintf(value, sizeof(value), "PM 10: - ");
        }
        display.drawString(0, 30, String(value) + UG_PER_M3);

        // 4th line
        if (valid) {
            snprintf(value, sizeof(value), "PM2.5:%3d ", (int) round(meas->pm2_5));
        } else {
            snprintf(value, sizeof(value), "PM2.5: - ");
        }
        display.drawString(0, 46, String(value) + UG_PER_M3);

        display.display();
        screen.update = false;
    }
}

static void set_fsm_state(fsm_state_t newstate)
{
    printf(">>> ");
    switch (newstate) {
    case E_INIT:    printf("E_INIT");     break;
    case E_IDLE:    printf("E_IDLE");     break;
    case E_WARMUP:  printf("E_WARMUP");   break;
    case E_MEASURE: printf("E_MEASURE");  break;
    default:
        break;
    }
    printf("\n");
    main_state = newstate;
}

static bool fsm_run(void)
{
    static sds_meas_t sum;
    static int sum_num = 0;
    static bool valid = false;

    int sec = (millis() / 1000) % TIME_CYCLE;

    switch (main_state) {
    case E_INIT:
        valid = false;

        // send command to get software version
        Serial.printf("Getting SDS version\n");
        char version[32];
        if (sds_version(version, sizeof(version))) {
            Serial.print("SDS version and date: ");
            Serial.println(version);
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
            // reset sum
            sum.pm2_5 = 0.0;
            sum.pm10 = 0.0;
            sum_num = 0;
            // flush input buffer
            while (sdsSerial.available()) {
                sdsSerial.read();
            }
        } else {
            set_fsm_state(E_MEASURE);
        }
        break;

    case E_MEASURE:
        if (sec < (TIME_WARMUP + TIME_MEASURE)) {
            // process measurement
            while (sdsSerial.available()) {
                uint8_t c = sdsSerial.read();
                if (sds.process(c, 0xC0)) {
                    // parse it
                    sds_meas_t sds_meas;
                    sds.getMeasurement(&sds_meas);
                    sum.pm2_5 += sds_meas.pm2_5;
                    sum.pm10 += sds_meas.pm10;
                    sum_num++;
                }
            }
        } else {
            // turn the fan off
            sds_fan(false);

            // get average filter value and send it
            if (sum_num > 0) {
                valid = true;
                avg.pm2_5 = sum.pm2_5 / sum_num;
                avg.pm10 = sum.pm10 / sum_num;
                send_dust(&avg);
            }

            set_fsm_state(E_IDLE);
        }
        break;

    default:
        set_fsm_state(E_INIT);
        break;
    }

    return valid;
}

void setup(void)
{
    Serial.begin(115200);
    Serial.println(F("Starting..."));

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
    deveui[0] = (chipid >> 0) & 0xFF;
    deveui[1] = (chipid >> 8) & 0xFF;
    deveui[2] = (chipid >> 16) & 0xFF;
    deveui[3] = (chipid >> 24) & 0xFF;
    deveui[4] = (chipid >> 32) & 0xFF;
    deveui[5] = (chipid >> 40) & 0xFF;
    deveui[6] = (chipid >> 48) & 0xFF;
    deveui[7] = (chipid >> 56) & 0xFF;
    snprintf(screen.loraDevEui, sizeof(screen.loraDevEui),
             "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", deveui[7], deveui[6], deveui[5], deveui[4],
             deveui[3], deveui[2], deveui[1], deveui[0]);

    // LMIC init
    os_init();
    LMIC_reset();
    EEPROM.begin(512);
    EEPROM.get(0, otaa_data);
    if (otaa_data.magic == OTAA_MAGIC) {
        setLoraStatus("Resume OTAA");
        LMIC_setSession(otaa_data.netid, otaa_data.devaddr, otaa_data.nwkKey, otaa_data.artKey);
        print_keys();
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
            Serial.println("Resetting OTAA");
            LMIC_reset();
            LMIC_startJoining();
            button_ts = ms;
        }
    } else {
        button_ts = ms;
    }

    // run the state machine
    bool have_data = fsm_run();

    // update screen
    screen_update(&avg, have_data);

    // run LoRa process
    os_runloop_once();
}

