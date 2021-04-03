// MIT License
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <Arduino.h>
#include <EEPROM.h>
#include "lmic.h"
#include <hal/hal.h>
#include "arduino_lmic_hal_boards.h"

#include <SPI.h>
#include <SSD1306.h>
#include <SparkFunBME280.h>
#include "soc/efuse_reg.h"
#include "HardwareSerial.h"

// OTA
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "sds011.h"
#include "sps30.h"

#include "editline.h"
#include "cmdproc.h"
#include "aggregator.h"

// This EUI must be in BIG-ENDIAN format, most-significant byte (MSB).
// For TTN issued EUIs the first bytes should be 0x70, 0xB3, 0xD5.
static const uint8_t APPEUI[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x01, 0xA0, 0x9B };

// This key should be in big endian format as well, see above.
static const uint8_t APPKEY[] = {
    0xAA, 0x9F, 0x12, 0x45, 0x7F, 0x06, 0x64, 0xDF, 0x4C, 0x1E, 0x9F, 0xC9, 0x5E, 0xDA, 0x1A, 0x8A
};

#define printf Serial.printf

#define OLED_I2C_ADDR 0x3C

#define PIN_OLED_RESET  16
#define PIN_OLED_SDA    4
#define PIN_OLED_SCL    15
#define PIN_BUTTON      0
#define PIN_SDS_RX      22
#define PIN_SDS_TX      23
#define PIN_VEXT        21

#define UG_PER_M3       "\u00B5g/m\u00B3"

// total measurement cycle time (seconds)
#define TIME_CYCLE      300
// time to show version info
#define TIME_VERSION    5
// duration of warmup (seconds)
#define TIME_WARMUP     20
// duration of measurement (seconds)
#define TIME_MEASURE    10
// reboot interval (seconds)
#define REBOOT_INTERVAL 2592000UL
// time to keep display on (seconds)
#define TIME_OLED_ENABLED   600

// how we know the non-volatile storage contains meaningful data
#define NVDATA_MAGIC    "magic"

// structure of non-volatile data
typedef struct {
    uint8_t deveui[8];
    uint8_t appeui[8];
    uint8_t appkey[16];
    char magic[8];
} nvdata_t;

typedef struct {
    float humidity;
    float temperature;
    float pressure;
} bme_meas_t;

typedef struct {
    bool enabled;
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
    E_INIT = 0,
    E_IDLE,
    E_WARMUP,
    E_MEASURE,
    E_SEND,
    E_LAST
} fsm_state_t;

typedef enum {
    E_ITEM_PM1_0,
    E_ITEM_PM2_5,
    E_ITEM_PM4_0,
    E_ITEM_PM10,
    E_ITEM_TEMPERATURE,
    E_ITEM_HUMIDITY,
    E_ITEM_PRESSURE,
    E_ITEM_MAX
} item_t;

typedef enum {
    E_PMSENSOR_NONE,
    E_PMSENSOR_SDS011,
    E_PMSENSOR_SPS30
} pmsensor_t;

// Pin mapping
const lmic_pinmap lmic_pins = *Arduino_LMIC::GetPinmap_ThisBoard();

// each measurement cycle takes 5 minutes, this table specifies how many cycles there are per transmission
static const int interval_table[] = {
    1,  // SF6
    1,  // SF7
    1,  // SF8
    2,  // SF9
    4,  // SF10
    8,  // SF11
    16  // SF12
};

static fsm_state_t main_state;

static SSD1306 display(OLED_I2C_ADDR, PIN_OLED_SDA, PIN_OLED_SCL);
static BME280 bme280;
static bool bmeFound = false;
static char bmeVersion[8] = "FAIL!";
static HardwareSerial serial(1);

static SDS011 sds(&serial, true);
//static bool sdsFound = false;
static char sdsVersion[8] = "FAIL!";

static SPS30 sps(&serial);
static pmsensor_t pmsensor = E_PMSENSOR_NONE;

static screen_t screen;
static unsigned long screen_last_enabled = 0;
static nvdata_t nvdata;
static char cmdline[100];
static rps_t last_tx_rps = 0;

static Aggregator aggregator(E_ITEM_MAX);

void os_getDevEui(u1_t * buf)
{
    for (int i = 0; i < 8; i++) {
        buf[i] = nvdata.deveui[7 - i];
    }
}

void os_getArtEui(u1_t * buf)
{
    for (int i = 0; i < 8; i++) {
        buf[i] = nvdata.appeui[7 - i];
    }
}

void os_getDevKey(u1_t * buf)
{
    memcpy(buf, nvdata.appkey, 16);
}

// saves OTAA keys to EEPROM
static void otaa_save(const uint8_t deveui[8], const uint8_t appeui[8], const uint8_t appkey[16])
{
    memcpy(&nvdata.deveui, deveui, 8);
    memcpy(&nvdata.appeui, appeui, 8);
    memcpy(&nvdata.appkey, appkey, 16);
    strcpy(nvdata.magic, NVDATA_MAGIC);
    EEPROM.put(0, nvdata);
    EEPROM.commit();
}

// restores OTAA parameters from EEPROM, returns false if there are none
static bool otaa_restore(void)
{
    EEPROM.get(0, nvdata);
    return strcmp(nvdata.magic, NVDATA_MAGIC) == 0;
}

// sets OTAA to defaults
static void otaa_defaults(void)
{
    uint64_t chipid = ESP.getEfuseMac();

    // setup of unique ids
    uint8_t deveui[8];
    deveui[0] = (chipid >> 56) & 0xFF;
    deveui[1] = (chipid >> 48) & 0xFF;
    deveui[2] = (chipid >> 40) & 0xFF;
    deveui[3] = (chipid >> 32) & 0xFF;
    deveui[4] = (chipid >> 24) & 0xFF;
    deveui[5] = (chipid >> 16) & 0xFF;
    deveui[6] = (chipid >> 8) & 0xFF;
    deveui[7] = (chipid >> 0) & 0xFF;
    otaa_save(deveui, APPEUI, APPKEY);
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
        setLoraStatus("JOIN OK!");
        break;
    case EV_JOIN_FAILED:
        setLoraStatus("JOIN failed!");
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
        last_tx_rps = LMIC.rps;
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

static void printhex(const uint8_t * buf, int len)
{
    for (int i = 0; i < len; i++) {
        printf("%02X", buf[i]);
    }
    printf("\n");
}

static void parsehex(const char *hex, uint8_t *buf, int len)
{
    char tmp[4];
    for (int i = 0; i < len; i++) {
        strncpy(tmp, hex, 2);
        *buf++ = strtoul(tmp, NULL, 16);
        hex += 2;
    }
}

static void add_cayenne_16bit(uint8_t *buf, int &index, item_t item, int channel, int type, double unit)
{
    double value;
    if (aggregator.get(item, value)) {
        int intval = value / unit;
        if (intval < 0) {
            intval = 0;
        } else if (intval > 32767) {
            intval = 32767;
        }
        buf[index++] = channel;
        buf[index++] = type;
        buf[index++] = highByte(intval);
        buf[index++] = lowByte(intval);
    }
}

static void add_cayenne_8bit(uint8_t *buf, int &index, item_t item, int channel, int type,  double unit)
{
    double value;
    if (aggregator.get(item, value)) {
        int intval = value / unit;
        buf[index++] = channel;
        buf[index++] = type;
        buf[index++] = lowByte(intval);
    }
}

static void send_dust(void)
{
    if ((LMIC.opmode & (OP_TXDATA | OP_TXRXPEND)) == 0) {
        // encode it as Cayenne
        uint8_t buf[32];
        int idx = 0;
        add_cayenne_16bit(buf, idx, E_ITEM_PM1_0, 0, 2, 0.01);
        add_cayenne_16bit(buf, idx, E_ITEM_PM10, 1, 2, 0.01);
        add_cayenne_16bit(buf, idx, E_ITEM_PM2_5, 2, 2, 0.01);
        add_cayenne_16bit(buf, idx, E_ITEM_PM4_0, 4, 2, 0.01);
        add_cayenne_16bit(buf, idx, E_ITEM_TEMPERATURE, 0, 103, 0.1);
        add_cayenne_8bit(buf, idx, E_ITEM_HUMIDITY, 0, 104, 0.5);
        add_cayenne_16bit(buf, idx, E_ITEM_PRESSURE, 0, 115, 10.0);

        // Prepare upstream data transmission at the next possible time.
        printf("Sending:");
        printhex(buf, idx);
        LMIC_setTxData2(1, buf, idx, 0);
    }
}

static void screen_update(unsigned long int second)
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
    if (screen.enabled && ((second - screen_last_enabled) > TIME_OLED_ENABLED)) {
        display.displayOff();
        screen.enabled = false;
    }
}

static void screen_format_dust(void)
{
    char line[16];
    double value;

    if (aggregator.get(E_ITEM_PM10, value)) {
        snprintf(line, sizeof(line), "PM 10:%3d ", (int) round(value));
        screen.dust1 = String(line) + UG_PER_M3;
    }
    if (aggregator.get(E_ITEM_PM2_5, value)) {
        snprintf(line, sizeof(line), "PM2.5:%3d ", (int) round(value));
        screen.dust2 = String(line) + UG_PER_M3;
    }
    screen.update = true;
}

static void set_fsm_state(fsm_state_t newstate)
{
    if (newstate < E_LAST) {
        static const char* states[] = { "E_INIT", "E_IDLE", "E_WARMUP", "E_MEASURE", "E_SEND" };
        printf(">>> %s\n", states[newstate]);
    }
    main_state = newstate;
}

static void pmsensor_on_off(boolean on)
{
    switch (pmsensor) {
    case E_PMSENSOR_SDS011:
        sds.fan(on);
        break;
    case E_PMSENSOR_SPS30:
        if (on) {
            sps.start();
        } else {
            sps.stop();
        }
        break;
    default:
        break;
    }
}

// return true if new measurement available
static bool pmsensor_measure(void)
{
    sds_meas_t meas;
    uint16_t pm1_0, pm2_5, pm4_0, pm10, ps;

    switch (pmsensor) {
    case E_PMSENSOR_SDS011:
        if (sds.poll(&meas)) {
            aggregator.add(E_ITEM_PM2_5, meas.pm2_5);
            aggregator.add(E_ITEM_PM10, meas.pm10);
            return true;
        }
        break;
    case E_PMSENSOR_SPS30:
        if (sps.read_measurement(&pm1_0, &pm2_5, &pm4_0, &pm10, &ps)) {
            aggregator.add(E_ITEM_PM1_0, pm1_0);
            aggregator.add(E_ITEM_PM2_5, pm2_5);
            aggregator.add(E_ITEM_PM4_0, pm4_0);
            aggregator.add(E_ITEM_PM10, pm10);
            return true;
        }
        break;
    default:
        break;
    }
    return false;
}

static void fsm_run(unsigned long int seconds)
{
    static int cycle = 0;

    unsigned long int sec = seconds % TIME_CYCLE;

    switch (main_state) {
    case E_INIT:
        if (pmsensor == E_PMSENSOR_NONE) {
            // detect SPS30
            printf("Detecting SPS30 sensor ...\n");
            serial.begin(115200, SERIAL_8N1, PIN_SDS_RX, PIN_SDS_TX);
            if (sps.wakeup()) {
                printf("Found SPS30\n");
                pmsensor = E_PMSENSOR_SPS30;
                screen.dust1 = String("PM: SPS30");
                screen.update = true;
                break;
            }
            // detect SDS011
            printf("Detecting SDS011 sensor ...\n");
            serial.begin(9600, SERIAL_8N1, PIN_SDS_RX, PIN_SDS_TX);
            if (sds.fan(true) || sds.fan(true)) {
                printf("Found SDS011\n");
                pmsensor = E_PMSENSOR_SDS011;
                screen.dust1 = String("PM: SDS011");
                screen.update = true;
                break;
            }
        } else {
            set_fsm_state(E_IDLE);
        }
        break;

    case E_IDLE:
        if (sec < TIME_WARMUP) {
            // turn fan on
            pmsensor_on_off(true);
            aggregator.reset();
            set_fsm_state(E_WARMUP);
        }
        break;

    case E_WARMUP:
        if (sec < TIME_WARMUP) {
            // read/show measurements while warming up
            if (pmsensor_measure()) {
                screen_format_dust();
            }
        } else {
            // reset sum
            aggregator.reset();
            set_fsm_state(E_MEASURE);
        }
        break;

    case E_MEASURE:
        if (sec < (TIME_WARMUP + TIME_MEASURE)) {
            if (pmsensor_measure()) {
                screen_format_dust();
            }
        } else {
            // turn the fan off
            pmsensor_on_off(false);

            // take temperature/humidity sample
            if (bmeFound) {
                aggregator.add(E_ITEM_TEMPERATURE, bme280.readTempC());
                aggregator.add(E_ITEM_HUMIDITY, bme280.readFloatHumidity());
                aggregator.add(E_ITEM_PRESSURE, bme280.readFloatPressure());
            }
            set_fsm_state(E_SEND);
        }
        break;

    case E_SEND:
        if (sec >= (TIME_WARMUP + TIME_MEASURE)) {
            // send data with an interval depending on the current spreading factor
            int sf = getSf(last_tx_rps);
            int interval = interval_table[sf];
            printf("SF %d, cycle %d / interval %d\n", sf + 6, cycle, interval);
            if ((cycle % interval) == 0) {
                send_dust();
            }
            cycle++;
            set_fsm_state(E_IDLE);
        }
        break;

    default:
        set_fsm_state(E_INIT);
        break;
    }

    // when measuring, light the LED
    digitalWrite(LED_BUILTIN, main_state == E_MEASURE);
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

static void show_help(const cmd_t * cmds)
{
    for (const cmd_t * cmd = cmds; cmd->cmd != NULL; cmd++) {
        printf("%10s: %s\n", cmd->name, cmd->help);
    }
}

static int do_help(int argc, char *argv[]);

static int do_reboot(int argc, char *argv[])
{
    ESP.restart();
    return CMD_OK;
}

static int do_otaa(int argc, char *argv[])
{
    // reset OTAA
    if ((argc == 2) && (strcmp(argv[1], "reset") == 0)) {
        printf("Resetting OTAA to defaults\n");
        otaa_defaults();
    }

    // save OTAA parameters
    if (argc == 4) {
        printf("Setting OTAA parameters\n");
        char *deveui_hex = argv[1];
        char *appeui_hex = argv[2];
        char *appkey_hex = argv[3];
        if ((strlen(deveui_hex) != 16) || (strlen(appeui_hex) != 16) || (strlen(appkey_hex) != 32)) {
            return CMD_ARG;
        }

        uint8_t deveui[8];
        uint8_t appeui[8];
        uint8_t appkey[16];
        parsehex(deveui_hex, deveui, 8);
        parsehex(appeui_hex, appeui, 8);
        parsehex(appkey_hex, appkey, 16);
        otaa_save(deveui, appeui, appkey);
    }

    // show current OTAA parameters
    printf("Dev EUI = ");
    printhex(nvdata.deveui, 8);
    printf("App EUI = ");
    printhex(nvdata.appeui, 8);
    printf("App key = ");
    printhex(nvdata.appkey, 16);

    return CMD_OK;
}

const cmd_t commands[] = {
    { "help", do_help, "Show help" },
    { "reboot", do_reboot, "Reboot ESP" },
    { "otaa", do_otaa, "[reset|<[deveui] [appeui] [appkey]>] Query/reset/set OTAA parameters" },
    { NULL, NULL, NULL }
};

static int do_help(int argc, char *argv[])
{
    show_help(commands);
    return CMD_OK;
}

void setup(void)
{
    Serial.begin(115200);
    Serial.println("Starting...");

    EditInit(cmdline, sizeof(cmdline));

    // VEXT config: 0 = enable Vext
    pinMode(PIN_VEXT, OUTPUT);
    digitalWrite(PIN_VEXT, 0);

    // LED config
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 0);

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
    screen.enabled = true;

    // restore LoRaWAN keys from EEPROM, or use a default
    EEPROM.begin(sizeof(nvdata));
    if (!otaa_restore()) {
        otaa_defaults();
    }
    snprintf(screen.loraDevEui, sizeof(screen.loraDevEui),
             "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", nvdata.deveui[0], nvdata.deveui[1], nvdata.deveui[2],
             nvdata.deveui[3], nvdata.deveui[4], nvdata.deveui[5], nvdata.deveui[6], nvdata.deveui[7]);

    // LMIC init
    os_init();
    LMIC_reset();
    LMIC_registerEventCb(onEventCallback, NULL);
    LMIC_startJoining();

    // detect BME280
    printf("Detecting BME280 ...\n");
    bmeFound = findBME280(bmeVersion);
    if (bmeFound) {
        printf("Found BME280, i2c=%s\n", bmeVersion);
        screen.dust2 = String("BME280:") + bmeVersion;
        screen.update = true;
    }

    // fan on, this makes the SDS011 respond to version commands
    sds.fan(true);

    // OTA init
    uint64_t chipid = ESP.getEfuseMac();
    char ssid[32];
    sprintf(ssid, "ESP32-%08X%08X", (uint32_t)(chipid >> 32), (uint32_t)chipid);
    WiFi.softAP(ssid);
    ArduinoOTA.setHostname("esp32-pmsensor");
    ArduinoOTA.begin();
}

void loop(void)
{
    unsigned long ms = millis();
    unsigned long second = ms / 1000UL;

    // parse command line
    if (Serial.available()) {
        char c;
        bool haveLine = EditLine(Serial.read(), &c);
        Serial.write(c);
        if (haveLine) {
            int result = cmd_process(commands, cmdline);
            switch (result) {
            case CMD_OK:
                printf("OK\n");
                break;
            case CMD_NO_CMD:
                break;
            case CMD_ARG:
                printf("Invalid arguments\n");
                break;
            case CMD_UNKNOWN:
                printf("Unknown command, available commands:\n");
                show_help(commands);
                break;
            default:
                printf("%d\n", result);
                break;
            }
            printf(">");
        }
    }

    // button press re-enabled the display
    if (digitalRead(PIN_BUTTON) == 0) {
        if (!screen.enabled) {
            display.displayOn();
            screen_last_enabled = second;
            screen.enabled = true;
        }
    }

    // run the measurement state machine
    fsm_run(second);

    // update screen
    screen_update(second);

    // run LoRa process
    os_runloop_once();

    // reboot every 30 days
    if (second > REBOOT_INTERVAL) {
        printf("Reboot ...\n");
        ESP.restart();
        while (true);
    }

    // run the OTA process
    ArduinoOTA.handle();
}

