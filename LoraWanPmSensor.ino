// MIT License
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <Arduino.h>
#include <EEPROM.h>
#include <WebServer.h>
#include <ElegantOTA.h>

#include "lmic.h"
#include <hal/hal.h>
#include "arduino_lmic_hal_boards.h"

#include <SPI.h>
#include <SSD1306.h>
#include <SparkFunBME280.h>
#include "soc/efuse_reg.h"
#include "HardwareSerial.h"
#include "lwqrcode.h"

// OTA
#include <WiFi.h>
#include <DNSServer.h>

#include "sds011.h"
#include "sps30.h"

#include "printf.h"
#include "hexutil.h"
#include "editline.h"
#include "cmdproc.h"
#include "aggregator.h"
#include "item.h"

// This EUI must be in BIG-ENDIAN format, most-significant byte (MSB).
// For TTN issued EUIs the first bytes should be 0x70, 0xB3, 0xD5.
static const uint8_t APPEUI[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x01, 0xA0, 0x9B };

// This key should be in big endian format as well, see above.
static const uint8_t APPKEY[] = {
    0xAA, 0x9F, 0x12, 0x45, 0x7F, 0x06, 0x64, 0xDF, 0x4C, 0x1E, 0x9F, 0xC9, 0x5E, 0xDA, 0x1A, 0x8A
};

#define DEFAULT_WIFI_PASSWORD   "pmsensor"

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
// time to keep display on (ms)
#define TIME_OLED_ENABLED   300000UL

// how we know the non-volatile storage contains meaningful data
#define NVDATA_MAGIC    "magic"

// structure of non-volatile data
typedef struct {
    uint8_t deveui[8];
    uint8_t appeui[8];
    uint8_t appkey[16];
    char wifipass[64];
    char magic[8];
} nvdata_t;

typedef enum {
    E_DISPLAYMODE_HWINFO,
    E_DISPLAYMODE_MEASUREMENTS,
    E_DISPLAYMODE_QRCODE,
    E_DISPLAYMODE_OFF
} displaymode_t;

// data structures related to information shown on screen
typedef struct {
    bool enabled;
    bool update;
    char loraDevEui[32];
    char loraStatus[32];
    displaymode_t displaymode = E_DISPLAYMODE_HWINFO;
    char pmsensor_name[32];
    char rhsensor_name[32];
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

static DNSServer dnsServer;
static char board_name[32];
static fsm_state_t main_state;
static WebServer webServer(80);
static SSD1306 display(OLED_I2C_ADDR, PIN_OLED_SDA, PIN_OLED_SCL);
static bool has_external_display = false;
static BME280 bme280;
static bool bmeFound = false;
static HardwareSerial serial(1);
static SDS011 sds(&serial);
static SPS30 sps(&serial);
static pmsensor_t pmsensor = E_PMSENSOR_NONE;
static screen_t screen;
static unsigned long button_last_pressed = 0;
static nvdata_t nvdata;
static char cmdline[100];
static rps_t last_tx_rps = 0;
static bool has_joined_otaa = false;

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

// saves settings to EEPROM
static void nvdata_save(void)
{
    strcpy(nvdata.magic, NVDATA_MAGIC);
    EEPROM.put(0, nvdata);
    EEPROM.commit();
}

// restores settings from EEPROM, restoring default is no valid settings were found 
static void nvdata_load(void)
{
    EEPROM.get(0, nvdata);
    if (strcmp(nvdata.magic, NVDATA_MAGIC) != 0) {
        memset(&nvdata, 0, sizeof(nvdata));

        // default OTAA settings
        uint64_t chipid = ESP.getEfuseMac();
        nvdata.deveui[0] = (chipid >> 56) & 0xFF;
        nvdata.deveui[1] = (chipid >> 48) & 0xFF;
        nvdata.deveui[2] = (chipid >> 40) & 0xFF;
        nvdata.deveui[3] = (chipid >> 32) & 0xFF;
        nvdata.deveui[4] = (chipid >> 24) & 0xFF;
        nvdata.deveui[5] = (chipid >> 16) & 0xFF;
        nvdata.deveui[6] = (chipid >> 8) & 0xFF;
        nvdata.deveui[7] = (chipid >> 0) & 0xFF;
        memcpy(&nvdata.appeui, APPEUI, 8);
        memcpy(&nvdata.appkey, APPKEY, 16);

        // default WiFi settings
        strcpy(nvdata.wifipass, DEFAULT_WIFI_PASSWORD);

        nvdata_save();
    }
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
        has_joined_otaa = false;
        break;
    case EV_JOINED:
        setLoraStatus("JOIN OK!");
        has_joined_otaa = true;
        break;
    case EV_JOIN_FAILED:
        setLoraStatus("JOIN failed!");
        has_joined_otaa = false;
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

static void add_cayenne_u16(uint8_t *buf, int &index, item_t item, int channel, int type, double unit)
{
    double value;
    if (aggregator.get(item, value)) {
        int intval = value / unit;
        intval = constrain(intval, 0, 32767);
        buf[index++] = channel;
        buf[index++] = type;
        buf[index++] = highByte(intval);
        buf[index++] = lowByte(intval);
    }
}

static void add_cayenne_s16(uint8_t *buf, int &index, item_t item, int channel, int type, double unit)
{
    double value;
    if (aggregator.get(item, value)) {
        int intval = value / unit;
        intval = constrain(intval, -32768, 32767);
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

static void add_be_16bit(uint8_t *buf, int &index, item_t item, double unit)
{
    double value;
    if (aggregator.get(item, value)) {
        int intval = value / unit;
        buf[index++] = (intval >> 8) & 0xFF;
        buf[index++] = (intval >> 0) & 0xFF;
    }
}

static bool send_dust(void)
{
    uint8_t buf[32];
    int idx = 0;

    if ((LMIC.opmode & (OP_TXDATA | OP_TXRXPEND)) != 0) {
        return false;
    }

    int port;
    if (pmsensor == E_PMSENSOR_SPS30) {
        // encode as custom SPS30
        port = 30;
        add_be_16bit(buf, idx, E_ITEM_PM1_0, 0.1);
        add_be_16bit(buf, idx, E_ITEM_PM2_5, 0.1);
        add_be_16bit(buf, idx, E_ITEM_PM4_0, 0.1);
        add_be_16bit(buf, idx, E_ITEM_PM10, 0.1);
        add_be_16bit(buf, idx, E_ITEM_N0_5, 1.0);
        add_be_16bit(buf, idx, E_ITEM_N1_0, 1.0);
        add_be_16bit(buf, idx, E_ITEM_N2_5, 1.0);
        add_be_16bit(buf, idx, E_ITEM_N4_0, 1.0);
        add_be_16bit(buf, idx, E_ITEM_N10, 1.0);
        add_be_16bit(buf, idx, E_ITEM_TPS, 0.001);
    } else {
        // encode as Cayenne
        port = 1;
        add_cayenne_u16(buf, idx, E_ITEM_PM1_0, 0, 2, 0.01);
        add_cayenne_u16(buf, idx, E_ITEM_PM10, 1, 2, 0.01);
        add_cayenne_u16(buf, idx, E_ITEM_PM2_5, 2, 2, 0.01);
        add_cayenne_u16(buf, idx, E_ITEM_PM4_0, 4, 2, 0.01);
        add_cayenne_8bit(buf, idx, E_ITEM_HUMIDITY, 10, 104, 0.5);
        add_cayenne_s16(buf, idx, E_ITEM_TEMPERATURE, 11, 103, 0.1);
        add_cayenne_u16(buf, idx, E_ITEM_PRESSURE, 12, 115, 10.0);
    }
    printf("Sending on port %d, ", port);
    hexprint("data ", buf, idx);
    LMIC_setTxData2(port, buf, idx, 0);
    return true;
}

static void screen_update(unsigned long int second)
{
    char line[16];
    double value;

    // do nothing if nothing to do, or already done
    if (!screen.update) {
        return;
    }

    switch (screen.displaymode) {
    case E_DISPLAYMODE_HWINFO:
        display.displayOn();
        display.clear();
        display.setColor(WHITE);
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 0, screen.loraDevEui);
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 14, board_name);
        display.drawString(0, 30, screen.pmsensor_name);
        display.drawString(0, 46, screen.rhsensor_name);
        display.display();
        break;
    case E_DISPLAYMODE_MEASUREMENTS:
        display.displayOn();
        display.clear();
        display.setColor(WHITE);
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 0, screen.loraDevEui);
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 14, screen.loraStatus);
        if (aggregator.get(E_ITEM_PM10, value)) {
            snprintf(line, sizeof(line), "PM 10:%3d ", (int) round(value));
            display.drawString(0, 30, String(line) + UG_PER_M3);
        }
        if (aggregator.get(E_ITEM_PM2_5, value)) {
            snprintf(line, sizeof(line), "PM2.5:%3d ", (int) round(value));
            display.drawString(0, 46, String(line) + UG_PER_M3);
        }
        display.display();
        break;
    case E_DISPLAYMODE_QRCODE:
        display.displayOn();
        qrcode_show(&display, nvdata.appeui, nvdata.deveui, nvdata.appkey);
        break;
    case E_DISPLAYMODE_OFF:
        display.displayOff();
        break;
    default:
        break;
    }
    screen.update = false;
}

static void set_display_mode(displaymode_t new_mode)
{
    screen.displaymode = new_mode;
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

static bool pmsensor_on_off(boolean on)
{
    switch (pmsensor) {
    case E_PMSENSOR_SDS011:
        return sds.fan(on);
    case E_PMSENSOR_SPS30:
        return on ? sps.start() : sps.stop();
    default:
        break;
    }
    return false;
}

// return true if new measurement available
static bool pmsensor_measure(void)
{
    sds_meas_t sds_meas;
    sps_meas_t sps_meas;

    switch (pmsensor) {
    case E_PMSENSOR_SDS011:
        if (sds.poll(&sds_meas)) {
            aggregator.add(E_ITEM_PM2_5, sds_meas.pm2_5);
            aggregator.add(E_ITEM_PM10, sds_meas.pm10);
            return true;
        }
        break;
    case E_PMSENSOR_SPS30:
        if (sps.read_measurement(&sps_meas)) {
            aggregator.add(E_ITEM_PM1_0, sps_meas.pm1_0);
            aggregator.add(E_ITEM_PM2_5, sps_meas.pm2_5);
            aggregator.add(E_ITEM_PM4_0, sps_meas.pm4_0);
            aggregator.add(E_ITEM_PM10, sps_meas.pm10);
            aggregator.add(E_ITEM_N0_5, sps_meas.n0_5);
            aggregator.add(E_ITEM_N1_0, sps_meas.n1_0);
            aggregator.add(E_ITEM_N2_5, sps_meas.n2_5);
            aggregator.add(E_ITEM_N4_0, sps_meas.n4_0);
            aggregator.add(E_ITEM_N10, sps_meas.n10);
            aggregator.add(E_ITEM_TPS, sps_meas.tps);
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
                snprintf(screen.pmsensor_name, sizeof(screen.pmsensor_name), "SPS30");
                break;
            }
            // detect SDS011
            printf("Detecting SDS011 sensor ...\n");
            serial.begin(9600, SERIAL_8N1, PIN_SDS_RX, PIN_SDS_TX);
            if (sds.fan(true) || sds.fan(true)) {
                printf("Found SDS011\n");
                char serial[16], date[16];
                if (sds.version(serial, date)) {
                    printf("SDS011: %s, %s\n", serial, date);
                }
                pmsensor = E_PMSENSOR_SDS011;
                snprintf(screen.pmsensor_name, sizeof(screen.pmsensor_name), "SDS011: %s", serial);
                break;
            }
        } else {
            set_display_mode(E_DISPLAYMODE_HWINFO);
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
        if (sec > TIME_WARMUP) {
            // reset sum
            aggregator.reset();
            set_display_mode(has_joined_otaa ? E_DISPLAYMODE_MEASUREMENTS : E_DISPLAYMODE_QRCODE);
            set_fsm_state(E_MEASURE);
        }
        break;

    case E_MEASURE:
        if (sec < (TIME_WARMUP + TIME_MEASURE)) {
            if (pmsensor_measure()) {
                screen.update = true;
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

static bool findDisplay(TwoWire *wire, int pinSda, int pinScl, uint8_t address)
{
    wire->begin(pinSda, pinScl);
    wire->beginTransmission(address);
    return (wire->endTransmission() == 0);
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
        memset(&nvdata, 0, sizeof(nvdata));
        EEPROM.put(0, nvdata);
        nvdata_load();
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

        hexparse(deveui_hex, nvdata.deveui, 8);
        hexparse(appeui_hex, nvdata.appeui, 8);
        hexparse(appkey_hex, nvdata.appkey, 16);
        nvdata_save();
    }

    // show current OTAA parameters
    hexprint("Dev EUI: ", nvdata.deveui, 8);
    hexprint("App EUI: ", nvdata.appeui, 8);
    hexprint("App key: ", nvdata.appkey, 16);

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

    // detect the display
    pinMode(PIN_OLED_RESET, OUTPUT);
    digitalWrite(PIN_OLED_RESET, LOW);
    has_external_display = findDisplay(&Wire, PIN_OLED_SDA, PIN_OLED_SCL, OLED_I2C_ADDR);
    printf("Found external display: %s\n", has_external_display ? "yes" : "no");
    if (!has_external_display) {
        // no external display found, use the internal one
        digitalWrite(PIN_OLED_RESET, HIGH);
        delay(100);
    }

    // init the display
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    if (!has_external_display) {
        // reduce brightness of internal display to mitigate burn-in
        display.setBrightness(127);
    }
    screen.enabled = true;

    // restore setting (LoRaWAN keys, etc) from EEPROM, or use a default
    EEPROM.begin(sizeof(nvdata));
    nvdata_load();

    snprintf(screen.loraDevEui, sizeof(screen.loraDevEui),
             "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", nvdata.deveui[0], nvdata.deveui[1], nvdata.deveui[2],
             nvdata.deveui[3], nvdata.deveui[4], nvdata.deveui[5], nvdata.deveui[6], nvdata.deveui[7]);

    // LMIC init
    os_init();
    LMIC_reset();
    LMIC_registerEventCb(onEventCallback, NULL);
    LMIC_startJoining();

    // detect hardware
    snprintf(board_name, sizeof(board_name), "ESP32-%s", BOARD_NAME);
    printf("Detecting BME280 ...\n");
    strcpy(screen.pmsensor_name, "");
    strcpy(screen.rhsensor_name, "");
    char bmeVersion[8];
    bmeFound = findBME280(bmeVersion);
    if (bmeFound) {
        printf("Found BME280, i2c=%s\n", bmeVersion);
        snprintf(screen.rhsensor_name, sizeof(screen.rhsensor_name), "BME280: %s", bmeVersion);
    }

    // OTA init
    uint64_t chipid = ESP.getEfuseMac();
    char ssid[32];
    sprintf(ssid, "%s-%04X%08X", board_name, (uint32_t)(chipid >> 32), (uint32_t)chipid);
    printf("Starting AP with SSID '%s', pass '%s'\n", ssid, nvdata.wifipass); 
    WiFi.softAP(ssid, nvdata.wifipass);
    webServer.on("/",[]() {
       webServer.sendContent("<html><head><meta http-equiv=\"Refresh\" content=\"0; url='/update'\"/></head></html>");
    });
    ElegantOTA.begin(&webServer);
    webServer.begin();
    dnsServer.start(53, "*", WiFi.softAPIP());
}

void loop(void)
{
    unsigned long ms = millis();
    unsigned long second = ms / 1000UL;

    // parse command line
    while (Serial.available()) {
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

    // button press cycles through display states
    if ((ms - button_last_pressed) > 500) {
        if (digitalRead(PIN_BUTTON) == 0) {
            button_last_pressed = ms;
            // cycle display mode
            switch (screen.displaymode) {
            case E_DISPLAYMODE_HWINFO:
                set_display_mode(E_DISPLAYMODE_MEASUREMENTS);
                break;
            case E_DISPLAYMODE_MEASUREMENTS:
                set_display_mode(E_DISPLAYMODE_QRCODE);
                break;
            case E_DISPLAYMODE_QRCODE:
                set_display_mode(E_DISPLAYMODE_OFF);
                break;
            case E_DISPLAYMODE_OFF:
                set_display_mode(E_DISPLAYMODE_HWINFO);
                break;
            }
        }
    }
    if ((ms - button_last_pressed) > TIME_OLED_ENABLED) {
        set_display_mode(E_DISPLAYMODE_OFF);
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
    webServer.handleClient();
    dnsServer.processNextRequest();
}

