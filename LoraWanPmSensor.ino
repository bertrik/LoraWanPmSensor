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

#include "sds011.h"

#define LEDPIN 2

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

#define PIN_RX  34
#define PIN_TX  25

unsigned int counter = 0;

static SSD1306 display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL);
static SoftwareSerial sds011(PIN_RX, PIN_TX);

/*************************************
 * TODO: Change the following keys
 * NwkSKey: network session key, AppSKey: application session key, and DevAddr: end-device address
 *************************************/
static u1_t NWKSKEY[16] =
    { 0xF6, 0x92, 0x38, 0x53, 0xB0, 0x8F, 0x43, 0x79, 0xE5, 0x30, 0x46,
    0x33, 0xDB, 0x76, 0x1D, 0x2E
};

static u1_t APPSKEY[16] =
    { 0xC8, 0x53, 0x59, 0xB8, 0x19, 0x69, 0x4B, 0x8C, 0x2B, 0xC0, 0x79,
    0x88, 0x8E, 0xD5, 0x4C, 0x15
};

static u4_t DEVADDR = 0x260119D3;       // Put here the device id in hexadecimal form.

void os_getArtEui(u1_t * buf)
{
}

void os_getDevEui(u1_t * buf)
{
}

void os_getDevKey(u1_t * buf)
{
}

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;
char TTN_response[30];

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = { 26, 33, 32 }       // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void onEvent(ev_t ev)
{
    if (ev == EV_TXCOMPLETE) {
        display.clear();
        display.drawString(0, 0, "EV_TXCOMPLETE event!");

        Serial.println(F
                       ("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK) {
            Serial.println(F("Received ack"));
            display.drawString(0, 20, "Received ACK.");
        }

        if (LMIC.dataLen) {
            int i = 0;
            // data received in rx slot after tx
            Serial.print(F("Data Received: "));
            Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
            Serial.println();

            display.drawString(0, 20, "Received DATA.");
            for (i = 0; i < LMIC.dataLen; i++)
                TTN_response[i] = LMIC.frame[LMIC.dataBeg + i];
            TTN_response[i] = 0;
            display.drawString(0, 32, String(TTN_response));
        }
        digitalWrite(LEDPIN, LOW);
        display.drawString(0, 50, String(counter));
        display.display();
    }
}

int getChipRevision()
{
    return (REG_READ(EFUSE_BLK0_RDATA3_REG) >> (EFUSE_RD_CHIP_VER_REV1_S)
            && EFUSE_RD_CHIP_VER_REV1_V);
}

void printESPRevision()
{
    Serial.print("REG_READ(EFUSE_BLK0_RDATA3_REG): ");
    Serial.println(REG_READ(EFUSE_BLK0_RDATA3_REG), BIN);

    Serial.print("EFUSE_RD_CHIP_VER_REV1_S: ");
    Serial.println(EFUSE_RD_CHIP_VER_REV1_S, BIN);

    Serial.print("EFUSE_RD_CHIP_VER_REV1_V: ");
    Serial.println(EFUSE_RD_CHIP_VER_REV1_V, BIN);

    Serial.println();

    Serial.print("Chip Revision (official version): ");
    Serial.println(getChipRevision());

    Serial.print("Chip Revision from shift Operation ");
    Serial.println(REG_READ(EFUSE_BLK0_RDATA3_REG) >> 15, BIN);

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

static int sds_exchange(uint8_t * cmd, int cmd_len, uint8_t * rsp,
                        int timeout)
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

    printESPRevision();

    // Use the Blue pin to signal transmission.
    pinMode(LEDPIN, OUTPUT);

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

    // initialize the SDS011
    sds011.begin(9600);
    SdsInit();

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);        // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);  // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Set static session parameters.
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    //LMIC_setDrTxpow(DR_SF11,14);
    LMIC_setDrTxpow(DR_SF9, 14);

    sds_version();
    sds_version();
}

static void send_dust(sds_meas_t * meas)
{
    uint8_t buf[20];

    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // encode it
        int idx = 0;
        buf[idx++] = 0;
        buf[idx++] = 1;

        buf[idx++] = 0xFF;
        buf[idx++] = 0xFF;
        buf[idx++] = 0xFF;
        buf[idx++] = 0xFF;
        buf[idx++] = 0xFF;
        buf[idx++] = 0xFF;

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
        digitalWrite(LEDPIN, HIGH);
        display.clear();
        display.drawString(0, 0, "Sending uplink packet...");
        display.drawString(0, 50, String(++counter));
        display.display();
    }
}

void loop()
{
    static sds_meas_t sds_meas;
    static unsigned long last_sent = 0;
    static bool have_data = false;

    // check for incoming measurement data
    while (sds011.available()) {
        Serial.print(".");
        uint8_t c = sds011.read();
        if (SdsProcess(c, 0xC0)) {
            // parse it
            SdsParse(&sds_meas);
            Serial.println("Got data");
            have_data = true;
        }
    }

    // time to send a dust measurement?
    unsigned long now = millis() / 1000;
    if ((now - last_sent) > 10) {
        last_sent = now;
        if (have_data) {
            Serial.println("Sending dust");
            send_dust(&sds_meas);
            have_data = false;
        }
    }
    // ?
    os_runloop_once();
}
