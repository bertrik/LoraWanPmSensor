#include <stdio.h>

#include <OLEDDisplay.h>
#include <qrcode.h>

#include "lwqrcode.h"

static QRCode qrcode;
static uint8_t qrcodeBytes[200];

void qrcode_show(OLEDDisplay *display, const uint8_t *appeui, const uint8_t *deveui, const uint8_t *appkey)
{
    char qrtext[80];
    char *ptext = qrtext;
    ptext += sprintf(qrtext, "LW:");
    for (int i = 0; i < 8; i++) {
        ptext += sprintf(ptext, "%02X", appeui[i]);
    }
    *ptext++ = ':';
    for (int i = 0; i < 8; i++) {
        ptext += sprintf(ptext, "%02X", deveui[i]);
    }
    *ptext++ = ':';
    for (int i = 0; i < 16; i++) {
        ptext += sprintf(ptext, "%02X", appkey[i]);
    }
    qrcode_initText(&qrcode, qrcodeBytes, 3, ECC_LOW, qrtext);

    display->setColor(WHITE);
    display->fillRect(0, 0, 128, 64);

    display->setColor(BLACK);
    for (int y = 0; y < qrcode.size; y++) {
        for (int x = 0; x < qrcode.size; x++) {
            int xx = 35 + x * 2;
            int yy = 3 + y * 2;
            if (qrcode_getModule(&qrcode, x, y)) {
                display->drawRect(xx, yy, 2, 2);
            }
        }
    }
    display->display();
}

