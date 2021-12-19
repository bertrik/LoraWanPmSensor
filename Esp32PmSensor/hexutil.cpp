#include <string.h>
#include "printf.h"

#include "hexutil.h"

void hexprint(const char *prefix, const uint8_t * buf, int len)
{
    printf(prefix);
    for (int i = 0; i < len; i++) {
        printf("%02X", buf[i]);
    }
    printf("\n");
}

void hexparse(const char *hex, uint8_t *buf, int len)
{
    char tmp[3];
    for (int i = 0; i < len; i++) {
        strlcpy(tmp, hex, 3);
        *buf++ = strtoul(tmp, NULL, 16);
        hex += 2;
    }
}

