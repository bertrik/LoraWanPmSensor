#ifndef SDS011_PROTOCOL_H
#define SDS011_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

// parsing state
typedef enum {
    HEAD = 0,
    COMMAND,
    DATA,
    CHECK,
    TAIL
} EState;

class SDS011Protocol {

private:
    EState  _state;
    uint8_t _buf[32];
    int     _size;
    int     _idx;
    int     _len;
    uint8_t _sum;

public:
    SDS011Protocol();

    bool process_rx(uint8_t b, uint8_t rsp_id);

    /**
     * Creates a command byte array from command data.
     * @param buf the destination buffer
     * @param cmd_len the length of the command data
     * @param cmd_data the command data to be copied
     * @return the actual size of the command byte array created
     */
    int build_tx(uint8_t *buf, uint8_t cmd, size_t cmd_len, const uint8_t *cmd_data);

    /**
     * Copies response data.
     * @param rsp the response data buffer, should be at least 6 bytes
     * @return the actual size of the response data buffer
     */
    size_t get_data(uint8_t *rsp);

};

#endif /* SDS011_PROTOCOL_H */

