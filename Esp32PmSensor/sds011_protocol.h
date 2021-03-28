#include <stdbool.h>
#include <stdint.h>

#define CMD_SET_DATA_REPORTING_MODE 2
#define CMD_QUERY_DATA              4
#define CMD_SET_DEVICE_ID           5
#define CMD_SET_SLEEP_AND_WORK      6
#define CMD_CHECK_FIRMWARE_VERSION  7
#define CMD_SET_WORKING_PERIOD      8

// parsing state
typedef enum {
    HEAD = 0,
    COMMAND,
    DATA,
    CHECK,
    TAIL
} EState;

typedef struct {
    float pm2_5;
    float pm10;
    uint16_t id;
} sds_meas_t;

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

    bool process(uint8_t b, uint8_t rsp_id);

    /**
     * Parses the received data into a sds_meas_t structure.
     * @param meas the measurement structure
     */
    void getMeasurement(sds_meas_t *measurement);

    /**
     * Creates a command byte array from command data.
     * @param buf the destination buffer
     * @param size the size of the destination buffer
     * @param cmd_data the command data to be copied
     * @param cmd_len the length of the command data
     * @return the actual size of the command byte array created
     */
    int createCommand(uint8_t * buf, int size, const uint8_t * cmd_data, int cmd_data_len);

    /**
     * Retrieves response data from the internal response data buffer.
     * @param rsp the response data buffer
     * @param rsp_size the response data buffer size
     * @return the actual size of the response data buffer
     */
    int getBuffer(uint8_t *rsp, int rsp_size);

};


