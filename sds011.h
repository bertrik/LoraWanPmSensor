#include <stdbool.h>
#include <stdint.h>

#define CMD_SET_DATA_REPORTING_MODE 2
#define CMD_QUERY_DATA              4
#define CMD_SET_DEVICE_ID           5
#define CMD_SET_SLEEP_AND_WORK      6
#define CMD_CHECK_FIRMWARE_VERSION  7
#define CMD_SET_WORKING_PERIOD      8

typedef struct {
    float pm2_5;
    float pm10;
    uint16_t id;
} sds_meas_t;

void SdsInit(void);
bool SdsProcess(uint8_t b, uint8_t cmd_id);
void SdsParse(sds_meas_t *meas);
int SdsCreateCmd(uint8_t *buf, int size, const uint8_t *cmd_data, int cmd_data_len);
int SdsGetBuffer(uint8_t *rsp);


