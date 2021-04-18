#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SHDLC_START,
    SHDLC_ADR,
    SHDLC_CMD,
    SHDLC_STATE,
    SHDLC_L,
    SHDLC_DATA,
    SHDLC_CHK,
    SHDLC_STOP
} state_t;



class SHDLC {

private:
    state_t _state;
    uint8_t _cmd;
    int _index;
    size_t _length;
    uint8_t _data[256];
    bool _escape;
    uint8_t _check;
    uint8_t _miso_state;

private:

    bool unescape(uint8_t *pc);

public:
    SHDLC();

    size_t build_tx(uint8_t *buf, uint8_t cmd, size_t data_len, const uint8_t *data);
    bool process_rx(uint8_t c, uint8_t cmd);
    
    size_t get_data(uint8_t *data);
    uint8_t get_state(void);

};

