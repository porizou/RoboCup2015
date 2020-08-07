#ifndef _XBEE_API
#define _XBEE_API


/*--- データフレーム用クラス ---*/
class xbeeAPI {
public:
    uint8_t address[8];
    uint8_t direction;
    uint8_t turning;
    void reset() {
        for (int i=0; i<8; i++)  address[i]=0;
        direction = 0;
        turning   = 0;
    }
};

#endif
