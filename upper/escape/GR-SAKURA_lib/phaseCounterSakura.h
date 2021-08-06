//#include"gr_common/rx63n/iodefine_gcc63n.h"
#include "iodefine_gcc63n.h"

#define ENC_INIT_VAL 0x7FFF

class phaseCounter
{
public:
    phaseCounter(int);
    int getCount();
    void setCount(int ch, int num);
    void init();

private:
    bool ch_available;
    bool init_done;
    
    unsigned short int pre_rawcount;
    int encount;
    
    int g_ch;
};