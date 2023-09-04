#ifndef __CAN_H__
#define __CAN_H__
#include "msg.h"
#include "fifo.h"
#include "cQueue.h"


void spitest();
void can_init();
void APP_CANFDSPI_Init(void *p);
static bool canEnqueueOneFrame(CAN_CMD_FRAME * msg);
static bool canSendOneFrame(CAN_CMD_FRAME * msg);
static bool canSetFilter(CAN_FILTER_CFG *flt);

typedef enum
{
    CAN_STAT_INIT,
    CAN_STAT_ERRORINIT,
    CAN_STAT_IDLE,
    CAN_STAT_TXQFULL,
    CAN_STAT_TXERROR,
    CAN_STAT_RXQFULL,
    CAN_STAT_RXERROR,
    CAN_STAT_RXQERROR,
    CAN_STAT_DATAERROR,
} CAN_STATS;

extern Queue_t rxMsgFifo;
extern u8 canCurrentChannel;
#endif