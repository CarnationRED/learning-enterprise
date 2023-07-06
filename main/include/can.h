#ifndef __CAN_H__
#define __CAN_H__
#include "msg.h"
#include "fifo.h"


void spitest();
void APP_CANFDSPI_Init(void *p);
bool canSendOneFrame(CAN_CMD_FRAME * msg);

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
extern int rxMsgFifo;
extern u8 canCurrentChannel;
#endif