#ifndef __WIFI_H__
#define __WIFI_H__

#include "stdint.h"
#include "stdbool.h"
#include "msg.h"
#define WIFI_SSID "DfTest"
#define WIFI_IDENTITY "oa-pda-mas1"
#define WIFI_USR "oa-pda-mas1"
#define WIFI_PWD "11111111"

static int32_t t_sysInit, t_staInit, t_staConfig, t_ApCnted;

typedef enum
{
    WIFI_STAT_INIT,
    WIFI_STAT_ERRORINIT,
    WIFI_STAT_ERRORPING,
    WIFI_STAT_ERRORCSVR,
    WIFI_STAT_APCNTING,
    WIFI_STAT_APCNTED,
    WIFI_STAT_SVRFOUND,
    WIFI_STAT_SVRCNTING,
    WIFI_STAT_SVRCNTED,
    WIFI_STAT_IDLE,
    WIFI_STAT_SVRLOST,
    WIFI_STAT_APLOST,
    WIFI_STAT_DATAERROR,
    WIFI_STAT_SENDERROR,
    WIFI_STAT_SENDIMCOMPL,
} WIFI_STATS;

int pingServer(void);
void connectServer(void);
void disconnectServer(void);

// extern bool canSendOneFrame(CAN_CMD_FRAME * msg);
void taskDataDown();
void taskDataUp();

#endif // !__WIFI__