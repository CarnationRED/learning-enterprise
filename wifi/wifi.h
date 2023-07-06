#ifndef __WIFI_H__
#define __WIFI_H__

#include "stdint.h"
#include "stdbool.h"
#include "msg.h"
#include "can.h"
#define WIFI_SSID "DFM-PV"
#define WIFI_IDENTITY "dfpv.com.cn"
#define WIFI_USR "oa-pda-mas"
#define WIFI_PWD "z4aIy^"

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

// extern bool canSendOneFrame(CAN_CMD_FRAME * msg);
void taskDataDown();
void taskDataUp();

#endif // !__WIFI__