#include "stdint.h"


#define WIFI_SSID "DFM-PV"
#define WIFI_IDENTITY "dfpv.com.cn"
#define WIFI_USR "oa-pda-mas"
#define WIFI_PWD "z4aIy^"

// void initialise_wifi(void);

static int32_t t_sysInit, t_staInit, t_staConfig, t_ApCnted;

typedef enum
{
    WIFI_STAT_INIT,
    WIFI_STAT_ERRORINIT,
    WIFI_STAT_ERRORPING,
    WIFI_STAT_APCNTING,
    WIFI_STAT_APCNTED,
    WIFI_STAT_SVRFOUND,
    WIFI_STAT_SVRCNTING,
    WIFI_STAT_SVRCNTED,
    WIFI_STAT_SVRLOST,
    WIFI_STAT_APLOST,
} WIFI_STATS;
