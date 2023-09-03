#ifndef __TASK__
#define __TASK__

#include "wifi.h"
#include "stdint.h"
#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include "esp_ping.h"
#define ESP_PING
#include "ping/ping.h"

#define CHECKTIMEOUT(time)           \
    vTaskDelay(pdMS_TO_TICKS(time)); \
    if (temp != wifiStats)           \
        break;

extern WIFI_STATS wifiStats;
char *statTimeoutErr = "WIFI stats timeout";
WIFI_STATS currStat;
int statTimeout;

void checkStats()
{
    while (1)
    {
        if (statTimeout > 0)
        {
            vTaskDelay(pdMS_TO_TICKS(statTimeout));
            statTimeout = 0;
            if (wifiStats == currStat)
            {
                ;
                printf("%s:%d", statTimeoutErr, currStat);
            }
        }
        else
            vTaskDelay(pdMS_TO_TICKS(20));
    }
}
// void task_mechine()
// {
//     while (1)
//     {
//         WIFI_STATS temp = wifiStats;
//         switch (wifiStats)
//         {
//         case WIFI_STAT_INIT:
//             break;
//         case WIFI_STAT_ERRORINIT:
//             break;
//         case WIFI_STAT_ERRORPING:
//             break;
//         case WIFI_STAT_ERRORCSVR:
//             break;
//         case WIFI_STAT_APCNTING:
//             break;
//         case WIFI_STAT_APCNTED:
//             pingServer();
//         case WIFI_STAT_SVRFOUND:
//             wifiStats = WIFI_STAT_SVRCNTING;
//             connectServer();
//             break;
//         case WIFI_STAT_SVRCNTING:
//             break;
//         case WIFI_STAT_SVRCNTED:
//             wifiStats = WIFI_STAT_IDLE;
//             xTaskCreate(taskDataDown,"taskDataDown",4096, NULL, 3, NULL);
//             break;
//         case WIFI_STAT_IDLE:
//             break;
//         case WIFI_STAT_SVRLOST:
//             break;
//         case WIFI_STAT_APLOST:
//             break;

//         default:
//             break;
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }
#endif // !__TASK__
