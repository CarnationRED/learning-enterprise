#ifndef __WIFI_C__
#define __WIFI_C__
#include "can.h"
#include "wifi.h"
#include "fifo.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_event.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_ping.h"
#include "esp_log.h"
#define ESP_PING
#define TAG "wifi.c"
#include "ping/ping.h"
#include <unistd.h>
#include <sys/socket.h>
#include <errno.h>
#include <netdb.h> // struct addrinfo
#include <arpa/inet.h>
char *thisIp = "255.255.255.255";
char *thisGw = "255.255.255.255";
char *thisMask = "255.255.255.255";

int sockDataUp = -1, sockDataDown = -1, sockCtrl = -1, sockReport = -1;
int prtDataDown = 2222, prtDataUp = 2223, prtCtrl = 2224, prtReport = 2225;
TaskHandle_t handleUp = NULL, handleDown = NULL, handleCtrl = NULL, handleReport = NULL;

WIFI_STATS wifiStats = WIFI_STAT_INIT;

static esp_err_t pingResults(ping_target_id_t msgType, esp_ping_found *pf)
{
    // printf("AvgTime:%.1fmS Sent:%ld Rec:%ld min(mS):%ld max(mS):%ld Resp(mS):%ld Timeouts:%ld Total Time:%ld\n", (float)pf->total_time / pf->recv_count, pf->send_count, pf->recv_count, pf->min_time, pf->max_time, pf->resp_time, pf->timeout_count, pf->total_time);
    if (wifiStats == WIFI_STAT_APCNTED)
        wifiStats = WIFI_STAT_SVRFOUND;
    ping_deinit();
    return ESP_OK;
}
int pingServer()
{
    const char *ip = "10.5.189.99";
    ping_deinit();
    uint32_t ping_count = 4;
    uint32_t ping_timeout = 500;
    uint32_t ping_delay = 500;
    esp_ping_set_target(PING_TARGET_IP_ADDRESS_COUNT, &ping_count, sizeof(uint32_t));
    esp_ping_set_target(PING_TARGET_RCV_TIMEO, &ping_timeout, sizeof(uint32_t));
    esp_ping_set_target(PING_TARGET_DELAY_TIME, &ping_delay, sizeof(uint32_t));
    esp_ping_set_target(PING_TARGET_IP_ADDRESS, ip, sizeof(uint32_t));
    esp_ping_set_target(PING_TARGET_RES_FN, &pingResults, sizeof(pingResults));
    uint8_t res = 0;
    res = ping_init();
    if (res == 0)
    {
        printf("PING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        ping_count = 2000;
        while (ping_count > 0)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            if (wifiStats != WIFI_STAT_APCNTED)
                return true;
            vTaskDelay(pdMS_TO_TICKS(150));
            ping_count -= 200;
        }
        printf("PING server timeout:%s\n", ip);
        wifiStats = WIFI_STAT_ERRORPING;
    }
    else
    {
        printf("error:%d\n", res);
        wifiStats = WIFI_STAT_ERRORPING;
    }
    return false;
}

int connectPrt(int prt)
{
    char rx_buffer[128];
    char host_ip[] = "10.5.113.172";
    int addr_family = 0;
    int ip_protocol = 0;
    int sock = 0;

    while (1)
    {
        struct sockaddr_in dest_addr;
        inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(prt);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, prt);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0)
        {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Port%d connected", prt);
        break;
    }
    return sock;
}
bool connectAllPrt()
{
    sockDataDown = connectPrt(prtDataDown);
    sockDataUp = connectPrt(prtDataUp);
    sockCtrl = connectPrt(prtCtrl);
    sockReport = connectPrt(prtReport);
    return sockDataDown >= 0 &&
           sockDataUp >= 0 &&
           sockCtrl >= 0 &&
           sockReport >= 0;
}
inline void cleanPrt(int sock)
{
    if (sock != -1)
    {
        shutdown(sock, 0);
        close(sock);
    }
}
void cleanAllPrt()
{
    ESP_LOGE(TAG, "Shutting down socket and restarting...");
    cleanPrt(sockDataDown);
    cleanPrt(sockDataUp);
    cleanPrt(sockCtrl);
    cleanPrt(sockReport);
}
// void connectServer()
// {
//     for (uint16_t i = 0; i < 3; i++)
//     {
//         if (connectAllPrt())
//         {
//             wifiStats = WIFI_STAT_SVRCNTED;
//             xTaskCreate(taskDataUp,"taskDataUp",2048, NULL,3,&handleUp);
//             xTaskCreate(taskDataDown,"taskDataDown",2048, NULL,3,&handleDown);
//             return;
//         }
//         else
//         {
//             cleanAllPrt();
//             vTaskDelete(handleUp);
//             vTaskDelete(handleDown);
//         }
//     }
//     wifiStats = WIFI_STAT_ERRORCSVR;
// }

// void disconnectServer()
// {
//     cleanAllPrt();
//     wifiStats = WIFI_STAT_INIT;
// }

/// @brief can't reference canSendOneFrame here, so a pointer canQueuePtr is defined to solve the bug
/// @param msg
/// @return
// bool (*canQueuePtr)(CAN_CMD_FRAME *msg) = NULL;
// bool (*canSetFilterPtr)(CAN_FILTER_CFG *flt) = NULL;
// void taskDataDown()
// {
//     char rxb[1600];
//     while (1)
//     {
//         if (sockDataDown > 0)
//         {
//             int len = recv(sockDataDown, rxb, sizeof(rxb) - 1, 0);
//             if (len >= sizeof(DataDownMsg))
//             {
//                 char *ptr = rxb;
//                 DataDownMsg *head = (DataDownMsg *)ptr;
//                 if (head != NULL)
//                 {
//                     switch (head->type)
//                     {
//                     case SINGLE_FRAME:
//                         if (head->dataLen == head->elementSize * head->elementCount)
//                         {
//                             if (canQueuePtr != NULL)
//                                 canQueuePtr((CAN_CMD_FRAME *)(ptr + sizeof(DataDownMsg)));
//                             // canSendOneFrame((CAN_CMD_FRAME *)ptr);
//                         }
//                         else
//                             wifiStats = WIFI_STAT_DATAERROR;
//                         break;

//                     default:
//                         wifiStats = WIFI_STAT_DATAERROR;
//                         break;
//                     }
//                 }
//             }
//         }
//     }
// }
// extern int rxMsgFifo;
// extern int fifo_readable_item_count(int i);
// void taskDataUp()
// {
//     char txb[1024];
//     // cited:https://blog.csdn.net/weixin_45499326/article/details/129155557
//     uint32_t ulEventsToProcess;
//     int maxCountPerPacket = sizeof(txb) / sizeof(CAN_MSG_FRAME);
//     for (;;)
//     {
//         /* 此处阻塞以等待中断ISR直接发送过来的通知，并读取返回值。
//         这里阻塞超时时间比产生两个中断之间的周期要大于10毫秒，因此不会出现中断到达，而阻塞已到期，返回0的情况*/
//         ulEventsToProcess = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//         if (ulEventsToProcess != 0)
//         {
//             // /* 要到达此处，必须至少发生了一个事件。在这里循环，直到处理完所有未决事件（在这种情况下，只需为每个事件打印一条消息）。 */
//             // while( ulEventsToProcess > 0 )
//             // {
//             // 	vPrintString( " Processing event.\r\n" );
//             // 	ulEventsToProcess--;
//             // }
//             for (int count = fifo_readable_item_count(rxMsgFifo); count > 0;)
//             {
//                 if (count > maxCountPerPacket)
//                     count = maxCountPerPacket;
//                 fifo_read_batch(rxMsgFifo, txb, count);
//                 int ret = send(sockDataUp, txb, count * sizeof(CAN_MSG_FRAME), 0);
//                 if (ret < 0)
//                 {
//                     wifiStats = WIFI_STAT_SENDERROR;
//                 }
//                 else if (ret != sizeof(txb))
//                 {
//                     wifiStats = WIFI_STAT_SENDIMCOMPL;
//                 }
//             }
//         }
//         else
//         {
//             /* 如果达到了功能的这一部分，则中断未在预期时间内到达，并且（在实际应用程序中）可能需要执行一些错误恢复操作。 */
//         }
//     }
// }


#endif // !__WIFI_C__