/* WiFi Connection Example using WPA2 Enterprise
 *
 * Original Copyright (C) 2006-2016, ARM Limited, All Rights Reserved, Apache 2.0 License.
 * Additions Copyright (C) Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD, Apache 2.0 License.
 *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_ping.h"
#define ESP_PING
#include "ping/ping.h"
#include "u8g2.h"
#include "oled.h"
#include "u8g2_wqy.h"
#include "msg.h"
#include "can.h"
#include "mqtt.h"
#include "task.h"

/* The examples use simple WiFi configuration that you can set via
   project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"

   You can choose EAP method via project configuration according to the
   configuration of AP.
*/

typedef int32_t s32;
typedef int16_t s16;

TaskHandle_t hdle0;
TaskHandle_t hdle1;
TaskHandle_t hdle2;
TaskHandle_t hdle3;
static const char *TAG = "example";

extern char *thisIp;
extern char *thisGw;
extern char *thisMask;
extern WIFI_STATS wifiStats;
extern int subscribed;
extern void (*mqtt_start)(void);
extern void (*canSend2Wifi)(char *data, int len);

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* esp netif object representing the WIFI station */
static esp_netif_t *sta_netif = NULL;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

// static esp_err_t pingResults(ping_target_id_t msgType, esp_ping_found *pf)
// {
//     // printf("AvgTime:%.1fmS Sent:%ld Rec:%ld min(mS):%ld max(mS):%ld Resp(mS):%ld Timeouts:%ld Total Time:%ld\n", (float)pf->total_time / pf->recv_count, pf->send_count, pf->recv_count, pf->min_time, pf->max_time, pf->resp_time, pf->timeout_count, pf->total_time);
//     wifiStats = WIFI_STAT_SVRFOUND;
//     ping_deinit();
//     return ESP_OK;
// }

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        wifiStats = WIFI_STAT_APCNTING;
        ESP_LOGI("WIFI", "Connecting");
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifiStats = WIFI_STAT_APLOST;
        ESP_LOGI("WIFI", "Re-Connecting");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        t_ApCnted = xTaskGetTickCount();
        esp_netif_ip_info_t ip;
        memset(&ip, 0, sizeof(esp_netif_ip_info_t));
        if (esp_netif_get_ip_info(sta_netif, &ip) == 0 && (xEventGroupGetBits(wifi_event_group) & CONNECTED_BIT) != 0)
        {
            wifiStats = WIFI_STAT_APCNTED;
            sprintf(thisIp, IPSTR, IP2STR(&ip.ip));
            sprintf(thisMask, IPSTR, IP2STR(&ip.netmask));
            sprintf(thisGw, IPSTR, IP2STR(&ip.gw));

            mqtt_start();
            /* printf("~~~~~~~~~~~");
             printf("IP:%s\n", thisIp);
             // printf("Startup: sys(%ldms) app(%ldms)", t_sysInit * 10, (t_staInit - t_sysInit) * 10);
             printf("Time: config(%ldms), connect(%ldms), total(%ldms)\n", pdTICKS_TO_MS(t_staConfig - t_staInit), (t_ApCnted - t_staConfig) * 10, (t_ApCnted - t_staInit) * 10);
             printf("~~~~~~~~~~~");
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
                 ping_timeout += ping_delay;
                 ping_timeout *= ping_count;
                 xTaskCreate(&checkPing, "checkPing", 1024, &ping_timeout, 6, NULL);
             }
             else
             {
                 printf("error:%d\n", res);
                 wifiStats = WIFI_STAT_ERRORPING;
             }*/
        }
    }
}
void initialise_wifi(void)
{
    t_staInit = xTaskGetTickCount();
    wifiStats = WIFI_STAT_INIT;
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {.ssid = WIFI_SSID},
    };
    ESP_LOGI("WIFI", "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)WIFI_IDENTITY, strlen(WIFI_IDENTITY)));

    ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_username((uint8_t *)WIFI_USR, strlen(WIFI_USR)));
    ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_password((uint8_t *)WIFI_PWD, strlen(WIFI_PWD)));

    ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_enable());
    ESP_ERROR_CHECK(esp_wifi_start());
    t_staConfig = xTaskGetTickCount();
    vTaskDelay(2000);
    vTaskDelete(NULL);
    vTaskDelay(2000);
}
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
void initialise_wifi_noneEnt(void)
{
    t_staInit = xTaskGetTickCount();
    wifiStats = WIFI_STAT_INIT;
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PWD,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = 4,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PWD);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PWD);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    t_staConfig = xTaskGetTickCount();
    vTaskDelay(2000);
    vTaskDelete(NULL);
    vTaskDelay(2000);
}

extern void spitest();
extern void task_mechine();
void led();
void app_main(void)
{
    t_sysInit = xTaskGetTickCount();
    ESP_ERROR_CHECK(nvs_flash_init());
    // initialise_wifi();
    //  xTaskCreate(&wpa2_enterprise_example_task, "wpa2_enterprise_example_task", 4096, NULL, 5, &hdle1);
    //  xTaskCreate(&APP_CANFDSPI_Init, "APP_CANFDSPI_Init", 4096, NULL, 5, &hdle);
    xTaskCreate(&initialise_wifi_noneEnt, "init_wifi", 4096, NULL, 4, &hdle0);
    // xTaskCreate(&task_mechine, "wifi_task", 4096, NULL, 3, &hdle1);
    // xTaskCreate(&spitest, "spitest", 4096, NULL, 5, &hdle2);
    // xTaskCreate(&led, "led", 4096, NULL, 6, &hdle3);
    canSend2Wifi = mqtt_dataUp_pulish;
    // ESP_LOGI(TAG, "sizeof ReportMsg=%d", sizeof(ReportMsg));
    // ESP_LOGI(TAG, "sizeof ReportMsg=%d", sizeof(ReportMsg));
    // ESP_LOGI(TAG, "sizeof ReportMsg=%d", sizeof(ReportMsg));
    can_init();
    msg_init();
//    volatile int s1 = sizeof(CAN_TX_MSGOBJ);
//    volatile int s2 = sizeof(CAN_MSGOBJ_ID);
//    volatile int s3 = sizeof(CAN_TX_MSGOBJ_CTRL);
//    volatile int s4 = sizeof(CAN_MSG_TIMESTAMP);
//    volatile CAN_CMD_FRAME ss;
//    ss.txObj.word[0]=  ss.txObj.word[1]=  ss.txObj.word[2]=0;
//    ss.txObj.bF.id.EID=0x112;
//    ss.txObj.bF.id.SID=0x112;
//    ss.txObj.bF.id.SID=0x113;
}
void led()
{
    volatile u8g2_t u;
    volatile u8g2_uint_t aaa;
    u8g2Init(&u);
    u8g2_SetFont(&u, u8g2_font_wqy12_t_gb2312a);
    aaa = u8g2_DrawUTF8(&u, 65, 40, "你好world"); // 打开显示器
    u8g2_SendBuffer(&u);
    while (t_sysInit < 8000000)
        t_sysInit++;

    uint16_t num = 0;
    s32 x = 0;
    float y = 0;
    s32 lx = 0, ly = 0;
    s32 dx = 1;
    float dy = 1;
    s32 w = 11 * 6, h = 12;
    s32 wm = 128 - w, hm = 64 - h;
    char text[6];
    text[5] = 0;
    u8g2_SetFontPosTop(&u);
    u8g2_SetFontMode(&u, 1);
    while (1)
    {
        lx = x;
        ly = y;
        x += dx;
        y += dy;
        dy += 0.2;
        num = xTaskGetTickCount();
        text[0] = 48 + num / 10000;
        text[1] = 48 + (num % 10000) / 1000;
        text[2] = 48 + (num % 1000) / 100;
        text[3] = 48 + (num % 100) / 10;
        text[4] = 48 + (num % 10);
        u8g2_SetDrawColor(&u, 0);
        u8g2_DrawBox(&u, lx > x ? x : lx, ly > y ? y : ly, w + (dx > 0 ? dx : -dx) + 3, h + (dy > 0 ? dy : -dy));
        // u8g2_ClearDisplay(&u);
        u8g2_SetDrawColor(&u, 1);
        u8g2_DrawUTF8(&u, x, y, (char *)thisIp);
        u8g2_SendBuffer(&u);
        if (x >= wm)
            dx = -1 * ((uint8_t)rand()) % 4;
        if (x < 0)
            dx = ((uint8_t)rand()) % 4;
        if (y >= hm)
            dy = -1 * ((uint8_t)rand()) % 6;
        if (y < 0)
            dy = ((uint8_t)rand()) % 4;
        if (dx == 0)
            dx = 1;
        if (dy == 0)
            dy = 1;
        vTaskDelay(29);
    }
}
