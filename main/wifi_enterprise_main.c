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

/* The examples use simple WiFi configuration that you can set via
   project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"

   You can choose EAP method via project configuration according to the
   configuration of AP.
*/

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* esp netif object representing the WIFI station */
static esp_netif_t *sta_netif = NULL;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

static const char *TAG = "example";

static int32_t ts, t0, t1, t2;

static esp_err_t pingResults(ping_target_id_t msgType, esp_ping_found *pf)
{
    printf("AvgTime:%.1fmS Sent:%ld Rec:%ld min(mS):%ld max(mS):%ld Resp(mS):%ld Timeouts:%ld Total Time:%ld\n", (float)pf->total_time / pf->recv_count, pf->send_count, pf->recv_count, pf->min_time, pf->max_time, pf->resp_time, pf->timeout_count, pf->total_time);
    return ESP_OK;
}
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        t2 = xTaskGetTickCount();  esp_netif_ip_info_t ip;
    memset(&ip, 0, sizeof(esp_netif_ip_info_t));
        if (esp_netif_get_ip_info(sta_netif, &ip) == 0 && (xEventGroupGetBits(wifi_event_group) & CONNECTED_BIT) != 0)
        {
            printf("~~~~~~~~~~~");
            printf("IP:" IPSTR, IP2STR(&ip.ip));
            printf("MASK:" IPSTR, IP2STR(&ip.netmask));
            printf("GW:" IPSTR, IP2STR(&ip.gw));
            printf("Startup: sys(%ldms) app(%ldms)", ts * 10, (t0 - ts) * 10);
            printf("Time: config(%ldms), connect(%ldms), total(%ldms)",  pdTICKS_TO_MS(t1 - t0), (t2 - t1) * 10, (t2 - t0) * 10);
            printf("~~~~~~~~~~~");
            const char *ip = "10.5.189.99";

            ping_deinit();
            uint32_t ping_count =4;
            uint32_t ping_timeout =1000;
            uint32_t ping_delay =1000;
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
            }
            else
            {
                printf("error:%d\n", res);
            }
        }
    }
}
#define WIFI_SSID "DFM-PV"
#define WIFI_IDENTITY "dfpv.com.cn"
#define WIFI_USR "oa-pda-mas"
#define WIFI_PWD "z4aIy^"
static void initialise_wifi(void)
{
    t0 = xTaskGetTickCount();
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
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)WIFI_IDENTITY, strlen(WIFI_IDENTITY)));

    ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_username((uint8_t *)WIFI_USR, strlen(WIFI_USR)));
    ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_password((uint8_t *)WIFI_PWD, strlen(WIFI_PWD)));

    ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_enable());
    ESP_ERROR_CHECK(esp_wifi_start());
    t1 = xTaskGetTickCount();
}

static TaskHandle_t hdle;
static void wpa2_enterprise_example_task(void *pvParameters)
{
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    while (1)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);

    esp_netif_ip_info_t ip;
    memset(&ip, 0, sizeof(esp_netif_ip_info_t));
        if (esp_netif_get_ip_info(sta_netif, &ip) == 0 && (xEventGroupGetBits(wifi_event_group) & CONNECTED_BIT) != 0)
        {
            ESP_LOGI(TAG, "~~~~~~~~~~~");
            ESP_LOGI(TAG, "IP:" IPSTR, IP2STR(&ip.ip));
            ESP_LOGI(TAG, "MASK:" IPSTR, IP2STR(&ip.netmask));
            ESP_LOGI(TAG, "GW:" IPSTR, IP2STR(&ip.gw));
            ESP_LOGI(TAG, "Startup: sys(%ldms) app(%ldms)", ts * 10, (t0 - ts) * 10);
            ESP_LOGI(TAG, "Time: config(%ldms), connect(%ldms), total(%ldms)", (t1 - t0) * 10, (t2 - t1) * 10, (t2 - t0) * 10);
            ESP_LOGI(TAG, "~~~~~~~~~~~");
            const char *ip = "10.5.189.99";

            ping_deinit();
            uint32_t ping_count =4;
            uint32_t ping_timeout =1000;
            uint32_t ping_delay =1000;
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
            }
            else
            {
                printf("error:%d\n", res);
            }
            vTaskDelete(hdle);
        }
    }
}

void app_main(void)
{
    ts = xTaskGetTickCount();
    ESP_ERROR_CHECK(nvs_flash_init());
    initialise_wifi();
    // xTaskCreate(&wpa2_enterprise_example_task, "wpa2_enterprise_example_task", 4096, NULL, 5, &hdle);
}
