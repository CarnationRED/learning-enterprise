#include "wifi.h"
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
char *thisIp = "255.255.255.255";
char *thisGw = "255.255.255.255";
char *thisMask = "255.255.255.255";
WIFI_STATS wifiStats = WIFI_STAT_INIT;

// /* FreeRTOS event group to signal when we are connected & ready to make a request */
// EventGroupHandle_t wifi_event_group;

// /* esp netif object representing the WIFI station */
// esp_netif_t *sta_netif = NULL;

// /* The event group allows multiple bits for each event,
//    but we only care about one event - are we connected
//    to the AP with an IP? */
// const int CONNECTED_BIT = BIT0;

// static esp_err_t pingResults(ping_target_id_t msgType, esp_ping_found *pf)
// {
//     printf("AvgTime:%.1fmS Sent:%ld Rec:%ld min(mS):%ld max(mS):%ld Resp(mS):%ld Timeouts:%ld Total Time:%ld\n", (float)pf->total_time / pf->recv_count, pf->send_count, pf->recv_count, pf->min_time, pf->max_time, pf->resp_time, pf->timeout_count, pf->total_time);
//     return ESP_OK;
// }
// static void event_handler(void *arg, esp_event_base_t event_base,
//                           int32_t event_id, void *event_data)
// {
//     if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
//     {
//         wifiStats = WIFI_STAT_APCNTING;
//         esp_wifi_connect();
//     }
//     else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
//     {
//         wifiStats = WIFI_STAT_APLOST;
//         esp_wifi_connect();
//         xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
//     }
//     else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
//     {
//         wifiStats = WIFI_STAT_APCNTED;
//         xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
//         t_ApCnted = xTaskGetTickCount();
//         esp_netif_ip_info_t ip;
//         memset(&ip, 0, sizeof(esp_netif_ip_info_t));
//         if (esp_netif_get_ip_info(sta_netif, &ip) == 0 && (xEventGroupGetBits(wifi_event_group) & CONNECTED_BIT) != 0)
//         {
//             printf("~~~~~~~~~~~");
//             printf("IP:" IPSTR, IP2STR(&ip.ip));
//             sprintf(thisIp, IPSTR, IP2STR(&ip.ip));
//             sprintf(thisMask, IPSTR, IP2STR(&ip.netmask));
//             sprintf(thisGw, IPSTR, IP2STR(&ip.gw));
//             printf("Startup: sys(%ldms) app(%ldms)", t_sysInit * 10, (t_staInit - t_sysInit) * 10);
//             printf("Time: config(%ldms), connect(%ldms), total(%ldms)", pdTICKS_TO_MS(t_staConfig - t_staInit), (t_ApCnted - t_staConfig) * 10, (t_ApCnted - t_staInit) * 10);
//             printf("~~~~~~~~~~~");
//             const char *ip = "10.5.189.99";

//             ping_deinit();
//             uint32_t ping_count = 4;
//             uint32_t ping_timeout = 500;
//             uint32_t ping_delay = 500;
//             esp_ping_set_target(PING_TARGET_IP_ADDRESS_COUNT, &ping_count, sizeof(uint32_t));
//             esp_ping_set_target(PING_TARGET_RCV_TIMEO, &ping_timeout, sizeof(uint32_t));
//             esp_ping_set_target(PING_TARGET_DELAY_TIME, &ping_delay, sizeof(uint32_t));
//             esp_ping_set_target(PING_TARGET_IP_ADDRESS, ip, sizeof(uint32_t));
//             esp_ping_set_target(PING_TARGET_RES_FN, &pingResults, sizeof(pingResults));
//             uint8_t res = 0;
//             res = ping_init();
//             if (res == 0)
//             {
//                 printf("PING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
//             }
//             else
//             {
//                 printf("error:%d\n", res);
//             }
//         }
//     }
// }
// void initialise_wifi(void)
// {
//     t_staInit = xTaskGetTickCount();
//     ESP_ERROR_CHECK(esp_netif_init());
//     wifi_event_group = xEventGroupCreate();
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     sta_netif = esp_netif_create_default_wifi_sta();
//     assert(sta_netif);

//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));
//     ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
//     ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
//     ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
//     wifi_config_t wifi_config = {
//         .sta = {.ssid = WIFI_SSID},
//     };
//     ESP_LOGI("WIFI", "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
//     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
//     ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)WIFI_IDENTITY, strlen(WIFI_IDENTITY)));

//     ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_username((uint8_t *)WIFI_USR, strlen(WIFI_USR)));
//     ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_password((uint8_t *)WIFI_PWD, strlen(WIFI_PWD)));

//     ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_enable());
//     ESP_ERROR_CHECK(esp_wifi_start());
//     t_staConfig = xTaskGetTickCount();
//     return;while(1) vTaskDelay(2000);
// }