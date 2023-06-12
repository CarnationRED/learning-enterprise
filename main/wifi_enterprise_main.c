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
#include "drv_spi.h"
#include "drv_canfdspi_api.h"
#include "i2c_oled_example_main.c"

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
        t2 = xTaskGetTickCount();
        esp_netif_ip_info_t ip;
        memset(&ip, 0, sizeof(esp_netif_ip_info_t));
        if (esp_netif_get_ip_info(sta_netif, &ip) == 0 && (xEventGroupGetBits(wifi_event_group) & CONNECTED_BIT) != 0)
        {
            printf("~~~~~~~~~~~");
            printf("IP:" IPSTR, IP2STR(&ip.ip));
            printf("MASK:" IPSTR, IP2STR(&ip.netmask));
            printf("GW:" IPSTR, IP2STR(&ip.gw));
            printf("Startup: sys(%ldms) app(%ldms)", ts * 10, (t0 - ts) * 10);
            printf("Time: config(%ldms), connect(%ldms), total(%ldms)", pdTICKS_TO_MS(t1 - t0), (t2 - t1) * 10, (t2 - t0) * 10);
            printf("~~~~~~~~~~~");
            const char *ip = "10.5.189.99";

            ping_deinit();
            uint32_t ping_count = 4;
            uint32_t ping_timeout = 1000;
            uint32_t ping_delay = 1000;
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

static TaskHandle_t hdle1;
static TaskHandle_t hdle2;
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
            uint32_t ping_count = 4;
            uint32_t ping_timeout = 1000;
            uint32_t ping_delay = 1000;
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
            vTaskDelete(hdle1);
        }
    }
}
static void spitest();
static void APP_CANFDSPI_Init(void *p);
void app_main(void)
{
    ts = xTaskGetTickCount();
    ESP_ERROR_CHECK(nvs_flash_init());
    initialise_wifi();
    // xTaskCreate(&wpa2_enterprise_example_task, "wpa2_enterprise_example_task", 4096, NULL, 5, &hdle1);
    // xTaskCreate(&APP_CANFDSPI_Init, "APP_CANFDSPI_Init", 4096, NULL, 5, &hdle);
    xTaskCreate(&spitest, "spitest", 4096, NULL, 5, &hdle2);
    oled_main();
}

// Message IDs
#define TX_REQUEST_ID 0x300
#define TX_RESPONSE_ID 0x301
#define BUTTON_STATUS_ID 0x201
#define LED_STATUS_ID 0x200
#define PAYLOAD_ID 0x101

// Transmit Channels
#define APP_TX_FIFO CAN_FIFO_CH2

// Receive Channels
#define APP_RX_FIFO CAN_FIFO_CH1
static CAN_CONFIG config;
static CAN_OPERATION_MODE opMode;

// Transmit objects
static CAN_TX_FIFO_CONFIG txConfig;
static CAN_TX_FIFO_EVENT txFlags;
static CAN_TX_MSGOBJ txObj = {};
static uint8_t txd[MAX_DATA_BYTES];
uint8_t rxd[MAX_DATA_BYTES];

// Receive objects
static CAN_RX_FIFO_CONFIG rxConfig;
static REG_CiFLTOBJ fObj;
static REG_CiMASK mObj;
static CAN_RX_FIFO_EVENT rxFlags;
static CAN_RX_MSGOBJ rxObj;
static bool APP_TestRamAccess(void)
{
    // Variables
    uint16_t i = 0;
    uint8_t length;
    bool good = false;

    Nop();

    // Verify read/write with different access length
    // Note: RAM can only be accessed in multiples of 4 bytes
    for (length = 4; length <= MAX_DATA_BYTES; length += 4)
    {
        for (i = 0; i < length; i++)
        {
            txd[i] = i;
            rxd[i] = 0xff;
        }

        Nop();

        // Write data to RAM
        DRV_CANFDSPI_WriteByteArray(DRV_CANFDSPI_INDEX_0, cRAMADDR_START, txd, length);

        // Read data back from RAM
        DRV_CANFDSPI_ReadByteArray(DRV_CANFDSPI_INDEX_0, cRAMADDR_START, rxd, length);

        // Verify
        good = false;
        for (i = 0; i < length; i++)
        {
            good = txd[i] == rxd[i];

            if (!good)
            {
                Nop();
                Nop();

                // Data mismatch
                return false;
            }
        }
    }

    return true;
}
static void APP_CANFDSPI_Init(void *p)
{
    volatile uint8_t s = 0;
    DRV_SPI_Initialize();
    // Reset device
    s = DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0);

    // Enable ECC and initialize RAM
    s = DRV_CANFDSPI_EccEnable(DRV_CANFDSPI_INDEX_0);

    s = DRV_CANFDSPI_RamInit(DRV_CANFDSPI_INDEX_0, 0xff);

    // Configure device
    s = DRV_CANFDSPI_ConfigureObjectReset(&config);
    config.IsoCrcEnable = 1;
    config.StoreInTEF = 0;

    s = DRV_CANFDSPI_Configure(DRV_CANFDSPI_INDEX_0, &config);

    // Setup TX FIFO
    s = DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
    txConfig.FifoSize = 7;
    txConfig.PayLoadSize = CAN_PLSIZE_64;
    txConfig.TxPriority = 1;

    s = DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txConfig);

    // Setup RX FIFO
    s = DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
    rxConfig.FifoSize = 15;
    rxConfig.PayLoadSize = CAN_PLSIZE_64;

    s = DRV_CANFDSPI_ReceiveChannelConfigure(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &rxConfig);

    // Setup RX Filter
    fObj.word = 0;
    fObj.bF.SID = 0xda;
    fObj.bF.EXIDE = 0;
    fObj.bF.EID = 0x00;

    s = DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &fObj.bF);

    // Setup RX Mask
    mObj.word = 0;
    mObj.bF.MSID = 0x0;
    mObj.bF.MIDE = 1; // Only allow standard IDs
    mObj.bF.MEID = 0x0;

    s = DRV_CANFDSPI_FilterMaskConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &mObj.bF);

    // Link FIFO and Filter
    s = DRV_CANFDSPI_FilterToFifoLink(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, APP_RX_FIFO, true);

    // Setup Bit Time
    // DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_500K_5M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_20M);
    // DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_500K_5M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
    s = DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_500K_5M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);

    // Setup Transmit and Receive Interrupts
    s = DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);
#ifdef APP_USE_TX_INT
    s = DRV_CANFDSPI_TransmitChannelEventEnable(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, CAN_TX_FIFO_NOT_FULL_EVENT);
#endif
    s = DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, CAN_RX_FIFO_NOT_EMPTY_EVENT);
    s = DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_0, CAN_TX_EVENT | CAN_RX_EVENT);

    // Select Normal Mode
    s = DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_CLASSIC_MODE);
    //	DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_CLASSIC_MODE);
    // Reset device
    // s = DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0);
}
void spitest()
{
    volatile bool a = false;
    APP_CANFDSPI_Init(NULL);
    long b=0;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        // Configure transmit message
        txObj.word[0] = 0;
        txObj.word[1] = 0;

        txObj.bF.id.SID = TX_RESPONSE_ID;
        txObj.bF.id.EID = 0;

        txObj.bF.ctrl.BRS = 0;
        txObj.bF.ctrl.DLC = CAN_DLC_8;
        txObj.bF.ctrl.FDF = 0;
        txObj.bF.ctrl.IDE = 0;

        // Configure message data
        for (uint8_t i = 0; i < 8; i++)
            txd[i] = txObj.bF.id.SID + i;

        // a = APP_TestRamAccess();
        // ESP_LOGI(TAG,"%d",a);
        ESP_LOGI(TAG, "%d", DRV_CANFDSPI_TransmitChannelLoad(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txObj, txd, DRV_CANFDSPI_DlcToDataBytes(txObj.bF.ctrl.DLC), (b++ %7)==0));
    }
}