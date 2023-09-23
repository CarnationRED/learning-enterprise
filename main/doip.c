#include "doip.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void doip_init()
{
    ch395_hardware_init();
    vTaskDelay(pdMS_TO_TICKS(200));
    while (1)
    {
        ESP_LOGI("ch395", "%d", ch395_cmd_check_exist(149));
        ESP_LOGI("ch395 stat", "%d", ch395_cmd_get_phy_status());
        vTaskDelay(500);
    }
}