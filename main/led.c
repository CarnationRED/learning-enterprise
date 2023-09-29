#include "led.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include <stdbool.h>
#include "esp_log.h"

bool led1, led2;
bool fls1, fls2;
bool flsStat1, flsStat2;
uint16_t t1 = 0xffff, t2 = 0xffff;
static void led_task()
{
    uint32_t t, next1, next2;

    next1 = next2 = t = pdTICKS_TO_MS(xTaskGetTickCount());
    while (1)
    {
        uint16_t min = t1;
        if (t2 < t1)
            min = t2;

        t = pdTICKS_TO_MS(xTaskGetTickCount());

        if (fls1)
        {
            if (t >= next1)
            {
                gpio_set_level(LED1, (flsStat1 = !flsStat1));
                next1 = t + t1;
            }
        }
        else
        {
            gpio_set_level(LED1, led1);
        }

        if (fls2)
        {
            if (t >= next2)
            {
                gpio_set_level(LED2, (flsStat2 = !flsStat2));
                next2 = t + t2;
            }
        }
        else
        {
            gpio_set_level(LED2, led2);
        }
        vTaskDelay(pdMS_TO_TICKS(min));
    }
}

void led_init()
{
    led1 = led2 = false;
    fls1 = fls2 = false;
    flsStat1 = flsStat2 = false;

    gpio_config_t g;
    g.mode = GPIO_MODE_OUTPUT;
    g.intr_type = GPIO_INTR_DISABLE;
    g.pin_bit_mask = LED1;
    gpio_config(&g);
    g.pin_bit_mask = LED2;
    gpio_config(&g);

    xTaskCreatePinnedToCore(led_task, "led_task", 2048, NULL, 0, NULL, 1);
}

void led1Flash(bool on, uint8_t hz)
{
    fls1 = on;
    if (!on)
    {
        led1 = false;
        t1 = 0xffff;
        return;
    }
    if (!hz)
        hz = 1;
    t1 = 1000 / hz;
    if (t1 < 5)
        t1 = 5;
}
void led2Flash(bool on, uint8_t hz)
{
    fls2 = on;
    if (!on)
    {
        led2 = false;
        t2 = 0xffff;
        return;
    }
    if (!hz)
        hz = 1;
    t2 = 1000 / hz;
    if (t2 < 5)
        t2 = 5;
}

// void led1Switch(bool on)
// {
//     led1 = on;
// }
// void led2Switch(bool on)
// {
//     led2 = on;
// }

gptimer_handle_t gptimer1 = NULL, gptimer2 = NULL;
uint8_t duty1, duty2;
uint32_t updownTotal1, updownTotal2;
uint16_t updownPeriod1, updownPeriod2;
void led1Switch(bool on)
{
    duty1 = ((1 << LEDC_TIMER_13_BIT) - 1) * on;
    // Set duty to 0%
    (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty1));
    // Update duty to apply the new value
    (ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}
void led2Switch(bool on)
{
    led2 = on;
}
static void IRAM_ATTR led1_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    if (updownTotal1 == 0)
        return;
    if (updownTotal1 / 2 * 2 == updownTotal1)
    {
        // ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty1, updownPeriod1);
        // ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
        // Set duty to beightness%
        (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty1));
        // Update duty to apply the new value
        (ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    }
    else
    {
        // ledc_fade_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        // ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, updownPeriod1);
        // ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
        // Set duty to beightness%
        (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
        // Update duty to apply the new value
        (ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    }
    updownTotal1--;
}
static void IRAM_ATTR led2_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    if (updownTotal2 == 0)
        return;
    if (updownTotal2 / 2 * 2 == updownTotal2)
    {
        (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty2));
        (ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
    }
    else
    {
        (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0));
        (ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
    }
    updownTotal2--;
}
void led_init2()
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, // esp32s3只支持低速率
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LED1,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.gpio_num = LED2;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    // ledc_fade_func_install(0);

    const char *TAG = "led";

    gptimer_config_t timer_config1 = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 10000, // 10kHz, 1 tick=0.1ms
    };
    gptimer_config_t timer_config2 = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 10000, // 10kHz, 1 tick=0.1ms
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config1, &gptimer1));
    assert(gptimer1);
    ESP_LOGI(TAG, "Create timer1 handle");
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config2, &gptimer2));
    assert(gptimer2);
    ESP_LOGI(TAG, "Create timer2 handle");

    gptimer_event_callbacks_t cbs = {
        .on_alarm = led1_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer1, &cbs, NULL));
    cbs.on_alarm =led2_cb;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer2, &cbs, NULL));

    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer1));
    ESP_ERROR_CHECK(gptimer_start(gptimer1));
    ESP_ERROR_CHECK(gptimer_enable(gptimer2));
    ESP_ERROR_CHECK(gptimer_start(gptimer2));
}

void led1Flash2(uint8_t hz, uint32_t times, uint8_t brightness)
{
    // // Set duty to beightness%
    // ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, ((1 << LEDC_TIMER_13_BIT) - 1) * 100 / brightness));
    // // Update duty to apply the new value
    // ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    if (!hz)
        return;
    if (hz > 500)
        hz = 500;
    duty1 = ((1 << LEDC_TIMER_13_BIT) - 1) * ((float)brightness / 100);
    updownTotal1 = times * 2;

    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = updownPeriod1 = 10000 / hz / 2, // period = timeout ms
        .reload_count = 0,
        .flags.auto_reload_on_alarm = 1,
    };
    assert(gptimer1);
    assert(gptimer2);

    ESP_LOGI("","alarm_count%ld",(long)alarm_config1.alarm_count);
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer1, &alarm_config1));

    gptimer_stop(gptimer1);
    gptimer_start(gptimer1);

    // Set duty to 0%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

void led2Flash2(uint8_t hz, uint32_t times, uint8_t brightness)
{
    // // Set duty to beightness%
    // ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, ((1 << LEDC_TIMER_13_BIT) - 1) * 100 / brightness));
    // // Update duty to apply the new value
    // ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    if (!hz)
        return;
    if (hz > 500)
        hz = 500;
    duty2 = ((1 << LEDC_TIMER_13_BIT) - 1) * ((float)brightness / 100);
    updownTotal2 = times * 2;

    gptimer_alarm_config_t alarm_config2 = {
        .alarm_count = updownPeriod2 = 10000 / hz / 2, // period = timeout ms
        .reload_count = 0,
        .flags.auto_reload_on_alarm = 1,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer2, &alarm_config2));

    gptimer_stop(gptimer2);
    gptimer_start(gptimer2);

    // Set duty to 0%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}
