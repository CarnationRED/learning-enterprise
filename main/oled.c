#ifndef __OLED__
#define __OLED__

#include <stdio.h>
#include "u8g2.h"
#include "oled.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO 47         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 48         /*!< GPIO number used for I2C master data  */
#define I2C_SCL_IO I2C_MASTER_SCL_IO /*!< GPIO number used for I2C master clock */
#define I2C_SDA_IO I2C_MASTER_SDA_IO /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0             /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000    /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0  /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0  /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */
#define portTICK_RATE_MS portTICK_PERIOD_MS
/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, 1024, 0);
}

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
void u8g2Init(u8g2_t *u8g2)
{
    volatile int a = 11;
    a = i2c_master_init();
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2, U8G2_R0, u8x8_byte_i2c, u8x8_gpio_and_delay); // 初始化 u8g2 结构体
    u8g2_InitDisplay(u8g2);                                                                    // 根据所选的芯片进行初始化工作，初始化完成后，显示器处于关闭状态
    u8g2_SetPowerSave(u8g2, 0);                                                                // 打开显示器
    u8g2_ClearBuffer(u8g2);
}

static __inline void delay_clock(int t_sysInit)
{
    uint32_t start, curr;

    __asm__ __volatile__("rsr %0, ccount"
                         : "=r"(start));
    do
        __asm__ __volatile__("rsr %0, ccount"
                             : "=r"(curr));
    while (curr - start <= t_sysInit);
}
#define delay_us(val) delay_clock(240 * val)
#define delay_100ns(val) delay_clock(24 * val)

int esp32_i2c_write(uint8_t addr, uint32_t idx, uint8_t *data)
{
    volatile int ret;
    i2c_cmd_handle_t handler = i2c_cmd_link_create();
    i2c_master_start(handler);
    i2c_master_write_byte(handler, addr | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(handler, data, idx, 2);
    i2c_master_stop(handler);
    ret = i2c_master_cmd_begin(I2C_NUM_0, handler, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(handler);
    return ret;
}

// u8g2用到的系统资源
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg)
    {
    case U8X8_MSG_DELAY_100NANO: // delay arg_int * 100 nano seconds
        delay_100ns(arg_int);
        break;
    case U8X8_MSG_DELAY_10MICRO: // delay arg_int * 10 micro seconds
        delay_us(arg_int * 10);
        break;
    case U8X8_MSG_DELAY_MILLI:               // delay arg_int * 1 milli second
        vTaskDelay(1000 / portTICK_RATE_MS); // 在这里既然使用了freertos的vTaskDelay，那么u8g2必须在进程上下文运行
        break;
    case U8X8_MSG_DELAY_I2C: // arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
        delay_us(10 / arg_int);
        break;                    // arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
    case U8X8_MSG_GPIO_I2C_CLOCK: // arg_int=0: Output low at I2C clock pin
        if (arg_int == 1)
        {
            gpio_set_level(I2C_SCL_IO, 1);
        }
        else if (arg_int == 0)
        {
            gpio_set_level(I2C_SCL_IO, 0);
        }
        break;                   // arg_int=1: Input dir with pullup high for I2C clock pin
    case U8X8_MSG_GPIO_I2C_DATA: // arg_int=0: Output low at I2C data pin
        if (arg_int == 1)
        {
            gpio_set_level(I2C_SDA_IO, 1);
        }
        else if (arg_int == 0)
        {
            gpio_set_level(I2C_SDA_IO, 0);
        }
        break; // arg_int=1: Input dir with pullup high for I2C data pin
    case U8X8_MSG_GPIO_MENU_SELECT:
        u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_NEXT:
        u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_PREV:
        u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_HOME:
        u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
        break;
    default:
        u8x8_SetGPIOResult(u8x8, 1); // default return value
        break;
    }
    return 1;
}
// u8g2用到的显示屏控制接口
uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    static uint8_t buffer[32]; /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
    static uint8_t buf_idx;
    uint8_t *data;

    switch (msg)
    {
    case U8X8_MSG_BYTE_SEND:
        data = (uint8_t *)arg_ptr;
        while (arg_int > 0)
        {
            buffer[buf_idx++] = *data;
            data++;
            arg_int--;
        }
        break;
    case U8X8_MSG_BYTE_INIT:
        /* add your custom code to init i2c subsystem */
        break;
    case U8X8_MSG_BYTE_SET_DC:
        /* ignored for i2c */
        break;
    case U8X8_MSG_BYTE_START_TRANSFER:
        buf_idx = 0;
        break;
    case U8X8_MSG_BYTE_END_TRANSFER:
        esp32_i2c_write(u8x8_GetI2CAddress(u8x8), buf_idx, buffer);
        break;
    default:
        return 0;
    }
    return 1;
}
#endif // !__U8G2__