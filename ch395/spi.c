#include "spi.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp32s3/rom/gpio.h"
#include "soc/gpio_periph.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "hal/gpio_hal.h"
#include "string.h"

static spi_device_handle_t spi = NULL;

void spi1_init(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_DO,
        .mosi_io_num = PIN_DI,
        .sclk_io_num = PIN_CK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .isr_cpu_id = 1,
        .intr_flags = ESP_INTR_FLAG_LEVEL3};
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000, // Clock out at 30 MHz
        .mode = 0,                          // SPI mode 0
        .spics_io_num = PIN_CS,                 // CS pin
        .cs_ena_pretrans = 1,
        .cs_ena_posttrans = 1,
        .queue_size = 400,
    };

    // gpio_config_t conf;
    // conf.intr_type=GPIO_INTR_DISABLE;
    // conf.mode=GPIO_MODE_OUTPUT;
    // conf.pin_bit_mask=PIN_NUM_CS;
    // conf.pull_down_en=GPIO_PULLDOWN_DISABLE;
    // conf.pull_up_en=GPIO_PULLUP_DISABLE;
    // gpio_config(&conf);
    // gpio_reset_pin(PIN_NUM_CS);
    if (spi != NULL)
    {
        spi_bus_remove_device(spi);
        spi = NULL;
        spi_bus_free(SPI3_HOST);
    }
    gpio_pad_select_gpio(PIN_CS);
    gpio_set_direction(PIN_CS, GPIO_MODE_OUTPUT);
    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach  to the SPI bus
    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

uint8_t spi1_read_write_byte(uint8_t write)
{
    uint8_t read;
    spi_transaction_t spi_packet;
    memset(&spi_packet, 0, sizeof(spi_packet));
    spi_packet.tx_buffer = &write;
    spi_packet.rx_buffer = &read;
    spi_packet.rxlength = 8 * 1;
    spi_packet.length = 8 * 1;
    
    if (spi_device_transmit(spi, &spi_packet) != ESP_OK)
    {
        return read;
    }
    return -1;
}