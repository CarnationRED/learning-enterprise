
/*!
    \file  main.c
    \brief SPI fullduplex communication use polling mode

    \version 2017-12-26, V1.0.0, firmware for GD32E10x
*/

/*
    Copyright (c) 2017, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

// Include files
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp32s3/rom/gpio.h"
#include "soc/gpio_periph.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "drv_spi.h"
#include "hal/gpio_hal.h"
#include "../main/include/can.h"

#define PIN_NUM_MISO 11
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK 12
#define PIN_NUM_CS 10

// Interrupts
#define INT_IN 8
#define INT_TX_IN 17
#define INT_RX_IN 18
#define ESP_INTR_FLAG_DEFAULT 0//定义默认的中断标志为0

#define SPI_FLASH_CS_LOW() gpio_set_level(PIN_NUM_CS, 0)
#define SPI_FLASH_CS_HIGH() gpio_set_level(PIN_NUM_CS, 1)

static spi_device_handle_t spi;
// static QueueHandle_t gpio_evt_queue = NULL;
void (*spican_rx_int_ptr)(void *para) = NULL;

/* Local function prototypes */
inline void spi_master_init(void);
inline int8_t spi_master_transfer(uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize);
// static void task_spi_rx_int(void *arg)
// {
//   uint32_t param;
//   for (;;)
//   {
//     if (xQueueReceive(gpio_evt_queue, &param, portMAX_DELAY))
//     {
//       int io_num = param & 0x00FFFFFF;
//       printf("GPIO[%" PRIu32 "] intr, val: %d\n", io_num, gpio_get_level(io_num));
//       if (spican_rx_int_ptr != NULL)
//         (*spican_rx_int_ptr)((void *)(param >> 24));
//     }
//   }
// }
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
  uint32_t gpio_num = (uint32_t)arg;
  gpio_num |= canCurrentChannel << 24;
  // xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);

  int io_num = gpio_num & 0x00FFFFFF;
  printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
  if (spican_rx_int_ptr != NULL)
    (*spican_rx_int_ptr)((void *)(canCurrentChannel));
}

void DRV_SPI_Initialize(void)
{
  spi_master_init();
  /**
   * 定义并配置GPIO口
   */
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_NEGEDGE; // 下降沿中断
  io_conf.mode = GPIO_MODE_INPUT;        // 输入模式
  io_conf.pin_bit_mask = INT_RX_IN;      // 配置要设置的引脚
  io_conf.pull_down_en = 0;              // 禁止下拉
  io_conf.pull_up_en = 0;                // 引脚电平上拉
  // 配置gpio
  gpio_config(&io_conf);

  // change gpio interrupt type for one pin
  gpio_set_intr_type(INT_RX_IN, GPIO_INTR_NEGEDGE);

  // create a queue to handle gpio event from isr
  // gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  // start gpio task
  // xTaskCreate(task_spi_rx_int, "task_spi_rx_int", 2048, NULL, 10, NULL);

  // install gpio isr service
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  // hook isr handler for specific gpio pin
  gpio_isr_handler_add(INT_RX_IN, gpio_isr_handler, (void *)INT_RX_IN);
}

int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
  return spi_master_transfer(SpiTxData, SpiRxData, spiTransferSize);
}
void spi_master_init(void)
{
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .miso_io_num = PIN_NUM_MISO,
      .mosi_io_num = PIN_NUM_MOSI,
      .sclk_io_num = PIN_NUM_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1};
  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 10 * 1000 * 1000, // Clock out at 20 MHz
      .mode = 0,                          // SPI mode 0
      .spics_io_num = PIN_NUM_CS,         // CS pin
      .cs_ena_pretrans = 1,
      .cs_ena_posttrans = 1,
      .queue_size = 40,
  };
  gpio_pad_select_gpio(PIN_NUM_CS);
  gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
  // Initialize the SPI bus
  ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);
  // Attach  to the SPI bus
  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);

  // gpio_set_direction(PIN_NUM_MISO, GPIO_MODE_INPUT);
  // // gpio_set_pull_mode(PIN_NUM_MISO, GPIO_FLOATING);
  // gpio_set_direction(PIN_NUM_MOSI, GPIO_MODE_OUTPUT);
  // gpio_set_direction(PIN_NUM_CLK, GPIO_MODE_OUTPUT);
  // Set up GPIO matrix
  // gpio_config_t io_conf;
  // io_conf.intr_type = GPIO_INTR_DISABLE;
  // io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
  // io_conf.pin_bit_mask = (1ULL << PIN_NUM_MISO) | (1ULL << PIN_NUM_CLK) | (1ULL << 9) | (1ULL << PIN_NUM_MOSI);
  // io_conf.pull_down_en = 0;
  // io_conf.pull_up_en = 0;
  // gpio_config(&io_conf);
  // gpio_matrix_out(9, FSPICS0_OUT_IDX, 0, 0);
  // // gpio_matrix_in(PIN_NUM_MISO, FSPID_IN_IDX, 0);
  // // gpio_matrix_out(PIN_NUM_CLK, FSPICLK_OUT_IDX, 0, 0);
  // // gpio_matrix_out(PIN_NUM_MOSI, FSPIQ_OUT_IDX, 0, 0);

  // REG_SET_FIELD(GPIO_FUNC11_OUT_SEL_CFG_REG, GPIO_FUNC11_OUT_SEL, FSPID_IN_IDX);
  // REG_SET_FIELD(GPIO_FUNC12_OUT_SEL_CFG_REG, GPIO_FUNC12_OUT_SEL, FSPICLK_OUT_IDX);
  // REG_SET_FIELD(GPIO_FUNC13_OUT_SEL_CFG_REG, GPIO_FUNC13_OUT_SEL, FSPIQ_OUT_IDX);

  // REG_CLR_BIT(GPIO_FUNC11_OUT_SEL_CFG_REG, GPIO_FUNC11_OEN_SEL);
  // REG_CLR_BIT(GPIO_FUNC12_OUT_SEL_CFG_REG, GPIO_FUNC12_OEN_SEL);
  // REG_CLR_BIT(GPIO_FUNC13_OUT_SEL_CFG_REG, GPIO_FUNC13_OEN_SEL);

  // PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[9], PIN_FUNC_GPIO);
  // PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_MISO], PIN_FUNC_GPIO);
  // PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_CLK], PIN_FUNC_GPIO);
  // PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_MOSI], PIN_FUNC_GPIO);
}

int8_t spi_master_transfer(uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
  spi_transaction_t spi_packet;
  memset(&spi_packet, 0, sizeof(spi_packet));
  spi_packet.tx_buffer = SpiTxData;
  spi_packet.rx_buffer = SpiRxData;
  spi_packet.rxlength = 8 * spiTransferSize;
  spi_packet.length = 8 * spiTransferSize;
  if (spi_device_transmit(spi, &spi_packet) != ESP_OK)
    return 1;
  return 0;
}
