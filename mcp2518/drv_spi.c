
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

#define PIN_NUM_MISO 11
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK 12
#define PIN_NUM_CS 10

#define SPI_FALSH_CS_PORT GPIOA
#define SPI_FALSH_CS_PIN GPIO_PIN_4

#define SPI_FLASH_CS_LOW() gpio_set_level(PIN_NUM_CS, 0)
#define SPI_FLASH_CS_HIGH() gpio_set_level(PIN_NUM_CS, 1)

static spi_device_handle_t spi;
/* Local function prototypes */
inline void spi_master_init(void);
inline int8_t spi_master_transfer(uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize);

void DRV_SPI_Initialize(void)
{
  spi_master_init();
}

int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
  return spi_master_transfer(SpiTxData, SpiRxData, spiTransferSize);
}static void gpio_matrix_out_check_and_set(gpio_num_t gpio, uint32_t signal_idx, bool out_inv, bool oen_inv)
{
    //if pin = -1, do not need to configure
    if (gpio != -1) {
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
        gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
        esp_rom_gpio_connect_out_signal(gpio, signal_idx, out_inv, oen_inv);
    }
}

static void gpio_matrix_in_check_and_set(gpio_num_t gpio, uint32_t signal_idx, bool inv)
{
    if (gpio != -1) {
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
        /* Set direction, for some GPIOs, the input function are not enabled as default */
        gpio_set_direction(gpio, GPIO_MODE_INPUT);
        esp_rom_gpio_connect_in_signal(gpio, signal_idx, inv);
    }
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
      .clock_speed_hz = 17 * 1000 * 1000, // Clock out at 20 MHz
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

  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[46], PIN_FUNC_GPIO);
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[12], PIN_FUNC_GPIO);
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[13], PIN_FUNC_GPIO);
  // gpio_set_direction(PIN_NUM_MISO, GPIO_MODE_INPUT);
  gpio_set_pull_mode(PIN_NUM_MISO, GPIO_FLOATING);
  // gpio_set_direction(PIN_NUM_MOSI, GPIO_MODE_OUTPUT);
  // gpio_set_direction(PIN_NUM_CLK, GPIO_MODE_OUTPUT);

  gpio_matrix_out_check_and_set(9, FSPICS0_OUT_IDX, 0, 0);
  gpio_matrix_in_check_and_set(46, FSPID_IN_IDX, 0);
  gpio_matrix_out_check_and_set(12, FSPICLK_OUT_IDX, 0, 0);
  gpio_matrix_out_check_and_set(13, FSPIQ_OUT_IDX, 0, 0);
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
