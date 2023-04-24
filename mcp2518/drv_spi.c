
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
#include "drv_spi.h"
#include "stdint.h"

#define PIN_NUM_MISO 11
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

#define SPI_FALSH_CS_PORT GPIOA
#define SPI_FALSH_CS_PIN GPIO_PIN_4

#define SPI_FLASH_CS_LOW() gpio_bit_reset(SPI_FALSH_CS_PORT, SPI_FALSH_CS_PIN)
#define SPI_FLASH_CS_HIGH() gpio_bit_set(SPI_FALSH_CS_PORT, SPI_FALSH_CS_PIN)

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
}

void spi_master_init(void)
{
  sp_err_t ret;
  spi_device_handle_t spi;
  spi_bus_config_t buscfg = {
      .miso_io_num = PIN_NUM_MISO,
      .mosi_io_num = PIN_NUM_MOSI,
      .sclk_io_num = PIN_NUM_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 8};
  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 40 * 1000 * 1000, // Clock out at 40 MHz
      .mode = 0,                               // SPI mode 0
      .spics_io_num = PIN_NUM_CS,              // CS pin
      .queue_size = 7,                         // We want to be able to queue 7 transactions at a time
      // .pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
  };
  // Initialize the SPI bus
  ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);
  // Attach the LCD to the SPI bus
  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);

  gpio_pad_select_gpio(PIN_NUM_CS);
  gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);

  spi_parameter_struct spi_init_struct;

  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(RCU_SPI0);

  /* SPI0 GPIO config:SCK/PA5, MISO/PA6, MOSI/PA7 */
  gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
  gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

  gpio_init(SPI_FALSH_CS_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPI_FALSH_CS_PIN);

  //	 GPIO_SetBits(GPIOB, GPIO_Pin_12);

  SPI_FLASH_CS_HIGH();
  /* SPI0 Config -------------------------------------------------------------*/

  /* deinitilize SPI and the parameters */
  spi_i2s_deinit(SPI0);

  spi_struct_para_init(&spi_init_struct);

  /* SPI0 parameter config */
  spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
  spi_init_struct.device_mode = SPI_MASTER;
  spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
  spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
  spi_init_struct.nss = SPI_NSS_SOFT;
  spi_init_struct.prescale = SPI_PSC_8;
  spi_init_struct.endian = SPI_ENDIAN_MSB;
  spi_init(SPI0, &spi_init_struct);

  spi_enable(SPI0);
}

// #define USE_SPI_FUNCTIONS

int8_t spi_master_transfer(uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
  uint16_t pos = 0;

  SPI_FLASH_CS_LOW();

  while (pos < spiTransferSize)
  {
    /* Loop while DR register in not emplty */
    while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE))
      ;
    /* Send byte through the SPI2 peripheral */
    spi_i2s_data_transmit(SPI0, SpiTxData[pos]);

    /* Wait to receive a byte */
    while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE))
      ;

    /* Return the byte read from the SPI bus */

    SpiRxData[pos] = spi_i2s_data_receive(SPI0);

    pos++;
  }

  SPI_FLASH_CS_HIGH();

  return 0;
}
