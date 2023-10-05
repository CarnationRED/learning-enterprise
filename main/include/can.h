#ifndef __CAN_H__
#define __CAN_H__
#include "msg.h"
#include "fifo.h"
#include "cQueue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"

// Interrupts
#define INT_IN 8
#define INT_TX_IN 17
#define INT_RX_IN 18

#define APP_INT() (!gpio_get_level(INT_IN))
#define APP_TX_INT() (!gpio_get_level(INT_TX_IN)) // 1：TXQ not full
#define APP_RX_INT() (!gpio_get_level(INT_RX_IN)) // 1：RX FIFO not empty

#define CH444G_SW0 36
#define CH444G_SW1 35
#define CH444G_SWEN 45
#define SWENABLE() gpio_pullup_en(CH444G_SWEN)
#define SWDISABLE() gpio_pulldown_en(CH444G_SWEN)
#define SW0_0() gpio_set_level(CH444G_SW0, 0)
#define SW1_0() gpio_set_level(CH444G_SW1, 0)
#define SW0_1() gpio_set_level(CH444G_SW0, 1)
#define SW1_1() gpio_set_level(CH444G_SW1, 1)

// Message IDs
#define TX_REQUEST_ID 0x300
#define TX_RESPONSE_ID 0x301
#define BUTTON_STATUS_ID 0x201
#define LED_STATUS_ID 0x200
#define PAYLOAD_ID 0x101

// Transmit Channels
#define APP_TX_FIFO CAN_FIFO_CH2
// #define APP_USE_TX_INT 1

// Receive Channels
#define APP_RX_FIFO CAN_FIFO_CH1

void spitest();
void can_init();
void APP_CANFDSPI_Init(void *p);
bool canrx_fifo_not_empty();
static bool canEnqueueOneFrame(CAN_CMD_FRAME * msg);
static bool canEnqueueMultiFrame(CAN_CMD_MULTIFRAME * msg);
bool canSendOneFrame(CAN_CMD_FRAME * msg);
static bool canSetFilter(CAN_FILTER_CFG *flt);
static bool canSetCanChl(u8 *channel);


typedef enum
{
    CAN_STAT_INIT,
    CAN_STAT_ERRORINIT,
    CAN_STAT_IDLE,
    CAN_STAT_TXQFULL,
    CAN_STAT_TXERROR,
    CAN_STAT_RXQFULL,
    CAN_STAT_RXERROR,
    CAN_STAT_RXQERROR,
    CAN_STAT_DATAERROR,
} CAN_STATS;

// extern QueueHandle_t rxMsgFifo;
extern u8 canCurrentChannel;
#endif