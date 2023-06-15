#include "can.h"
#include "drv_spi.h"
#include "drv_canfdspi_api.h"
#include <stdio.h>
#include "u8g2.h"
#include "oled.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "driver/i2c.h"

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
void APP_CANFDSPI_Init(void *p)
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

    // Configure Transmit Event FIFO
    // CAN_TEF_CONFIG tefCfg;
    // s = DRV_CANFDSPI_TefConfigureObjectReset(&tefCfg);
    // tefCfg.FifoSize = 10;
    // tefCfg.TimeStampEnable = 1;
    // s = DRV_CANFDSPI_TefConfigure(DRV_CANFDSPI_INDEX_0, &tefCfg);

    // Setup TX FIFO
    s = DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
    txConfig.FifoSize = 12;
    txConfig.PayLoadSize = CAN_PLSIZE_64;
    txConfig.TxPriority = 1;

    s = DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txConfig);

    // Setup RX FIFO
    s = DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
    rxConfig.FifoSize = 12;
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
    long b = 0;
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
        ESP_LOGI("CAN", "%d", DRV_CANFDSPI_TransmitChannelLoad(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txObj, txd, DRV_CANFDSPI_DlcToDataBytes(txObj.bF.ctrl.DLC), (b++ % 7) == 0));
    }
}
