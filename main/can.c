#ifndef __CAN_C__
#define __CAN_C__

#include "stdbool.h"
#include <stdio.h>
#include "msg.h"
#include "can.h"
#include "drv_spi.h"
#include "drv_canfdspi_api.h"
#include "u8g2.h"
#include "oled.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "mqtt.h"
#include "cQueue.h"
#include "led.h"
extern int push_wait;
extern int pull_wait;
extern int recvedMsgs;

CAN_STATS canStats = CAN_STAT_INIT;

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
u8 canCurrentChannel = -1;

// Message IDs
#define TX_REQUEST_ID 0x300
#define TX_RESPONSE_ID 0x301
#define BUTTON_STATUS_ID 0x201
#define LED_STATUS_ID 0x200
#define PAYLOAD_ID 0x101

// Transmit Channels
#define APP_TX_FIFO CAN_FIFO_CH2
#define APP_USE_TX_INT 1

// Receive Channels
#define APP_RX_FIFO CAN_FIFO_CH1
static CAN_CONFIG config;
static CAN_OPERATION_MODE opMode;

// Transmit objects
static CAN_TX_FIFO_CONFIG txConfig;
static CAN_TX_FIFO_EVENT txFlags;
static CAN_TX_MSGOBJ txObj = {};
static uint8_t txd[MAX_DATA_BYTES];

// Receive objects
static CAN_RX_FIFO_CONFIG rxConfig;
static REG_CiFLTOBJ fObj;
static REG_CiMASK mObj;
static CAN_RX_FIFO_EVENT rxFlags;

#define RX_MSGLEN 100
#define TX_MSGLEN 32
#define TX_MULTIMSGLEN 4
static TaskHandle_t txHandle, rxHandle;
//handle to use in canrx, should be set to one of wifiSendHandle and udsSendHandle
TaskHandle_t canRxHandle;
//handle of wifiSend task
TaskHandle_t wifiSendHandle;
//handle of udsSend task
TaskHandle_t udsSendHandle;
QueueHandle_t rxMsgFifo;
QueueHandle_t txMsgFifo;
QueueHandle_t txMultiMsgFifo;
QueueSetHandle_t txFifoSet;


static CAN_MSG_FRAME msg = {.direction = 0};
uint8_t canSendWifiTxBuffer[16384];
long wifiSent = 0, canSent = 0;

extern void (*spican_rx_int_ptr)(u8 para);
extern bool (*canQueuePtr)(CAN_CMD_FRAME *msg);
extern bool (*canQueueMultiPtr)(CAN_CMD_MULTIFRAME *msgs);
extern bool (*canSetFilterPtr)(CAN_FILTER_CFG *flt);
extern bool (*canSetCanChlPtr)(u8 *channel);
extern bool mqttconnected;
static char *canErrStr = "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF\0";

void (*canSend2Wifi)(uint8_t *data, int len) = NULL;

static bool APP_TestRamAccess(void)
{
    // Variables
    uint16_t i = 0;
    uint8_t length;
    bool good = false;
    u8 rxd[64];
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

static void can_rx_int_handler(u8 para)
{
    BaseType_t xHigherProrityTaskWoken = pdFALSE;
    msg.channel = canCurrentChannel;
    vTaskNotifyGiveFromISR(rxHandle, &xHigherProrityTaskWoken);
    portYIELD_FROM_ISR(xHigherProrityTaskWoken);
}
void canChannelInit()
{
    esp_rom_gpio_pad_select_gpio(CH444G_SW0);
    gpio_set_direction(CH444G_SW0, GPIO_MODE_OUTPUT);
    // gpio_set_pull_mode(CH444G_SW0, GPIO_PULLUP_PULLDOWN);
    esp_rom_gpio_pad_select_gpio(CH444G_SW1);
    gpio_set_direction(CH444G_SW1, GPIO_MODE_OUTPUT);
    // gpio_set_pull_mode(CH444G_SW1, GPIO_PULLUP_PULLDOWN);
    esp_rom_gpio_pad_select_gpio(CH444G_SWEN);
    // gpio_set_direction(CH444G_SWEN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(CH444G_SWEN, GPIO_PULLUP_PULLDOWN);
}
static void task_canrx()
{
    u8 result;
    u8 count = 0;
    CAN_RX_FIFO_EVENT evt;
    while (1)
    {
        DRV_CANFDSPI_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &evt);
        if (!(evt & CAN_RX_FIFO_NOT_EMPTY_EVENT))
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        count = 0;
        while (APP_RX_INT())
        {
            result = DRV_CANFDSPI_ReceiveMessageGet(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &msg.rxObj, msg.data, MAX_DATA_BYTES);
            if (0 != result)
            {
                canStats = CAN_STAT_RXERROR;
                sprintf(canErrStr, "can get recv err: %d", result);
                report(canErrStr);
                continue;
            }
            else
            {
                BaseType_t receiveTaskWoken = pdFALSE;
                if (!xQueueSendFromISR(rxMsgFifo, (void *)&msg, &receiveTaskWoken))
                {
                    sprintf(canErrStr, "can recv fifo full, discard");
                    report(canErrStr);
                    continue;
                }
                else
                {
                    if (receiveTaskWoken)
                    {
                        portYIELD_FROM_ISR();
                    }
                }
                count++;
                // sprintf(canErrStr, "0x%X", msg.rxObj.bF.id.SID);
                // ESP_LOGI("canrx", "count: %d\ttick:%d\t%s", count, tick, canErrStr);
            }
            if (count == 12)
            {
                count = 0;
                xTaskNotifyGive(canRxHandle);//Optimize to ISR function
            }
        }
        xTaskNotifyGive(canRxHandle);
    }
}
static void task_cantx()
{
    volatile int t;
    CAN_CMD_FRAME msg1;
    CAN_CMD_MULTIFRAME frames;
    while (1)
    {
        QueueHandle_t q = xQueueSelectFromSet(txFifoSet, portMAX_DELAY);
        if (q == NULL)
            continue;
        // ESP_LOGI("cantx","xQueueSelectFromSet");
        if (q == txMsgFifo)
        {
            while (xQueueReceive(q, &msg1, 0))
            {
                if (!canSendOneFrame(&msg1))
                {
                    // mqtt_report_pulish(canErrStr, 0);
                }
            }
        }
        else if (q == txMultiMsgFifo)
        {
            while (xQueueReceive(q, &frames, 0))
            {
                u8 dlc = frames.txObj.bF.ctrl.DLC;
                msg1.txObj = frames.txObj;
                msg1.channel = frames.channel;
                u8 *d = frames.data;
                for (size_t i = 0; i < frames.frames; i++)
                {
                    memcpy(msg1.data, d, dlc);
                    if (!canSendOneFrame(&msg1))
                    {
                        // mqtt_report_pulish(canErrStr, 0);
                    }
                    d += dlc;
                }
            }
        }
        else
        {
            canErrStr = "fifo name error";
            report(canErrStr);
        }
    }
}
// static void sendWifiOnce(int count)
// {
//     void *ptr = (void *)&canSendWifiTxBuffer;
//     for (int i = count; i > 0; i--)
//     {
//         if (!q_pop(&rxMsgFifo, ptr))
//         {
//             sprintf(canErrStr, "pop from rxMsgFifo failed");
//             mqtt_report_pulish(canErrStr, 0);
//         }
//         ptr += sizeof(CAN_MSG_FRAME);
//     }
//     // txb[count * sizeof(CAN_MSG_FRAME)] = '\0';
//     canSend2Wifi(canSendWifiTxBuffer, count * sizeof(CAN_MSG_FRAME));
//     // ESP_LOGI("can2wifi", "sending %d msgs", count);
// }
static void task_can2wifi()
{
    while (!mqttconnected)
        vTaskDelay(pdMS_TO_TICKS(80));
    APP_CANFDSPI_Init(NULL);
    // spitest();
    const int maxCountPerPacket = sizeof(canSendWifiTxBuffer) / sizeof(CAN_MSG_FRAME);

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        assert(canSend2Wifi != NULL);
        int count = 0;
        void *ptr = (void *)&canSendWifiTxBuffer;
        while (count <= maxCountPerPacket && xQueueReceive(rxMsgFifo, ptr, portTICK_PERIOD_MS * 1))
        {
            ptr += sizeof(CAN_MSG_FRAME);
            count++;
        }
        if (count)
        {
            canSend2Wifi(canSendWifiTxBuffer, count * sizeof(CAN_MSG_FRAME));
            wifiSent += count;
        }

        // for (int count = 0; (count = q_getCount(&rxMsgFifo)) > 0;)
        // {
        //     if (count > maxCountPerPacket)
        //         count = maxCountPerPacket;
        //     sendWifiOnce(count);
        //     wifiSent += count;
        // }
        // ESP_LOGI("can2wifi sent", "%d", sent);
    }
}
static void task_dbg_print()
{
    long lastWifiSent = 0;
    long lastCanSent = 0;
    long t0 = canSent;
    long t1 = wifiSent;
    // Idle counter
    u8 times = 0;
    while (1)
    {
        // No activities
        if (t0 + t1 != 0 && canSent - t0 == 0 && wifiSent - t1 == 0)
        {
            times++;
        }
        // Reset idle counter
        else
        {
            t0 = canSent;
            t1 = wifiSent;
            times = 0;

            // flash led2
            if (t0 + t1 > 0)
                led2Flash2(8, 0xffffffff, 100);
        }
        // Idle time > 1s and transmission happened
        if (times == 2 && wifiSent - lastWifiSent + canSent - lastCanSent != 0)
        {
            times = 0;
            ESP_LOGI("wifi sent", "%d\tcan sent:%d\n     Free Heap: %dKB\n     pushwait:%d\tpullwait:%d\trecvedMsgs:%d", (int)(wifiSent - lastWifiSent), (int)(canSent - lastCanSent), xPortGetFreeHeapSize() / 1024, push_wait, pull_wait, recvedMsgs);
            lastWifiSent = wifiSent;
            lastCanSent = canSent;

            push_wait = 0;
            pull_wait = 0;
            // led2 stop flash
            led2Flash2(10, 0, 0);
        }
        vPortYield();
        vTaskDelay(500);
    }
}
void can_init()
{
    canChannelInit();
    SWENABLE();
    spican_rx_int_ptr = &can_rx_int_handler;
    canQueuePtr = canEnqueueOneFrame;
    canQueueMultiPtr = canEnqueueMultiFrame;
    canSetFilterPtr = canSetFilter;
    canSetCanChlPtr = canSetCanChl;
    // q_init(&txMsgFifo, sizeof(CAN_CMD_FRAME), 1000, FIFO, false);
    // q_init(&rxMsgFifo, sizeof(CAN_MSG_FRAME), 1000, FIFO, false);
    rxMsgFifo = xQueueCreate(RX_MSGLEN, sizeof(CAN_MSG_FRAME));
    txMsgFifo = xQueueCreate(TX_MSGLEN, sizeof(CAN_CMD_FRAME));
    txMultiMsgFifo = xQueueCreate(TX_MULTIMSGLEN, sizeof(CAN_CMD_MULTIFRAME));


    txFifoSet = xQueueCreateSet(TX_MSGLEN + TX_MULTIMSGLEN + 10);
    xQueueAddToSet(txMsgFifo, txFifoSet);
    xQueueAddToSet(txMultiMsgFifo, txFifoSet);

    xTaskCreatePinnedToCore(task_canrx, "task_canrx", 4096, NULL, 12, &rxHandle, 1);
    xTaskCreatePinnedToCore(task_cantx, "task_cantx", 4096, NULL, 24, &txHandle, 1);
    xTaskCreatePinnedToCore(task_can2wifi, "task_can2wifi", 4096, NULL, 11, &wifiSendHandle, 0);
    xTaskCreatePinnedToCore(task_dbg_print, "task_dbg_print", 2048, NULL, 0, NULL, 0);
    canRxHandle = wifiSendHandle;
}
void can_clearFilter()
{
    CAN_FILTER_CFG c;
    c.enabled = false;
    ESP_LOGI("can", "clear filter");
    for (size_t i = 0; i < 32; i++)
    {
        c.filterId = i;
        canSetFilter(&c);
    }
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
    config.RestrictReTxAttempts = 1;

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
    txConfig.TxAttempts = 1;

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

    can_clearFilter();
}
void spitest()
{
    volatile bool a = false;
    APP_CANFDSPI_Init(NULL);
    long b = 0;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
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
bool _canWaitTxFifoEmpty()
{
    CAN_TX_FIFO_STATUS stat;
    for (u8 i = 0; i < 10; i++)
    {
        if (0 != DRV_CANFDSPI_TransmitQueueStatusGet(DRV_CANFDSPI_INDEX_0, &stat))
        {
            report("get can module tx failed");
            return false;
        }
        if (stat & CAN_TX_FIFO_EMPTY)
            return true;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    report("wait can module tx timeout");
    return false;
}
bool canSwitchChannel(u8 channel)
{
    // SW0		SW1		CAN			CHANNEL	        CAN_CHIPSELECT
    // 0		0		6/14		0				DRV_CANFDSPI_INDEX_0
    // 1		0		11/12		1				DRV_CANFDSPI_INDEX_0
    // 0		1		3/8			2				DRV_CANFDSPI_INDEX_0
    // 1		1		2/10		3				DRV_CANFDSPI_INDEX_0
    if (!(channel >= 0 && channel <= 4))
        return false;
    u8 b = channel != canCurrentChannel;
    if (b)
    {
        switch (channel)
        {
        case 0:
            _canWaitTxFifoEmpty();
            SW0_0();
            SW1_0();
            break;
        case 1:
            _canWaitTxFifoEmpty();
            SW0_1();
            SW1_0();
        case 2:
            _canWaitTxFifoEmpty();
            SW0_0();
            SW1_1();
            break;
        case 3:
            _canWaitTxFifoEmpty();
            SW0_1();
            SW1_1();
            break;
        default:
            return false;
        }
        // sprintf(canErrStr, "channel switch: %d->%d", canCurrentChannel, channel);
        canCurrentChannel = channel;
        vTaskDelay(pdMS_TO_TICKS(1));
        // mqtt_report_pulish(canErrStr, 0);
    }
    return true;
}
static bool canEnqueueOneFrame(CAN_CMD_FRAME *msg1)
{
    BaseType_t postTaskWoken = pdFALSE;
    // ESP_LOGI("enqueue single", "is inside ISR%d", xPortInIsrContext());
    if (xQueueSend(txMsgFifo, (void *)msg1, 1))
    {
        // if (postTaskWoken)
        // {
        //     portYIELD_FROM_ISR();
        // }
        // xTaskNotifyGive(txHandle);
        return true;
    }
    else
    {
        canErrStr = "can tx fifo full, discard";
        report(canErrStr);
        return false;
    }
}
static bool canEnqueueMultiFrame(CAN_CMD_MULTIFRAME *msgs)
{
    BaseType_t postTaskWoken = pdFALSE;
    // ESP_LOGI("enqueue multi", "is inside ISR: %d", xPortInIsrContext());
    if (xQueueSend(txMultiMsgFifo, (void *)msgs, 2))
    {
        // if (postTaskWoken)
        // {
        //     portYIELD_FROM_ISR();
        // }
        // xTaskNotifyGive(txHandle);
        return true;
    }
    else
    {
        canErrStr = "can tx fifo full, discard";
        report(canErrStr);
        return false;
    }
}
static bool canSendOneFrame(CAN_CMD_FRAME *msg1)
{
    if (canSwitchChannel(msg1->channel)) // switch to correct channel
    {
        u8 retry = 0;
        u8 attempts = 50;
        // Check if FIFO is not full
        do
        {
            DRV_CANFDSPI_TransmitChannelEventGet(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txFlags);
            if (attempts == 0)
            {
                Nop();
                Nop();
                // DRV_CANFDSPI_ErrorCountStateGet(DRV_CANFDSPI_INDEX_0, &tec, &rec, &errorFlags);

                canStats = CAN_STAT_TXQFULL;
                canErrStr = "wait txq timeout";
                report(canErrStr);
                break;
            }
            attempts--;
        } while (!(txFlags & CAN_TX_FIFO_NOT_FULL_EVENT));

        // while (!APP_TX_INT()) // wait txq not full, timeout 10*50ms = 500ms
        // {
        //     if (retry++ == 10)
        //     {
        //         canStats = CAN_STAT_TXQFULL;
        //         canErrStr = "wait txq timeout";
        //         mqtt_report_pulish(canErrStr, 0);
        //         APP_CANFDSPI_Init(NULL);
        //         break;
        //     }
        //     vTaskDelay(pdMS_TO_TICKS(50));
        // }
        int sendresult = DRV_CANFDSPI_TransmitChannelLoad(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &(msg1->txObj), msg1->data, DRV_CANFDSPI_DlcToDataBytes(msg1->txObj.bF.ctrl.DLC), true);
        if (sendresult != 0)
        {
            ESP_LOGI("CAN", "Error: %d", sendresult);
            canStats = CAN_STAT_TXERROR;
            report(canErrStr);
            return false;
        }
        canSent++;
        return true;
    }
    canErrStr = "channel switch failed";
    report(canErrStr);
    return false;
}

static bool canSetFilter(CAN_FILTER_CFG *flt)
{
    CAN_FILTER f = (CAN_FILTER)flt->filterId;
    u8 s = DRV_CANFDSPI_FilterDisable(DRV_CANFDSPI_INDEX_0, f);
    // if (!flt->enabled)
    // {
    //     ESP_LOGI("CAN", "Disable Filter%d\t", flt->filterId);
    //     return true;
    // }
    s += DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_0, f, &flt->fObj);
    s += DRV_CANFDSPI_FilterMaskConfigure(DRV_CANFDSPI_INDEX_0, f, &flt->mObj);
    s += DRV_CANFDSPI_FilterToFifoLink(DRV_CANFDSPI_INDEX_0, f, APP_RX_FIFO, flt->enabled);

    return !s;
}

static bool canSetCanChl(u8 *channel)
{
    return canSwitchChannel(channel[0]);
}

#endif // !__CAN_C__