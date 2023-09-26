#include "uds.h"
#include "stdint.h"
#include "stdbool.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "freertos/queue.h"
#include "can.h"
#include "msg.h"

#define TX_UDSMSGNUM 100

#define UDSSEND ("uds send %s")
#define UDSECU ("uds ecu %s")
#define UDSRECV ("uds recv %s")
#define UDSSID ("uds sid %s")
#define UDSREQ ("uds req %s")

#define ERRMSGFMT(__fmt__, s) sprintf((result.errorMessage), __fmt__, (s))
#define ERRMSG(s) sprintf((result.errorMessage), (s))

#define null NULL

QueueHandle_t txUDSFifo;
u8 *udsBuffer;
StaticQueue_t udsStaticQueue;

extern TaskHandle_t canRxHandle;
extern TaskHandle_t wifiSendHandle;
extern TaskHandle_t udsSendHandle;
extern QueueHandle_t rxMsgFifo;
extern QueueHandle_t txMsgFifo;
extern void can_clearFilter();
extern bool (*canSetFilterPtr)(CAN_FILTER_CFG *flt);
extern bool (*canQueueUDSPtr)(CAN_CMD_UDSFRAME *msg);
extern void (*canSend2Wifi)(uint8_t *data, int len);

u32 send, recv;
u32 funcId = 0x7df;
CAN_CMD_FRAME cmd;
CAN_MSG_FRAME txMsg;
CAN_CMD_UDSFRAME txUdsMsg;
CAN_MSG_UDSFRAME rxUdsMsg;

TaskHandle_t udsTaskHandle;

UDSResult do_udsRequest(CAN_CMD_UDSFRAME *request, CAN_MSG_UDSFRAME *response);
bool logAndSendFrame(FrameData *frame);
static __inline void delay_clock(int ts)
{
    uint32_t start, curr;

    __asm__ __volatile__("rsr %0, ccount"
                         : "=r"(start));
    do
    {
        __asm__ __volatile__("rsr %0, ccount"
                             : "=r"(curr));

    } while (curr - start <= ts);
}

static void delay_us(int us)
{
    while (us--)
    {
        delay_clock(esp_clk_cpu_freq() / 1000000);
    }
}
/// @brief
/// @param stminbyte STmin byte in the UDS frame
/// @return STmin in us, or 1/1000 ms
int getSTmin(u8 stminbyte)
{
    if (stminbyte <= 0x7F)
        // stminbyte ms
        return stminbyte * 1000;
    else if (stminbyte >= 0xF1 && stminbyte <= 0xF9)
        //((stminbyte - 0xF0) * 1000) * 0.1us
        return (stminbyte - 0xF0) * 100;
    return 20 * 1000;
}
/// @brief
/// @param frameData
/// @param len if the frameData is a FF(ie. First Frame), returns length of the actual long frame
/// @return
bool isFirstFrame(FrameData *frameData, u16 *len)
{
    if (frameData->data != null && frameData->dlc > 1)
    {
        u8 first = frameData->data[0];
        bool is1st = (first & 0xF0) == 0x10;
        if (is1st)
            *len = ((first & 0x0F) << 8) + frameData->data[1];
        else
            *len = -1;
        return is1st;
    }
    *len = -1;
    return false;
}
bool isNRC78Frame(FrameData *frameData)
{
    if (frameData->data != null && frameData->dlc >= 4)
        return frameData->data[0] == 0x03 && frameData->data[1] == 0x7f && frameData->data[3] == 0x78;
    return false;
}

u8 getDataOffset(u8 sid)
{
    switch (sid)
    {
        // clang-format off
        case 0x01: case 0x41: return 2;
        case 0x03: case 0x43: return 2;
        case 0x09: case 0x49: return 3;
        case 0x10: case 0x50: return 2;
        case 0x11: case 0x51: return 2;
        case 0x27: case 0x67: return 2;
        case 0x28: case 0x68: return 2;
        case 0x29: case 0x69: return 2;
        case 0x3E: case 0x7E: return 1;
        case 0x83: case 0xC3: return 2;
        case 0x84: case 0xC4: return 2;
        case 0x85: case 0xC5: return 2;
        case 0x86: case 0xC6: return 2;
        case 0x87: case 0xC7: return 2;
        case 0x22: case 0x62: return 3;
        case 0x23: case 0x63: return 2;
        case 0x24: case 0x64: return 2;
        case 0x2A: case 0x6A: return 2;
        case 0x2C: case 0x6C: return 2;
        case 0x2E: case 0x6E: return 3;
        case 0x3D: case 0x7D: return 2;
        case 0x14: case 0x54: return 1;
        case 0x19: case 0x59: return 3;
        case 0x2F: case 0x6F: return 3;
        case 0x31: case 0x71: return 4;
        case 0x34: case 0x74: return 4;
        case 0x35: case 0x75: return 4;
        case 0x36: case 0x76: return 4;
        case 0x37: case 0x77: return 4;
        case 0x38: case 0x78: return 4;
        default:              return 1;
        // clang-format on
    }
}
/// @brief
/// @param udsSid the UDS request SID
/// @param frame the CAN frame to be parsed
/// @param resultMsg data returned by ECU, if any
/// @param result UDS request result
void parseSingleFrame(int udsSid, FrameData *frame, CAN_MSG_UDSFRAME *resultMsg, UDSResult *result)
{
    if (udsSid > 0xBF)
    {
        ESP_LOGE("UDS", "parseSingleFrame: request sid wrong: %2x", udsSid);
        return;
    }
    if (frame->data[1] == 0x7F)
    {
        result->nrc = frame->data[3];
        sprintf((char *)&result->errorMessage, "NRC:%2x", result->nrc);
        resultMsg->dataLen = 0;
        return;
    }
    if (udsSid + 0x40 != frame->data[1])
    {
        sprintf((char *)&result->errorMessage, "wrong sid:%2x, expected:%2x", frame->data[1], udsSid + 0x40);
        resultMsg->dataLen = 0;
        return;
    }
    result->success = true;
    result->errorMessage[0] = '\0';
    int offet = getDataOffset(udsSid);
    // 06 27 02 01 02 03 04
    // 02 67 02
    // offset=2, udsLen=2
    u8 udsLen = frame->data[0];
    resultMsg->dataLen = udsLen - offet;
    for (u8 i = offet + 1; i < frame->dlc; i++)
    {
        resultMsg->data[i - (offet + 1)] = frame->data[i];
    }
}

FrameData logAndTakeFrameData(CAN_MSG_FRAME *frame)
{
    FrameData array;
    array.dlc = frame->rxObj.bF.ctrl.DLC;
    array.data = &frame->data;

    BaseType_t receiveTaskWoken = pdFALSE;
    if (!xQueueSendFromISR(rxMsgFifo, (void *)&frame, &receiveTaskWoken))
    {
        report("can recv fifo full, discard");
    }
    else
    {
        xTaskNotifyGive(wifiSendHandle);
        if (receiveTaskWoken)
        {
            portYIELD_FROM_ISR();
        }
    }
    return array;
}

bool logAndSendFrame(FrameData *frame)
{
    cmd.txObj.bF.ctrl.DLC = frame->dlc;
    for (u8 i = frame->dlc - 1; i >= 0; i--)
        cmd.data[i] = frame->data[i];
    if (xQueueSend(txMsgFifo, &cmd, 0))
    {
        //$"{DateTime.Now:HH:mm:ss.fff}\t{vci.SendAddress:X2}\t{frame.ByteArrToHexString()}".DebugLogToFile();
        txMsg.rxObj.bF.timeStamp = xTaskGetTickCount();
        if (!xQueueSend(rxMsgFifo, &txMsg, 0))
        {
            report("uds up fifo full");
        }
        else
            xTaskNotifyGive(wifiSendHandle);
        return true;
    }
    else
    {
        report("can tx fifo full");
        return false;
    }
}
bool setAndFilterSendRecvAddress(CAN_CMD_UDSFRAME *request)
{
    can_clearFilter();
    CAN_FILTER_CFG f = {
        .enabled = 1,
        .fObj.SID = request->txObj.bF.id.SID,
        .fObj.SID11 = request->txObj.bF.id.SID11,
        .fObj.EID = request->txObj.bF.id.EID,
        .fObj.EXIDE = request->txObj.bF.ctrl.IDE,

        .mObj.MSID = 0x7ff,
        .mObj.MSID11 = 1,
        .mObj.MIDE = 1,
        .filterId = 0,
    };
    bool s = canSetFilterPtr(&f);
    f.filterId = 1;
    f.fObj.SID = request->rxObj.bF.id.SID,
    f.fObj.SID11 = request->rxObj.bF.id.SID11,
    f.fObj.EID = request->rxObj.bF.id.EID,
    f.fObj.EXIDE = request->rxObj.bF.ctrl.IDE,
    s = s && canSetFilterPtr(&f);
    if (!s)
        report("uds flt fail");
    return s;
}

UDSResult udsRequest(CAN_CMD_UDSFRAME *request, CAN_MSG_UDSFRAME *response)
{
    canRxHandle = udsSendHandle;
    do_udsRequest(request, response);
    canRxHandle = wifiSendHandle;
}
UDSResult do_udsRequest(CAN_CMD_UDSFRAME *request, CAN_MSG_UDSFRAME *response)
{
    UDSResult result = {.success = false, .errorMessage = *response->errorMessage};
    bool error = false;
    bool responseComplete = false;
    u16 P2CAN_Client = 500;
    u16 requestDataLength = request->dataLen;

    response->dataLen = 0;

    if (request->data == null || !request->dataLen)
    {
        report("uds req empty");
        return result;
    }

    int sid = request->data[0];
    if (sid + 0x40 > 0xC7)
    {
        report("uds sid wrong");
        return result;
    }

    //setAndFilterSendRecvAddress(request);
    // 多帧传输
    if (request->dataLen > 7)
    {
        u8 *requestData = &request->data;

        u16 first2Bytes = 0x1000 | request->dataLen;
        u8 data[8];
        // temp value, one frame
        FrameData frame = {.data = &data};

        frame.data[0] = (u8)((first2Bytes & 0xFF00) >> 8);
        frame.data[1] = (u8)(first2Bytes & 0x00FF);
        for (u8 i = 0; i < 6; frame.data[i + 2] = requestData[i++])
            ;

        u8 *dataCurrentPos = requestData[6 + 1];
        // 发送首帧
        if (LogAndSendFrame(frame))
        {
            // sent frames count
            u32 sent = 6;
            // time in ms when send operation completes
            u32 sendCompleted = 0;
            CAN_MSG_FRAME msg;
            // timeout value between each 2 frames, can be set larger if NRC78 is received
            u16 timeout = P2CAN_Client;
            while (xQueueReceive(rxMsgFifo, &msg, pdMS_TO_TICKS(timeout)))
            {
                timeout = P2CAN_Client;
                FrameData f = logAndTakeFrameData(&msg);
                if (sendCompleted == 0)
                {
                    if (f.dlc == 0)
                        continue;
                    u8 first = f.data[0];
                    if (first == 0x30)
                    {
                        // max block serial
                        u8 BS = f.data[1];
                        // interval between continuous frames
                        u32 STmin = getSTmin(f.data[2]);
                        // frame serial
                        u8 SN = 1;
                        // block serial
                        u8 BN = 0;
                        // current frame data len
                        u8 frameDataLen;
                        for (; sent < requestDataLength; sent += frameDataLen)
                        {
                            if (STmin > 1000)
                                vTaskDelay(pdMS_TO_TICKS(STmin / 1000));
                            else
                                delay_us(STmin);

                            frameDataLen = requestDataLength - sent;
                            if (frameDataLen > 7)
                                frameDataLen = 7;

                            frame.data[0] = 0x20 + SN;
                            for (u8 i = 0; i < 7; i++)
                            {
                                frame.data[1 + i] = *(dataCurrentPos++);
                            }

                            if (!LogAndSendFrame(frame))
                            {
                                ERRMSGFMT(UDSSEND, "fail");
                                result.success = false;
                                error = true;
                                break;
                            }
                            if (++SN > 0xF)
                                SN = 1;
                            if (++BN == BS)
                            {
                                BN = 0;
                                break;
                            }
                        }
                        if (sent == requestDataLength)
                        {
                            sendCompleted = pdTICKS_TO_MS(xTaskGetTickCount());
                            continue;
                        }
                    }
                    else if (first == 0x31)
                        continue;
                    else if (first == 0x32)
                    {
                        ERRMSGFMT(UDSECU, "ovrflw");
                        result.success = false;
                        error = true;
                        break;
                    }
                }
                else
                {
                    if (f.dlc != 0)
                    {
                        parseSingleFrame(sid, &frame, response, &result);
                        if (result.nrc == 0x78)
                        {
                            timeout = 3300;
                            sendCompleted += 3300;
                            continue;
                        }
                        break;
                    }
                    else if (pdTICKS_TO_MS(xTaskGetTickCount()) - sendCompleted >= P2CAN_Client)
                    {
                        result.success = false;
                        ERRMSGFMT(UDSSEND, "timout");
                        break;
                    }
                }
            }
        }
        else
        {
            ERRMSGFMT(UDSSEND, "fail");
            result.success = false;
            error = true;
        }
    }
    // 发送单帧
    else
    {
        // var bytes = new byte[8];
        // var content = request.ToArray();
        // bytes[0] = (byte)content.Length;
        // Array.Copy(content, 0, bytes, 1, content.Length);

        u8 data[8];
        // temp value, one frame
        FrameData frame = {.data = &data};
        frame.data[0] = request->dataLen;
        memcpy(frame.data + 1, request->data, request->dataLen);

        // 发送一帧
        if (LogAndSendFrame(frame))
        {
            result.success = false;
            bool isECUBusy = true;
            bool isLongMessage = false;
            u16 totalLen = 0;
            u16 totalFrames = 0;
            u16 received = 0;
            u8 receivedFrames = 0;
            u8 SN = 0;

            CAN_MSG_FRAME msg;
            // timeout value between each 2 frames, can be set larger if NRC78 is received
            u16 timeout = P2CAN_Client;
            while (xQueueReceive(rxMsgFifo, &msg, pdMS_TO_TICKS(timeout)))
            {
                timeout = P2CAN_Client;
                FrameData recvFrame = logAndTakeFrameData(&msg);
                if (isNRC78Frame(&recvFrame))
                {
                    timeout = 3300;
                    if (isECUBusy)
                        continue;
                    else
                    {
                        ERRMSGFMT(UDSECU, "rspns err");
                        result.success = false;
                        error = true;
                        break;
                    }
                }
                else
                    isECUBusy = false;
                // 收到首帧，或收到单帧
                if (!isLongMessage)
                {
                    // 收到首帧
                    if (isFirstFrame(&recvFrame, &totalLen))
                    {
                        totalFrames = (totalLen - 6) / 7 + 1;
                        if ((totalLen - 6) % 7 != 0)
                            totalFrames++;
                        receivedFrames = 1;
                        isLongMessage = true;
                        received = recvFrame.dlc - (2 + getDataOffset(sid));
                        memcpy(response->data, recvFrame.data, received);

                        // 发送流控帧
                        vTaskDelay(pdMS_TO_TICKS(5));
                        frame.dlc = 8;
                        frame.data[0] = 0x30;
                        memset(frame.data + 1, 0, 7);
                        if (logAndSendFrame(&frame))
                        {
                            SN = 1;
                            timeout = 1000;
                            continue;
                        }
                        else
                        {
                            ERRMSGFMT(UDSSEND, "fc fail");
                            result.success = false;
                            error = true;
                            break;
                        }
                    }
                    // 收到单帧答复
                    else
                    {
                        parseSingleFrame(sid, &recvFrame, response, &result);
                        break;
                    }
                }
                // 接收连续帧
                else
                {
                    u8 first = recvFrame.data[0];
                    // 校验连续帧的序列号 0x21...
                    if ((first & 0xF0) == 0x20 && (first & 0x0F) == SN)
                    {
                        SN++;
                        if (SN == 0x10)
                            SN = 0x01;
                        // 将收到的一帧数据，添加到总结果中
                        memcpy(&response->data[received], &recvFrame.data[1], recvFrame.dlc - 1);
                        received += recvFrame.dlc - 1;
                        receivedFrames++;
                        if (receivedFrames == totalFrames)
                        {
                            result.success = true;
                            result.errorMessage[0] = '\0';
                            error = false;
                            response->dataLen = received;
                            break;
                        }
                        if (received - totalLen >= 7)
                        {
                            ERRMSGFMT(UDSECU, "msg len err");
                            result.success = false;
                            error = true;
                            break;
                        }
                        continue;
                    }
                    else
                    {
                        ERRMSGFMT(UDSECU, "msg SN err");
                        result.success = false;
                        error = true;
                        break;
                    }
                }
            }
        }
        else
        {
            ERRMSGFMT(UDSSEND, "fail");
            error = true;
        }
    }
    return result;
}

static void uds_task()
{
    while (1)
    {
        if (!xQueueReceive(txUDSFifo, &txUdsMsg, portMAX_DELAY))
            continue;
        UDSResult udsRes = udsRequest(&txUdsMsg, &rxUdsMsg);

        rxUdsMsg.success = udsRes.success;
        rxUdsMsg.nrc = udsRes.nrc;
        canSend2Wifi(&txUdsMsg, sizeof(CAN_MSG_UDSFRAME) - (4096 - txUdsMsg.dataLen));
    }
}

static bool uds_enqueue_msg(CAN_CMD_UDSFRAME *msg)
{
    BaseType_t postTaskWoken = pdFALSE;
    if (!xQueueSend(txUDSFifo, (void *)msg, 2))
    {
        report("can tx fifo full, discard");
        return false;
    }
    return true;
}

void uds_init()
{
    udsBuffer = (uint8_t *)heap_caps_malloc(TX_UDSMSGNUM * sizeof(CAN_CMD_UDSFRAME), MALLOC_CAP_SPIRAM | MALLOC_CAP_DEFAULT);
    txUDSFifo = xQueueCreateStatic(TX_UDSMSGNUM, sizeof(CAN_CMD_UDSFRAME), udsBuffer, &udsStaticQueue);

    xTaskCreatePinnedToCore(uds_task, "uds task", 4096, NULL, 10, &udsTaskHandle, 1);

    canQueueUDSPtr = uds_enqueue_msg;
}