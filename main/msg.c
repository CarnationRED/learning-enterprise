#include "msg.h"
#include "can.h"
#include "mqtt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "msg.h"
// #include "can.h"
// #include "freertos/queue.h"

// extern QueueHandle_t txMultiMsgFifo;

MSG_STATS msgStats;
int recvedMsgs = 0;
extern CAN_STATS canStats;
typedef struct _ReportMsg
{
    u8 isResponse;      // 1
    u8 responseSuccess; // 1
    u8 reserved0;       // 1
    u8 reserved1;       // 1

    DataDownMsgType responseType; // 4
    MSG_STATS msgStats;           // 4
    CAN_STATS canStats;           // 4
    char msg[32];                 // 33
} ReportMsg;

u8 rptBuffer[sizeof(ReportMsg) + 1];
ReportMsg *resportMsgInit()
{
    ReportMsg *msg = (ReportMsg *)(((void *)&rptBuffer) + 1);
    rptBuffer[0] = 0xff;
    msg->msgStats = msgStats;
    msg->canStats = canStats;
    msg->responseType = 0xff;
    msg->msg[32] = '\0';
    return msg;
}
/// @brief can't reference canSendOneFrame here, so canSendPtr is defined to solve this
/// @param msg
/// @return
bool (*canQueuePtr)(CAN_CMD_FRAME *msg);
bool (*canQueueMultiPtr)(CAN_CMD_MULTIFRAME *msgs);
CAN_CMD_UDSFRAME *(*canQueueUDSPtr)(u8 *msg, u16 len);
bool (*canSetFilterPtr)(CAN_FILTER_CFG *flt);
bool (*canSetCanChlPtr)(u8 *channel);
extern void (*dataDownHandler)(int len, uint8_t *data);
extern void (*dataCtrlHandler)(int len, uint8_t *data);
static CAN_CMD_MULTIFRAME *frames;
static CAN_CMD_UDSFRAME *udsf;
static void dataDown(int len, u8 *data)
{

    if (len >= sizeof(DataDownMsg))
    {
        uint8_t *ptr = data;
        DataDownMsg *head = (DataDownMsg *)ptr;
        if (head != NULL)
        {
            switch (head->type)
            {
            case SINGLE_FRAME:
                if (head->dataLen == head->elementSize * head->elementCount)
                {
                    if (canQueuePtr != NULL)
                    {
                        canQueuePtr((CAN_CMD_FRAME *)(ptr + sizeof(DataDownMsg)));
                        recvedMsgs++;
                    }
                    // canSendOneFrame((CAN_CMD_FRAME *)ptr);
                }
                else
                    msgStats = MSG_DATALEN_MISMATCH;
                break;
            case MULTI_FRAMES:
                if (head->dataLen == head->elementSize * head->elementCount)
                {
                    if (canQueueMultiPtr != NULL)
                    {
                        // volatile int t = xTaskGetTickCount();
                        // CAN_CMD_FRAME msg;
                        frames = (CAN_CMD_MULTIFRAME *)(ptr + sizeof(DataDownMsg));
                        // u8 dlc = frames->txObj.bF.ctrl.DLC;
                        // msg.txObj = frames->txObj;
                        // msg.channel = frames->channel;
                        // u8 *d = frames->data;
                        // for (size_t i = 0; i < frames->frames; i++)
                        // {
                        //     memcpy(msg.data, d, dlc);
                        //     canQueuePtr(&msg);
                        //     d += dlc;
                        // }
                        canQueueMultiPtr(frames);
                        recvedMsgs += frames->frames;
                        // t = xTaskGetTickCount() - t;
                        // msg.channel = t;
                    }
                    // canSendOneFrame((CAN_CMD_FRAME *)ptr);
                }
                else
                    msgStats = MSG_DATALEN_MISMATCH;
                break;
            case UDS_FRAMES:
                if (head->dataLen == head->elementSize * head->elementCount)
                {
                    if (canQueueUDSPtr != NULL)
                    {
                        udsf = canQueueUDSPtr((u8 *)(ptr + sizeof(DataDownMsg)), len - sizeof(DataDownMsg));
                        if (!udsf)
                        {
                            msgStats = MSG_DATA_FORMAT_ERROR;
                        }
                        else
                        {
                            u8 lenPerFrame = (udsf->txObj.bF.ctrl.DLC - 1);
                            if (udsf->dataLen > lenPerFrame)
                            {
                                u16 continuousDataLen = udsf->dataLen - udsf->txObj.bF.ctrl.DLC;
                                u16 continuousFrames = continuousDataLen / lenPerFrame;
                                if (continuousFrames * lenPerFrame < continuousDataLen)
                                    continuousFrames++;
                                recvedMsgs += 1 + continuousFrames;
                            }
                            else
                                recvedMsgs += 1;
                        }
                    }
                    // canSendOneFrame((CAN_CMD_FRAME *)ptr);
                }
                else
                    msgStats = MSG_DATALEN_MISMATCH;
                break;
            default:
                msgStats = MSG_DATA_FORMAT_ERROR;
                break;
            }
        }
    }
}
void dataCtrl(int len, u8 *data)
{
    if (len >= sizeof(DataDownMsg))
    {
        uint8_t *ptr = data;
        DataDownMsg *head = (DataDownMsg *)ptr;
        bool success = false;
        ReportMsg *msg = resportMsgInit();
        msg->isResponse = true;
        if (head != NULL)
        {
            switch (head->type)
            {
            case SET_FILTER:
                if (head->dataLen == head->elementSize * head->elementCount)
                {
                    if (canSetFilterPtr != NULL)
                    {
                        success = true;
                        for (size_t i = 0; i < head->elementCount; i++)
                        {
                            success &= canSetFilterPtr((CAN_FILTER_CFG *)(ptr + sizeof(DataDownMsg) + i * head->elementSize));
                        }
                    }
                    // canSendOneFrame((CAN_CMD_FRAME *)ptr);
                }
                else
                    msgStats = MSG_DATALEN_MISMATCH;
                msg->responseType = SET_FILTER;
                break;
            case SET_CANCHL:
                if (head->dataLen == head->elementSize * head->elementCount)
                {
                    if (canSetCanChlPtr != NULL)
                    {
                        success = true;
                        success &= canSetCanChlPtr((u8 *)(ptr + sizeof(DataDownMsg)));
                    }
                    // canSendOneFrame((CAN_CMD_FRAME *)ptr);
                }
                else
                    msgStats = MSG_DATALEN_MISMATCH;
                msg->responseType = SET_CANCHL;
                break;
            case KEEP_ALIVE:
                mqtt_report_pulish("pingok", 0);
                return;
            default:
                msgStats = MSG_DATA_FORMAT_ERROR;
                break;
            }
            msg->responseSuccess = success;
            mqtt_report_pulish((char *)rptBuffer, sizeof(ReportMsg) + 1);
        }
    }
}
void report(char *msg)
{
    ReportMsg *m = resportMsgInit();
    u8 len = strlen(msg);
    if (len > 31)
        len = 31;
    memcpy(m->msg, msg, len);
    mqtt_report_pulish((char *)rptBuffer, sizeof(ReportMsg) + 1);
}
void msg_init()
{
    dataDownHandler = dataDown;
    dataCtrlHandler = dataCtrl;
}