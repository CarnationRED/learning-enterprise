#include "msg.h"
#include "can.h"
#include "mqtt.h"

MSG_STATS msgStats;
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
} ReportMsg;

u8 rptBuffer[sizeof(ReportMsg) + 1];
ReportMsg *resportMsgInit()
{
    ReportMsg *msg = (ReportMsg *)(((void *)&rptBuffer) + 1);
    rptBuffer[0] = 0xff;
    msg->msgStats = msgStats;
    msg->canStats = canStats;
    msg->responseType = 0xff;
    return msg;
}
/// @brief can't reference canSendOneFrame here, so canSendPtr is defined to solve this
/// @param msg
/// @return
bool (*canSendPtr)(CAN_CMD_FRAME *msg);
bool (*canSetFilterPtr)(CAN_FILTER_CFG *flt);
extern void (*dataDownHandler)(int len, uint8_t *data);
extern void (*dataCtrlHandler)(int len, uint8_t *data);
void dataDown(int len, u8 *data)
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
                    if (canSendPtr != NULL)
                        canSendPtr((CAN_CMD_FRAME *)(ptr + sizeof(DataDownMsg)));
                    // canSendOneFrame((CAN_CMD_FRAME *)ptr);
                }
                else
                    msgStats = MSG_DATALEN_MISMATCH;
                break;
            case MULTI_FRAMES:
                if (head->dataLen == head->elementSize * head->elementCount)
                {
                    if (canSendPtr != NULL)
                        for (size_t i = 0; i < head->elementCount; i++)
                        {
                            canSendPtr((CAN_CMD_FRAME *)(ptr + sizeof(DataDownMsg)+ i * head->elementSize));
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

            default:
                msgStats = MSG_DATA_FORMAT_ERROR;
                break;
            }
            msg->responseSuccess = success;
            mqtt_report_pulish((char *)rptBuffer, sizeof(ReportMsg) + 1);
        }
    }
}
void msg_init()
{
    dataDownHandler = dataDown;
    dataCtrlHandler = dataCtrl;
}