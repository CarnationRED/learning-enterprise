#ifndef __MSG_H__
#define __MSG_H__
#include "stdint.h"
#include "stdbool.h"
#include "D:\LYSGZS\DfConnector-V2\learning-enterprise\mcp2518\drv_canfdspi_api.h"

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef int64_t s64;
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;

typedef enum  
{
    SINGLE_FRAME=0,
    MULTI_FRAMES=1,
}DataDownMsgType;
/// @brief DownData Structure: [DataDownMsg][DataStruct0][DataStruct1][DataStruct2]...
typedef struct 
{
    DataDownMsgType type;
    u16 dataLen;
    u16 elementCount;
    u16 elementSize;
    u16 serial;
}DataDownMsg;

#if 0

//! CAN TX Message Object Control
typedef struct _CAN_TX_MSGOBJ_CTRL {
    uint32_t DLC : 4;
    uint32_t IDE : 1;
    uint32_t RTR : 1;
    uint32_t BRS : 1;
    uint32_t FDF : 1;
    uint32_t ESI : 1;
#ifdef MCP2517FD
    uint32_t SEQ : 7;
    uint32_t unimplemented1 : 16;
#else
    uint32_t SEQ : 23;
#endif
} CAN_TX_MSGOBJ_CTRL;

//! CAN RX Message Object Control
typedef struct _CAN_RX_MSGOBJ_CTRL {
    uint32_t DLC : 4;
    uint32_t IDE : 1;
    uint32_t RTR : 1;
    uint32_t BRS : 1;
    uint32_t FDF : 1;
    uint32_t ESI : 1;
    uint32_t unimplemented1 : 2;
    uint32_t FilterHit : 5;
    uint32_t unimplemented2 : 16;
} CAN_RX_MSGOBJ_CTRL;

//! CAN Message Object ID
typedef struct _CAN_MSGOBJ_ID {
    uint32_t SID : 11;
    uint32_t EID : 18;
    uint32_t SID11 : 1;
    uint32_t unimplemented1 : 2;
} CAN_MSGOBJ_ID;

//! CAN Message Time Stamp
typedef uint32_t CAN_MSG_TIMESTAMP;
//! CAN TX Message Object
typedef union _CAN_TX_MSGOBJ {

    struct {
        CAN_MSGOBJ_ID id;
        CAN_TX_MSGOBJ_CTRL ctrl;
        CAN_MSG_TIMESTAMP timeStamp;
    } bF;
    uint32_t word[3];
    uint8_t byte[12];
} CAN_TX_MSGOBJ;
#endif 

typedef struct 
{
	CAN_TX_MSGOBJ txObj;
    u8 channel;
	u8 data[64];
}CAN_CMD_FRAME;

typedef struct 
{
	CAN_RX_MSGOBJ rxObj;
    u8 channel;
	u8 data[64];
}CAN_MSG_FRAME;

#endif // !__MSG_H__