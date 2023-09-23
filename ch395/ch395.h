/**
 ****************************************************************************************************
 * @file        ch395q.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-6-17
 * @brief       CH395 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20200417
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __CH395_H
#define __CH395_H
#include "ch395inc.h"
#include "spi.h"
#include "ch395cmd.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "driver/gpio.h"


/******************************************************************************************/
/* 引脚 定义 */

// #define CH395_SCS_GPIO_PORT                  GPIOG
// #define CH395_SCS_GPIO_PIN                   10
// #define CH395_SCS_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)            /* PG口时钟使能 */

// #define CH395_INT_GPIO_PORT                  GPIOG
// #define CH395_INT_GPIO_PIN                   -1
// #define CH395_INT_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)            /* PG口时钟使能 */

// #define CH395_RST_GPIO_PORT                  GPIOD
// #define CH395_RST_GPIO_PIN                   -1
// #define CH395_RST_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)            /* PD口时钟使能 */

/******************************************************************************************/
#define ch395_scs_low                        gpio_set_level(PIN_CS, 0)   /* SPI片选引脚输出低电平 */
#define ch395_scs_hign                       gpio_set_level(PIN_CS, 1)     /* SPI片选引脚输出高电平 */
#define ch395_sdo_pin                        gpio_get_level(13)                     /* 获取CH395的SPI数据输出引脚电平 */
#define ch395_int_pin_wire                   gpio_get_level(13)                     /* 假定CH395的INT#引脚,如果未连接那么也可以通过查询兼做中断输出的SDO引脚状态实现 */


typedef struct ch395q_socket_t
{
    uint8_t socket_enable;                                          /* Socket使能 */
    uint8_t socket_index;                                           /* Socket标号 */
    uint8_t proto;                                                  /* Socket协议 */
    uint8_t des_ip[4];                                              /* 目的IP地址 */
    uint16_t des_port;                                              /* 目的端口 */
    uint16_t sour_port;                                             /* 源端口 */
    
    struct
    {
        uint8_t *buf;                                               /* 缓冲空间 */
        uint32_t size;                                              /* 缓冲空间大小 */
    } send;                                                         /* 发送缓冲 */
    
    struct
    {
        uint8_t recv_flag;                                          /* 接收数据标志位 */
        uint8_t *buf;                                               /* 缓冲空间 */
        uint32_t size;                                              /* 缓冲空间大小 */
    } recv;                                                         /* 接收缓冲 */                                                       /* 接收缓冲 */
    
    struct
    {
        uint8_t ip[4];                                              /* IP地址 */
        uint8_t gwip[4];                                            /* 网关IP地址 */
        uint8_t mask[4];                                            /* 子网掩码 */
        uint8_t dns1[4];                                            /* DNS服务器1地址 */
        uint8_t dns2[4];                                            /* DNS服务器2地址 */
    } net_info;                                                     /* 网络信息 */
    
    struct
    {
        uint8_t ipaddr[4];                                          /* IP地址 32bit*/
        uint8_t gwipaddr[4];                                        /* 网关地址 32bit*/
        uint8_t maskaddr[4];                                        /* 子网掩码 32bit*/
        uint8_t macaddr[6];                                         /* MAC地址 48bit*/
    } net_config;                                                   /* 网络配置信息 */

} ch395_socket;

/* DHCP状态 */
enum DHCP
{
    DHCP_UP = 0,                                                    /* DHCP获取成功状态 */
    DHCP_DOWN,                                                      /* DHCP获取失败状态 */
    DHCP_STA,                                                       /* DHCP开启状态 */
};

struct ch395q_t
{
    uint8_t version;                                                /* 版本信息 */
    uint8_t phy_status;                                             /* PHY状态 */
    uint8_t dhcp_status;                                            /* DHCP状态 */
    uint8_t  ipinf_buf[20];                                         /* 获取IP信息 */
    
    struct
    {
        ch395_socket config;                                        /* 配置信息 */
    } socket[8];                                                    /* Socket状态 */
    
    void (*ch395_error)(uint8_t i);                                 /* ch395q错误检测函数 */
    void (*ch395_phy_cb)(uint8_t phy_status);                       /* ch395q phy状态回调函数 */
    void (*ch395_reconnection)(void);                               /* ch395q 重新连接函数 */
};

extern struct ch395q_t g_ch395q_sta;

/* CH395Q模块Socket标号定义 */
#define CH395Q_SOCKET_0             0                               /* Socket 0 */
#define CH395Q_SOCKET_1             1                               /* Socket 1 */
#define CH395Q_SOCKET_2             2                               /* Socket 2 */
#define CH395Q_SOCKET_3             3                               /* Socket 3 */
#define CH395Q_SOCKET_4             4                               /* Socket 4 */
#define CH395Q_SOCKET_5             5                               /* Socket 5 */
#define CH395Q_SOCKET_6             6                               /* Socket 6 */
#define CH395Q_SOCKET_7             7                               /* Socket 7 */

/* 使能定义 */
#define CH395Q_DISABLE              1                               /* 禁用 */
#define CH395Q_ENABLE               2                               /* 使能 */

/* CH395Q模块Socket协议类型定义 */
#define CH395Q_SOCKET_UDP           0                               /* UDP */
#define CH395Q_SOCKET_TCP_CLIENT    1                               /* TCP客户端 */
#define CH395Q_SOCKET_TCP_SERVER    2                               /* TCP服务器 */
#define CH395Q_SOCKET_MAC_RAW       3                               /* MAC_RAW */

#define DEF_KEEP_LIVE_IDLE          (15*1000)                       /* 空闲时间 */
#define DEF_KEEP_LIVE_PERIOD        (15*1000)                       /* 间隔为15秒，发送一次KEEPLIVE数据包 */
#define DEF_KEEP_LIVE_CNT           200

uint8_t  ch395_read_data(void ) ;
void ch395_write_cmd( uint8_t mcmd );
void ch395_write_data( uint8_t mdata );
void ch395q_handler(void* arg);
void ch395_interrupt_handler(void);
void ch395_hardware_init(void);
uint8_t ch395q_socket_config(ch395_socket * ch395_sokect);
void ch395_reconnection(void);

#endif
