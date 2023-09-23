/**
 ****************************************************************************************************
 * @file        ch395cmd.C
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-6-17
 * @brief       ch395命令接口文件
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
 ****************************************************************************************************
 */

#include "ch395inc.h"
#include "ch395cmd.h"
#include "ch395.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_private/esp_clk.h"
#include "stdint.h"

#define delay_ms(ms) vTaskDelay(pdMS_TO_TICKS(ms))

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

/**
 * @brief       复位ch395芯片
 * @param       无
 * @retval      无
 */
void ch395_cmd_reset(void)
{
    ch395_write_cmd(CMD00_RESET_ALL);
    ch395_scs_hign;
}

/**
 * @brief       使ch395进入睡眠状态
 * @param       无
 * @retval      无
 */
void ch395_cmd_sleep(void)
{
    ch395_write_cmd(CMD00_ENTER_SLEEP);
    ch395_scs_hign;
}

/**
 * @brief       获取芯片以及固件版本号，1字节，高四位表示芯片版本，
 * @param       无
 * @retval      1字节芯片及固件版本号
 */
uint8_t ch395_cmd_get_ver(void)
{
    uint8_t i;
    ch395_write_cmd(CMD01_GET_IC_VER);
    i = ch395_read_data();
    ch395_scs_hign;
    return i;
}

/**
 * @brief       测试命令，用于测试硬件以及接口通讯，
 * @param       1字节测试数据
 * @retval      硬件ok，返回 testdata按位取反
 */
uint8_t ch395_cmd_check_exist(uint8_t testdata)
{
    uint8_t i;

    ch395_write_cmd(CMD11_CHECK_EXIST);
    ch395_write_data(testdata);
    i = ch395_read_data();
    ch395_scs_hign;
    return i;
}

/**
 * @brief       设置phy，主要设置ch395 phy为100/10m 或者全双工半双工，ch395默为自动协商。
 * @param       参考phy 命令参数/状态
 * @retval      无
 */
void ch395_cmd_set_phy(uint8_t phystat)
{
    ch395_write_cmd(CMD10_SET_PHY);
    ch395_write_data(phystat);
    ch395_scs_hign;
}

/**
 * @brief       获取phy的状态
 * @param       无
 * @retval      当前ch395phy状态，参考phy参数/状态定义
 */
uint8_t ch395_cmd_get_phy_status(void)
{
    uint8_t i;

    ch395_write_cmd(CMD01_GET_PHY_STATUS);
    i = ch395_read_data();
    ch395_scs_hign;
    return i;
}

/**
 * @brief       获取全局中断状态，收到此命令ch395自动取消中断，0x43及以下版本使用
 * @param       无
 * @retval      返回当前的全局中断状态
 */
uint8_t ch395_cmd_get_glob_int_status(void)
{
    uint8_t init_status;

    ch395_write_cmd(CMD01_GET_GLOB_INT_STATUS);
    init_status = ch395_read_data();
    ch395_scs_hign;
    return init_status;
}

/**
 * @brief       初始化ch395芯片
 * @param       无
 * @retval      返回执行结果
 */
uint8_t ch395_cmd_init(void)
{
    uint8_t i = 0;
    uint8_t s = 0;

    ch395_write_cmd(CMD0W_INIT_CH395);
    ch395_scs_hign;

    while (1)
    {
        delay_ms(10);               /* 延时查询，建议2MS以上 */
        s = ch395_get_cmd_status(); /* 不能过于频繁查询 */

        if (s != CH395_ERR_BUSY)
        {
            break; /* 如果CH395芯片返回忙状态 */
        }

        if (i++ > 200)
        {
            return CH395_ERR_UNKNOW; /* 超时退出,本函数需要500MS以上执行完毕 */
        }
    }

    return s;
}

/**
 * @brief       设置ch395串口波特率，仅在串口模式下有效
 * @param       baudrate 串口波特率
 * @retval      无
 */
void ch395_cmd_set_uart_baud_rate(uint32_t baudrate)
{
    ch395_write_cmd(CMD31_SET_BAUDRATE);
    ch395_write_data((uint8_t)baudrate);
    ch395_write_data((uint8_t)((uint16_t)baudrate >> 8));
    ch395_write_data((uint8_t)(baudrate >> 16));
    uint8_t i = ch395_read_data();
    ch395_scs_hign;
}

/**
 * @brief       获取命令执行状态，某些命令需要等待命令执行结果
 * @param       无
 * @retval      返回上一条命令执行状态
 */
uint8_t ch395_get_cmd_status(void)
{
    uint8_t i;

    ch395_write_cmd(CMD01_GET_CMD_STATUS);
    i = ch395_read_data();
    ch395_scs_hign;
    return i;
}

/**
 * @brief       设置ch395的ip地址
 * @param       ipaddr 指ip地址
 * @retval      无
 */
void ch395_cmd_set_ipaddr(uint8_t *ipaddr)
{
    uint8_t i;

    ch395_write_cmd(CMD40_SET_IP_ADDR);

    for (i = 0; i < 4; i++)
    {
        ch395_write_data(*ipaddr++);
    }

    ch395_scs_hign;
}

/**
 * @brief       设置ch395的网关ip地址
 * @param       ipaddr 指向网关ip地址
 * @retval      无
 */
void ch395_cmd_set_gw_ipaddr(uint8_t *gwipaddr)
{
    uint8_t i;

    ch395_write_cmd(CMD40_SET_GWIP_ADDR);

    for (i = 0; i < 4; i++)
    {
        ch395_write_data(*gwipaddr++);
    }

    ch395_scs_hign;
}

/**
 * @brief       设置ch395的子网掩码，默认为255.255.255.0
 * @param       maskaddr 指子网掩码地址
 * @retval      无
 */
void ch395_cmd_set_maskaddr(uint8_t *maskaddr)
{
    uint8_t i;

    ch395_write_cmd(CMD40_SET_MASK_ADDR);

    for (i = 0; i < 4; i++)
    {
        ch395_write_data(*maskaddr++);
    }

    ch395_scs_hign;
}

/**
 * @brief       设置ch395的mac地址。
 * @param       mcaddr mac地址指针
 * @retval      无
 */
void ch395_cmd_set_macaddr(uint8_t *amcaddr)
{
    uint8_t i;

    ch395_write_cmd(CMD60_SET_MAC_ADDR);

    for (i = 0; i < 6; i++)
    {
        ch395_write_data(*amcaddr++);
    }

    ch395_scs_hign;
    delay_ms(100);
}

/**
 * @brief       获取ch395的mac地址。
 * @param       amcaddr mac地址指针
 * @retval      无
 */
void ch395_cmd_get_macaddr(uint8_t *amcaddr)
{
    uint8_t i;

    ch395_write_cmd(CMD06_GET_MAC_ADDR);

    for (i = 0; i < 6; i++)
    {
        *amcaddr++ = ch395_read_data();
    }

    ch395_scs_hign;
}

/**
 * @brief       设置mac过滤。
 * @param       filtype 参考 mac过滤
 * @param       table0 hash0
 * @param       table1 hash1
 * @retval      无
 */
void ch395_cmd_set_macfilt(uint8_t filtype, uint32_t table0, uint32_t table1)
{
    ch395_write_cmd(CMD90_SET_MAC_FILT);
    ch395_write_data(filtype);
    ch395_write_data((uint8_t)table0);
    ch395_write_data((uint8_t)((uint16_t)table0 >> 8));
    ch395_write_data((uint8_t)(table0 >> 16));
    ch395_write_data((uint8_t)(table0 >> 24));

    ch395_write_data((uint8_t)table1);
    ch395_write_data((uint8_t)((uint16_t)table1 >> 8));
    ch395_write_data((uint8_t)(table1 >> 16));
    ch395_write_data((uint8_t)(table1 >> 24));
    ch395_scs_hign;
}

/**
 * @brief       获取不可达信息 (ip,port,protocol type)
 * @param       list 保存获取到的不可达
     @arg       第1个字节为不可达代码，请参考 不可达代码(ch395inc.h)
     @arg       第2个字节为ip包协议类型
     @arg       第3-4字节为端口号
     @arg       第4-8字节为ip地址
 * @retval      无
 */
void ch395_cmd_get_unreachippt(uint8_t *list)
{
    uint8_t i;

    ch395_write_cmd(CMD08_GET_UNREACH_IPPORT);

    for (i = 0; i < 8; i++)
    {
        *list++ = ch395_read_data();
    }

    ch395_scs_hign;
}

/**
 * @brief       获取远端的ip和端口地址，一般在tcp server模式下使用
 * @param       sockindex socket索引
 * @param       list 保存ip和端口
 * @retval      无
 */
void ch395_cmd_get_remoteipp(uint8_t sockindex, uint8_t *list)
{
    uint8_t i;

    ch395_write_cmd(CMD06_GET_REMOT_IPP_SN);
    ch395_write_data(sockindex);

    for (i = 0; i < 6; i++)
    {
        *list++ = ch395_read_data();
    }

    ch395_scs_hign;
}

/**
 * @brief       设置socket n的目的ip地址
 * @param       sockindex socket索引
 * @param       ipaddr 指向ip地址
 * @retval      无
 */
void ch395_set_socket_desip(uint8_t sockindex, uint8_t *ipaddr)
{
    ch395_write_cmd(CMD50_SET_IP_ADDR_SN);
    ch395_write_data(sockindex);
    ch395_write_data(*ipaddr++);
    ch395_write_data(*ipaddr++);
    ch395_write_data(*ipaddr++);
    ch395_write_data(*ipaddr++);
    ch395_scs_hign;
}

/**
 * @brief       设置socket 的协议类型
 * @param       sockindex socket索引,prottype 协议类型
 * @param       请参考 socket协议类型定义(ch395inc.h)
 * @retval      无
 */
void ch395_set_socket_prot_type(uint8_t sockindex, uint8_t prottype)
{
    ch395_write_cmd(CMD20_SET_PROTO_TYPE_SN);
    ch395_write_data(sockindex);
    ch395_write_data(prottype);
    ch395_scs_hign;
}

/**
 * @brief       设置socket n的协议类型
 * @param       sockindex socket索引
 * @param       desprot 2字节目的端口
 * @retval      无
 */
void ch395_set_socket_desport(uint8_t sockindex, uint16_t desprot)
{
    ch395_write_cmd(CMD30_SET_DES_PORT_SN);
    ch395_write_data(sockindex);
    ch395_write_data((uint8_t)desprot);
    ch395_write_data((uint8_t)(desprot >> 8));
    ch395_scs_hign;
}

/**
 * @brief       设置socket n的协议类型
 * @param       sockindex socket索引
 * @param       desprot 2字节源端口
 * @retval      无
 */
void ch395_set_socket_sourport(uint8_t sockindex, uint16_t surprot)
{
    ch395_write_cmd(CMD30_SET_SOUR_PORT_SN);
    ch395_write_data(sockindex);
    ch395_write_data((uint8_t)surprot);
    ch395_write_data((uint8_t)(surprot >> 8));
    ch395_scs_hign;
}

/**
 * @brief       ip模式下，socket ip包协议字段
 * @param       sockindex socket索引
 * @param       prototype ipraw模式1字节协议字段
 * @retval      无
 */
void ch395_set_socket_ipraw_proto(uint8_t sockindex, uint8_t prototype)
{
    ch395_write_cmd(CMD20_SET_IPRAW_PRO_SN);
    ch395_write_data(sockindex);
    ch395_write_data(prototype);
    ch395_scs_hign;
}

/**
 * @brief       开启/关闭 ping
 * @param       senable :0 / 1, 具体含义如下:
 *   @arg       1:  开启ping
 *   @arg       0:  关闭ping
 * @retval      无
 */
void ch395_enable_ping(uint8_t enable)
{
    ch395_write_cmd(CMD01_PING_ENABLE);
    ch395_write_data(enable);
    ch395_scs_hign;
}

/**
 * @brief       向发送缓冲区写数据
 * @param       sockindex socket索引
 * @param       databuf  数据缓冲区
 * @param       len   长度
 * @retval      无
 */
void ch395_send_data(uint8_t sockindex, uint8_t *databuf, uint16_t len)
{
    uint16_t i;

    ch395_write_cmd(CMD30_WRITE_SEND_BUF_SN);
    ch395_write_data((uint8_t)sockindex);
    ch395_write_data((uint8_t)len);
    ch395_write_data((uint8_t)(len >> 8));

    for (i = 0; i < len; i++)
    {
        ch395_write_data(*databuf++);
    }

    ch395_scs_hign;
}

/**
 * @brief       获取接收缓冲区长度
 * @param       sockindex socket索引
 * @retval      返回接收缓冲区有效长度
 */
uint16_t ch395_get_recv_length(uint8_t sockindex)
{
    uint16_t i;

    ch395_write_cmd(CMD12_GET_RECV_LEN_SN);
    ch395_write_data((uint8_t)sockindex);
    i = ch395_read_data();
    i = (uint16_t)(ch395_read_data() << 8) + i;
    ch395_scs_hign;
    return i;
}

/**
 * @brief       清除接收缓冲区
 * @param       sockindex socket索引
 * @retval      无
 */
void ch395_clear_recv_buf(uint8_t sockindex)
{
    ch395_write_cmd(CMD10_CLEAR_RECV_BUF_SN);
    ch395_write_data((uint8_t)sockindex);
    ch395_scs_hign;
}

/**
 * @brief       读取接收缓冲区数据
 * @param       sockindex socket索引
 * @param       len  长度
 * @param       pbuf  缓冲区
 * @retval      无
 */
void ch395_get_recv_data(uint8_t sockindex, uint16_t len, uint8_t *pbuf)
{
    uint16_t i;

    if (!len)
        return;

    ch395_write_cmd(CMD30_READ_RECV_BUF_SN);
    ch395_write_data(sockindex);
    ch395_write_data((uint8_t)len);
    ch395_write_data((uint8_t)(len >> 8));
    delay_us(1);

    for (i = 0; i < len; i++)
    {
        *pbuf = ch395_read_data();
        pbuf++;
    }

    ch395_scs_hign;
}

/**
 * @brief       设置重试次数
 * @param       count 重试值，最大为20次
 * @retval      无
 */
void ch395_cmd_set_retry_count(uint8_t count)
{
    ch395_write_cmd(CMD10_SET_RETRAN_COUNT);
    ch395_write_data(count);
    ch395_scs_hign;
}

/**
 * @brief       设置重试周期
 * @param       period 重试周期单位为毫秒，最大1000ms
 * @retval      无
 */
void ch395_cmd_set_retry_period(uint16_t period)
{
    ch395_write_cmd(CMD10_SET_RETRAN_COUNT);
    ch395_write_data((uint8_t)period);
    ch395_write_data((uint8_t)(period >> 8));
    ch395_scs_hign;
}

/**
 * @brief       获取socket
 * @param       sockindex socket索引
 * @retval      socket n的状态信息，第1字节为socket 打开或者关闭,第2字节为tcp状态
 */
void ch395_cmd_get_socket_status(uint8_t sockindex, uint8_t *status)
{
    ch395_write_cmd(CMD12_GET_SOCKET_STATUS_SN);
    ch395_write_data(sockindex);
    *status++ = ch395_read_data();
    *status++ = ch395_read_data();
    ch395_scs_hign;
}

/**
 * @brief       打开socket，此命令需要等待执行成功
 * @param       sockindex socket索引
 * @retval      返回执行结果
 */
uint8_t ch395_open_socket(uint8_t sockindex)
{
    uint8_t i = 0;
    uint8_t s = 0;
    ch395_write_cmd(CMD1W_OPEN_SOCKET_SN);
    ch395_write_data(sockindex);
    ch395_scs_hign;

    while (1)
    {
        delay_ms(5);                /* 延时查询，建议2MS以上 */
        s = ch395_get_cmd_status(); /* 不能过于频繁查询 */

        if (s != CH395_ERR_BUSY)
        {
            break; /* 如果CH395芯片返回忙状态 */
        }

        if (i++ > 200)
        {
            return CH395_ERR_UNKNOW; /* 超时退出 */
        }
    }

    return s;
}

/**
 * @brief       关闭socket，
 * @param       sockindex socket索引
 * @retval      返回执行结果
 */
uint8_t ch395_close_socket(uint8_t sockindex)
{
    uint8_t i = 0;
    uint8_t s = 0;
    ch395_write_cmd(CMD1W_CLOSE_SOCKET_SN);
    ch395_write_data(sockindex);
    ch395_scs_hign;

    while (1)
    {
        delay_ms(5);                /* 延时查询，建议2MS以上 */
        s = ch395_get_cmd_status(); /* 不能过于频繁查询 */

        if (s != CH395_ERR_BUSY)
        {
            break; /* 如果CH395芯片返回忙状态 */
        }

        if (i++ > 200)
        {
            return CH395_ERR_UNKNOW; /* 超时退出 */
        }
    }

    return s;
}

/**
 * @brief       tcp连接，仅在tcp模式下有效，此命令需要等待执行成功
 * @param       sockindex socket索引
 * @retval      返回执行结果
 */
uint8_t ch395_tcp_connect(uint8_t sockindex)
{
    uint8_t i = 0;
    uint8_t s = 0;
    ch395_write_cmd(CMD1W_TCP_CONNECT_SN);
    ch395_write_data(sockindex);
    ch395_scs_hign;

    while (1)
    {
        delay_ms(5);                /* 延时查询，建议2MS以上 */
        s = ch395_get_cmd_status(); /* 不能过于频繁查询 */

        if (s != CH395_ERR_BUSY)
        {
            break; /* 如果CH395芯片返回忙状态 */
        }

        if (i++ > 200)
        {
            return CH395_ERR_UNKNOW; /* 超时退出 */
        }
    }

    return s;
}

/**
 * @brief       tcp监听，仅在tcp模式下有效，此命令需要等待执行成功
 * @param       sockindex socket索引
 * @retval      返回执行结果
 */
uint8_t ch395_tcp_listen(uint8_t sockindex)
{
    uint8_t i = 0;
    uint8_t s = 0;
    ch395_write_cmd(CMD1W_TCP_LISTEN_SN);
    ch395_write_data(sockindex);
    ch395_scs_hign;

    while (1)
    {
        delay_ms(5);                /* 延时查询，建议2MS以上 */
        s = ch395_get_cmd_status(); /* 不能过于频繁查询 */

        if (s != CH395_ERR_BUSY)
        {
            break; /* 如果CH395芯片返回忙状态 */
        }

        if (i++ > 200)
        {
            return CH395_ERR_UNKNOW; /* 超时退出 */
        }
    }

    return s;
}

/**
 * @brief       tcp断开，仅在tcp模式下有效，此命令需要等待执行成功
 * @param       sockindex socket索引
 * @retval      无
 */
uint8_t ch395_tcp_disconnect(uint8_t sockindex)
{
    uint8_t i = 0;
    uint8_t s = 0;
    ch395_write_cmd(CMD1W_TCP_DISNCONNECT_SN);
    ch395_write_data(sockindex);
    ch395_scs_hign;

    while (1)
    {
        delay_ms(5);                /* 延时查询，建议2MS以上 */
        s = ch395_get_cmd_status(); /* 不能过于频繁查询 */

        if (s != CH395_ERR_BUSY)
        {
            break; /* 如果CH395芯片返回忙状态 */
        }

        if (i++ > 200)
        {
            return CH395_ERR_UNKNOW; /* 超时退出 */
        }
    }

    return s;
}

/**
 * @brief       获取socket n的中断状态
 * @param       sockindex   socket索引
 * @retval      中断状态
 */
uint8_t ch395_get_socket_int(uint8_t sockindex)
{
    uint8_t intstatus;
    ch395_write_cmd(CMD11_GET_INT_STATUS_SN);
    ch395_write_data(sockindex);
    delay_us(2);
    intstatus = ch395_read_data();
    ch395_scs_hign;
    return intstatus;
}

/**
 * @brief       对多播地址进行crc运算，并取高6位。
 * @param       mac_addr   mac地址
 * @retval      返回crc32的高6位
 */
uint8_t ch395_crcret_6bit(uint8_t *mac_addr)
{
    signed long perbyte;
    signed long perbit;
    const uint32_t poly = 0x04c11db7;
    uint32_t crc_value = 0xffffffff;
    uint8_t c;

    for (perbyte = 0; perbyte < 6; perbyte++)
    {
        c = *(mac_addr++);

        for (perbit = 0; perbit < 8; perbit++)
        {
            crc_value = (crc_value << 1) ^ ((((crc_value >> 31) ^ c) & 0x01) ? poly : 0);
            c >>= 1;
        }
    }

    crc_value = crc_value >> 26;
    return ((uint8_t)crc_value);
}

/**
 * @brief       启动/停止dhcp
 * @param       flag:0 / 1, 具体含义如下:
 *   @arg       1:启动dhcp
 *   @arg       0：停止dhcp
 * @retval      执行状态
 */
uint8_t ch395_dhcp_enable(uint8_t flag)
{
    uint8_t i = 0;
    uint8_t s;
    ch395_write_cmd(CMD10_DHCP_ENABLE);
    ch395_write_data(flag);
    ch395_scs_hign;

    while (1)
    {
        delay_ms(20);
        s = ch395_get_cmd_status(); /* 不能过于频繁查询 */

        if (s != CH395_ERR_BUSY)
        {
            break; /* 如果ch395芯片返回忙状态 */
        }

        if (i++ > 200)
        {
            return CH395_ERR_UNKNOW; /* 超时退出 */
        }
    }

    return s;
}

/**
 * @brief       获取dhcp状态
 * @param       无
 * @retval      dhcp状态，0为成功，其他值表示错误
 */
uint8_t ch395_get_dhcp_status(void)
{
    uint8_t status;
    ch395_write_cmd(CMD01_GET_DHCP_STATUS);
    status = ch395_read_data();
    ch395_scs_hign;
    return status;
}

/**
 * @brief       获取ip，子网掩码和网关地址
 * @param       sockindex socket索引
 * @retval      12个字节的ip,子网掩码和网关地址
 */
void ch395_get_ipinf(uint8_t *addr)
{
    uint8_t i;
    ch395_write_cmd(CMD014_GET_IP_INF);

    for (i = 0; i < 20; i++)
    {
        *addr++ = ch395_read_data();
    }

    ch395_scs_hign;
}

/**
 * @brief       写gpio寄存器
 * @param       regadd   寄存器地址
 * @param       regval   寄存器值
 * @retval      无
 */
void ch395_write_gpio_addr(uint8_t regadd, uint8_t regval)
{
    ch395_write_cmd(CMD20_WRITE_GPIO_REG);
    ch395_write_data(regadd);
    ch395_write_data(regval);
}

/**
 * @brief       读gpio寄存器
 * @param       regadd   寄存器地址
 * @retval      寄存器的值
 */
uint8_t ch395_read_gpio_addr(uint8_t regadd)
{
    uint8_t i;
    ch395_write_cmd(CMD10_READ_GPIO_REG);
    ch395_write_data(regadd);
    delay_ms(1);
    i = ch395_read_data();
    return i;
}

/**
 * @brief       擦除eeprom
 * @param       无
 * @retval      执行状态
 */
uint8_t ch395_eeprom_erase(void)
{
    uint8_t i;
    ch395_write_cmd(CMD00_EEPROM_ERASE);

    while (1)
    {
        delay_ms(20);
        i = ch395_get_cmd_status();

        if (i == CH395_ERR_BUSY)
        {
            continue;
        }

        break;
    }

    return i;
}

/**
 * @brief       写eeprom
 * @param       eepaddr  eeprom地址
 * @param       buf      缓冲区地址
 * @param       len      长度
 * @retval      无
 */
uint8_t ch395_eeprom_write(uint16_t eepaddr, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    ch395_write_cmd(CMD30_EEPROM_WRITE);
    ch395_write_data((uint8_t)(eepaddr));
    ch395_write_data((uint8_t)(eepaddr >> 8));
    ch395_write_data(len);

    while (len--)
        ch395_write_data(*buf++);

    while (1)
    {
        delay_ms(20);
        i = ch395_get_cmd_status();

        if (i == CH395_ERR_BUSY)
        {
            continue;
        }

        break;
    }

    return i;
}

/**
 * @brief       写eeprom
 * @param       eepaddr  eeprom地址
 * @param       buf      缓冲区地址
 * @param       len      长度
 * @retval      无
 */
void ch395_eeprom_read(uint16_t eepaddr, uint8_t *buf, uint8_t len)
{
    ch395_write_cmd(CMD30_EEPROM_READ);
    ch395_write_data((uint8_t)(eepaddr));
    ch395_write_data((uint8_t)(eepaddr >> 8));
    ch395_write_data(len);
    delay_ms(1);

    while (len--)
    {
        *buf++ = ch395_read_data();
    }
}

/**
 * @brief       设置tcp mss值
 * @param       tcpmss
 * @retval      无
 */
void ch395_set_tcpmss(uint16_t tcpmss)
{
    ch395_write_cmd(CMD20_SET_TCP_MSS);
    ch395_write_data((uint8_t)(tcpmss));
    ch395_write_data((uint8_t)(tcpmss >> 8));
}

/**
 * @brief       设置socket接收缓冲区
 * @param       sockindex  socket索引,址,blknum
 * @param       startblk   起始地
 * @param       单位缓冲区个数 ，单位为512字节
 * @retval      无
 */
void ch395_set_socket_recv_buf(uint8_t sockindex, uint8_t startblk, uint8_t blknum)
{
    ch395_write_cmd(CMD30_SET_RECV_BUF);
    ch395_write_data(sockindex);
    ch395_write_data(startblk);
    ch395_write_data(blknum);
}

/**
 * @brief       设置socket发送缓冲区
 * @param       sockindex  socket索引
 * @param       startblk   起始地址
 * @param       blknum     单位缓冲区个数
 * @retval      无
 */
void ch395_set_socket_send_buf(uint8_t sockindex, uint8_t startblk, uint8_t blknum)
{
    ch395_write_cmd(CMD30_SET_SEND_BUF);
    ch395_write_data(sockindex);
    ch395_write_data(startblk);
    ch395_write_data(blknum);
}

/**
 * @brief       udp向指定的ip和端口发送数据
 * @param       buf     : 发送数据缓冲区
 * @param       len     : 发送数据长度
 * @param       ip      : 目标ip
 * @param       port    : 目标端口
 * @param       sockeid : socket索引值
 * @retval      无
 */
void ch395_udp_send_data(uint8_t *buf, uint32_t len, uint8_t *ip, uint16_t port, uint8_t sockindex)
{
    ch395_set_socket_desip(sockindex, ip); /* 设置socket 0目标IP地址 */
    ch395_set_socket_desport(sockindex, port);
    ch395_send_data(sockindex, buf, len);
}

/**
 * @brief       设置ch395启动参数
 * @param       mdata 设置的参数
 * @retval      无
 */
void ch395_set_start_para(uint32_t mdata)
{
    ch395_write_cmd(CMD40_SET_FUN_PARA);
    ch395_write_data((uint8_t)mdata);
    ch395_write_data((uint8_t)((uint16_t)mdata >> 8));
    ch395_write_data((uint8_t)(mdata >> 16));
    ch395_write_data((uint8_t)(mdata >> 24));
}

/**
 * @brief       获取全局中断状态，收到此命令ch395自动取消中断,0x44及以上版本使用
 * @param       无
 * @retval      返回当前的全局中断状态
 */
uint16_t ch395_cmd_get_glob_int_status_all(void)
{
    uint16_t init_status;
    ch395_write_cmd(CMD02_GET_GLOB_INT_STATUS_ALL);
    delay_us(2);
    init_status = ch395_read_data();
    init_status = (uint16_t)(ch395_read_data() << 8) + init_status;
    ch395_scs_hign;
    return init_status;
}

/**
 * @brief       设置keepalive功能
 * @param       sockindex socket号
 * @param       cmd 0：关闭 1：开启
 * @retval      无
 */
void ch395_set_keeplive(uint8_t sockindex, uint8_t cmd)
{
    ch395_write_cmd(CMD20_SET_KEEP_LIVE_SN);
    ch395_write_data(sockindex);
    ch395_write_data(cmd);
}

/**
 * @brief       设置keepalive重试次数
 * @param       cnt 重试次数（）
 * @retval      无
 */
void ch395_keeplive_cnt(uint8_t cnt)
{
    ch395_write_cmd(CMD10_SET_KEEP_LIVE_CNT);
    ch395_write_data(cnt);
}

/**
 * @brief       设置keeplive空闲
 * @param       idle 空闲时间（单位：ms）
 * @retval      无
 */
void ch395_keeplive_idle(uint32_t idle)
{
    ch395_write_cmd(CMD40_SET_KEEP_LIVE_IDLE);
    ch395_write_data((uint8_t)idle);
    ch395_write_data((uint8_t)((uint16_t)idle >> 8));
    ch395_write_data((uint8_t)(idle >> 16));
    ch395_write_data((uint8_t)(idle >> 24));
}

/**
 * @brief       设置keeplive间隔时间
 * @param       intvl 间隔时间（单位：ms）
 * @retval      无
 */
void ch395_keeplive_intvl(uint32_t intvl)
{
    ch395_write_cmd(CMD40_SET_KEEP_LIVE_INTVL);
    ch395_write_data((uint8_t)intvl);
    ch395_write_data((uint8_t)((uint16_t)intvl >> 8));
    ch395_write_data((uint8_t)(intvl >> 16));
    ch395_write_data((uint8_t)(intvl >> 24));
}

/**
 * @brief       设置ttl
 * @param       ssockindex socket号
 * @param       ttlnum:ttl数
 * @retval      无
 */
void ch395_setttl_num(uint8_t sockindex, uint8_t ttlnum)
{
    ch395_write_cmd(CMD20_SET_TTL);
    ch395_write_data(sockindex);
    ch395_write_data(ttlnum);
}
