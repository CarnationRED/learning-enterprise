/**
 ****************************************************************************************************
 * @file        ch395cmd.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-6-17
 * @brief       ch395cmd 驱动代码
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
 
#ifndef __CH395CMD_H__
#define __CH395CMD_H__
#include "ch395inc.h"
#include "stdint.h"


void ch395_cmd_reset(void);                                                                             /* 复位 */

void ch395_cmd_sleep(void);                                                                             /* 睡眠 */

uint8_t ch395_cmd_get_ver(void);                                                                        /* 获取芯片及固件版本号 */

uint8_t ch395_cmd_check_exist(uint8_t testdata);                                                        /* 测试命令 */

void ch395_cmd_set_phy(uint8_t phystat);                                                                /* 设置phy状态  */

uint8_t ch395_cmd_get_phy_status(void);                                                                 /* 获取phy状态 */

uint8_t ch395_cmd_get_glob_int_status(void);                                                            /* 获取ch395全局中断状态 */

uint8_t ch395_cmd_init(void);                                                                           /* 初始化ch395 */

void ch395_cmd_set_uart_baud_rate(uint32_t baudrate);                                                   /* 设置波特率 */

uint8_t ch395_get_cmd_status(void);                                                                     /* 获取命令执行状态 */

void ch395_cmd_set_ipaddr(uint8_t *ipaddr);                                                             /* 设置ch395的ip地址 */

void ch395_cmd_set_gw_ipaddr(uint8_t *gwipaddr);                                                        /* 设置ch395的网关ip地址 */

void ch395_cmd_set_maskaddr(uint8_t *maskaddr);                                                         /* 设置子网掩码 */

void ch395_cmd_set_macaddr(uint8_t *amcaddr);                                                           /* 设置ch395的mac地址 */

void ch395_cmd_get_macaddr(uint8_t *amcaddr);                                                           /* 获取mac地址 */

void ch395_cmd_set_macfilt(uint8_t filtype, uint32_t table0, uint32_t table1);                          /* 设置ch395的mac过滤 */

void ch395_cmd_get_unreachippt(uint8_t *list);                                                          /* 获取不可达地址以及端口 */

void ch395_cmd_get_remoteipp(uint8_t sockindex, uint8_t *list);                                         /* 获取远端ip和端口，一般在tcp server下使用 */

void ch395_set_socket_desip(uint8_t sockindex, uint8_t *ipaddr);                                        /* 设置scoket n的目的ip地址 */

void ch395_set_socket_prot_type(uint8_t sockindex, uint8_t prottype);                                   /* 设置socket n的协议类型 */

void ch395_set_socket_desport(uint8_t sockindex, uint16_t desprot);                                     /* 设置socket n的目的端口 */

void ch395_set_socket_sourport(uint8_t sockindex, uint16_t surprot);                                    /* 设置socket n的源端口 */

void ch395_set_socket_ipraw_proto(uint8_t sockindex, uint8_t prototype);                                /* 在ipraw模式下，设置socket n的ip包协议字段 */

void ch395_set_recv_threslen(uint8_t sockindex, uint16_t len);                                          /* 设置socket n的接收中断阀值 */

void ch395_send_data(uint8_t sockindex, uint8_t *databuf, uint16_t len);                                /* 向socket n的发送缓冲区写数据 */

uint16_t ch395_get_recv_length(uint8_t sockindex);                                                      /* 获取socket n的接收长度 */

void ch395_clear_recv_buf(uint8_t sockindex);                                                           /* 清除socket n的接收缓冲区 */

void ch395_get_recv_data(uint8_t sockindex, uint16_t len, uint8_t *pbuf);                               /* 获取接收数据 */

void ch395_cmd_set_retry_count(uint8_t count);                                                          /* 设置最大重试次数 */

void ch395_cmd_set_retry_period(uint16_t period);                                                       /* 设置最大重试周期 单位 毫秒 */

void ch395_cmd_get_socket_status(uint8_t sockindex, uint8_t *status) ;                                  /* 获取socket n的状态 */

uint8_t  ch395_open_socket(uint8_t sockindex);                                                          /* 打开socket n*/

uint8_t  ch395_close_socket(uint8_t sockindex);                                                         /* 关闭socket n*/

uint8_t ch395_tcp_connect(uint8_t sockindex);                                                           /* tcp连接 */

uint8_t ch395_tcp_listen(uint8_t sockindex);                                                            /* tcp监听 */

uint8_t ch395_tcp_disconnect(uint8_t sockindex);                                                        /* tcp断开连接 */

uint8_t ch395_get_socket_int(uint8_t sockindex);                                                        /* 获取socket n的中断状态 */

uint8_t ch395_crcret_6bit(uint8_t *mac_addr);                                                           /* 多播地址crc32，用于hash过滤 function count = 36 */

void ch395_get_ipinf(uint8_t *addr);                                                                    /* 获取ip，子网掩码和网关地址 */

uint8_t ch395_get_dhcp_status(void);                                                                    /* 获取dhcp状态 */

uint8_t  ch395_dhcp_enable(uint8_t flag);                                                               /* 启动/停止dhcp */

void ch395_write_gpio_addr(uint8_t regadd, uint8_t regval);                                             /* 写gpio寄存器 */

uint8_t ch395_read_gpio_addr(uint8_t regadd);                                                           /* 读gpio寄存器 */

uint8_t ch395_eeprom_erase(void);                                                                       /* 擦除eeprom */

uint8_t ch395_eeprom_write(uint16_t eepaddr, uint8_t *buf, uint8_t len);                                /* 写eeprom */

void ch395_eeprom_read(uint16_t eepaddr, uint8_t *buf, uint8_t len);                                    /* 读eeprom */

void ch395_set_tcpmss(uint16_t tcpmss);                                                                 /* 设置tcp mss值 */

void ch395_set_socket_recv_buf(uint8_t sockindex, uint8_t startblk, uint8_t blknum);                    /* 设置socket接收缓冲区 */

void ch395_set_socket_send_buf(uint8_t sockindex, uint8_t startblk, uint8_t blknum);                    /* 设置socket发送缓冲区 */

void ch395_udp_send_data(uint8_t *buf, uint32_t len, uint8_t *ip, uint16_t port, uint8_t sockindex);    /* udp向指定的ip和端口发送数据 */

void ch395_set_start_para(uint32_t mdata);                                                              /* 设置ch395启动参数 */

uint16_t ch395_cmd_get_glob_int_status_all(void);                                                       /* 获取全局中断状态，收到此命令ch395自动取消中断,0x44及以上版本使用 */

void ch395_keeplive_idle(uint32_t idle);                                                                /* 设置keeplive空闲 */

void ch395_keeplive_intvl(uint32_t intvl);                                                              /* 设置keeplive间隔时间 */

void ch395_keeplive_cnt(uint8_t cnt);                                                                   /* 设置keepalive重试次数 */

void ch395_set_keeplive(uint8_t sockindex, uint8_t cmd);                                                /* 设置ttl */

void ch395_setttl_num(uint8_t sockindex, uint8_t ttlnum);                                               /* 设置ttl */

#endif
