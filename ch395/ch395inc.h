/**
 ****************************************************************************************************
 * @file        ch395inc.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-6-17
 * @brief       ch395inc 命令代码
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

#ifndef   __CH395INC_H__
#define   __CH395INC_H__


/* ********************************************************************************************************************* */
/* 常用类型和常量定义 */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef     TRUE
#define     TRUE    1
#define     FALSE   0
#endif
#ifndef     NULL
#define     NULL    0
#endif

/* 命令代码:
 * 一个命令操作顺序包含:
 * 一个命令码(对于串口方式,命令码之前还需要两个同步码)
 * 若干个输入数据(可以是0个)
 * 若干个输出数据(可以是0个)
 * 命令码起名规则: CMDxy_NAME
 * 其中的x和y都是数字, x说明最少输入数据个数(字节数), y说明最少输出数据个数(字节数), y如果是W表示需要等待命令执行成功
 * 有些命令能够实现0到多个字节的数据块读写, 数据块本身的字节数未包含在上述x或y之内
 */
#define CMD01_GET_IC_VER                  0x01        /* 获取芯片以及固件版本号 */

#define CMD31_SET_BAUDRATE                0x02        /* 串口方式: 设置串口通讯波特率(上电或者复位后的默认波特率为9600bps */

#define CMD00_ENTER_SLEEP                 0x03        /* 进入睡眠状态 */

#define CMD00_RESET_ALL                   0x05        /* 执行硬件复位 */

#define CMD11_CHECK_EXIST                 0x06        /* 测试通讯接口以及工作状态 */

#define CMD02_GET_GLOB_INT_STATUS_ALL     0x19        /* 获取全局中断状态，V44版本以后的程序由于增加了socket数量需要用此命令获取全部的中断 */

#define CMD10_SET_PHY                     0x20        /* 设置PHY，默认为Auto，自动协商 */

#define CMD60_SET_MAC_ADDR                0x21        /* 设置MAC地址 必须在CMD00H_INIT_CH395之前设置完毕 */

#define CMD40_SET_IP_ADDR                 0x22        /* 设置IP地址 必须在CMD00H_INIT_CH395之前设置完毕 */

#define CMD40_SET_GWIP_ADDR               0x23        /* 设置网关IP地址 必须在CMD00H_INIT_CH395之前设置完毕 */

#define CMD40_SET_MASK_ADDR               0x24        /* 设置子网掩码， 必须在CMD00H_INIT_CH395之前设置完毕 */

#define CMD90_SET_MAC_FILT                0x25        /* 设置MAC过滤 可以进行广播，多播等过滤 */

#define CMD01_GET_PHY_STATUS              0x26        /* 获取PHY当前状态，如断开连接，10/100M FULL/HALF */

#define CMD0W_INIT_CH395                  0x27        /* 初始化CH395 */

#define CMD08_GET_UNREACH_IPPORT          0x28        /* 获取不可达信息 */

#define CMD01_GET_GLOB_INT_STATUS         0x29        /* 获取全局中断状态，最大值为1S，不可以设置为0 */

#define CMD10_SET_RETRAN_COUNT            0x2A        /* 重试次数，仅在TCP模式下有效 */

#define CMD20_SET_RETRAN_PERIOD           0x2B        /* 重试周期，最大值为20，仅在TCP模式下有效，不可以设置为0 */

#define CMD01_GET_CMD_STATUS              0x2C        /* 获取命令执行状态 */

#define CMD06_GET_REMOT_IPP_SN            0x2D        /* 获取远端的端口以及IP地址，该命令在TCP服务器模式下使用 */

#define CMD10_CLEAR_RECV_BUF_SN           0x2E        /* 清除接收缓冲区  */

#define CMD12_GET_SOCKET_STATUS_SN        0x2F        /* 获取socket n状态 */

#define CMD11_GET_INT_STATUS_SN           0x30        /* 获取socket n的中断状态 */

#define CMD50_SET_IP_ADDR_SN              0x31        /* 设置socket n的目的IP地址 */

#define CMD30_SET_DES_PORT_SN             0x32        /* 设置socket n的目的端口 */

#define CMD30_SET_SOUR_PORT_SN            0x33        /* 设置socket n的源端口 */

#define CMD20_SET_PROTO_TYPE_SN           0x34        /* 设置socket n的协议类型 */

#define CMD1W_OPEN_SOCKET_SN              0x35        /* 打开socket n */

#define CMD1W_TCP_LISTEN_SN               0x36        /* socket n监听，收到此命令，socket n进入服务器模式，仅对TCP模式有效 */

#define CMD1W_TCP_CONNECT_SN              0x37        /* socket n连接，收到此命令，socket n进入客户端模式，仅对TCP模式有效 */

#define CMD1W_TCP_DISNCONNECT_SN          0x38        /* socket n断开连接，收到此命令，socket n断开已有连接，仅对TCP模式有效 */

#define CMD30_WRITE_SEND_BUF_SN           0x39        /* 向socket n缓冲区写入数据 */

#define CMD12_GET_RECV_LEN_SN             0x3B        /* 获取socket n接收数据的长度 */

#define CMD30_READ_RECV_BUF_SN            0x3C        /* 读取socket n接收缓冲区数据 */

#define CMD1W_CLOSE_SOCKET_SN             0x3D        /* 关闭socket n */

#define CMD20_SET_IPRAW_PRO_SN            0x3E        /* 在IP RAW下，设置socket n的IP包协议类型 */

#define CMD01_PING_ENABLE                 0x3F        /* 开启/关闭PING */

#define CMD06_GET_MAC_ADDR                0x40        /* 获取MAC地址 */

#define CMD10_DHCP_ENABLE                 0x41        /* DHCP使能 */

#define CMD01_GET_DHCP_STATUS             0x42        /* 获取DHCP状态 */

#define CMD014_GET_IP_INF                 0x43        /* IP,子网掩码，网关 */

#define CMD00_PPPOE_SET_USER_NAME         0x44        /* 设置PPPOE用户名 */

#define CMD00_PPPOE_SET_PASSWORD          0x45        /* 设置密码 */

#define CMD10_PPPOE_ENABLE                0x46        /* PPPOE使能 */

#define CMD01_GET_PPPOE_STATUS            0x47        /* 获取pppoe状态 */

#define CMD20_SET_TCP_MSS                 0x50        /* 设置TCP MSS */

#define CMD20_SET_TTL                     0x51        /* 设置TTL，TTL最大值为128 */

#define CMD30_SET_RECV_BUF                0x52        /* 设置SOCKET接收缓冲区 */

#define CMD30_SET_SEND_BUF                0x53        /* 设置SOCKET发送缓冲区 */

#define CMD10_SET_MAC_RECV_BUF            0x54        /* 设置MAC接收缓冲区 */

#define CMD40_SET_FUN_PARA                0x55        /* 设置功能参数 */

#define CMD40_SET_KEEP_LIVE_IDLE          0x56        /* 设置KEEPLIVE空闲 */

#define CMD40_SET_KEEP_LIVE_INTVL         0x57        /* 设置间隔时间 */

#define CMD10_SET_KEEP_LIVE_CNT           0x58        /* 重试次数 */

#define CMD20_SET_KEEP_LIVE_SN            0X59        /* 设置socket nkeeplive功能*/

#define CMD00_EEPROM_ERASE                0xE9        /* 擦除EEPROM*/

#define CMD30_EEPROM_WRITE                0xEA        /* 写EEPROM */

#define CMD30_EEPROM_READ                 0xEB        /* 读EEPROM */

#define CMD10_READ_GPIO_REG               0xEC        /* 读GPIO寄存器 */

#define CMD20_WRITE_GPIO_REG              0xED        /* 写GPIO寄存器 */

/* 协议类型 */
#define PROTO_TYPE_IP_RAW                 0           /* IP层原始数据 */
#define PROTO_TYPE_MAC_RAW                1           /* MAC层原始数据 */
#define PROTO_TYPE_UDP                    2           /* UDP协议类型 */
#define PROTO_TYPE_TCP                    3           /* TCP协议类型 */

/* PHY 命令参数/状态 */
#define PHY_DISCONN                       (1<<0)      /* PHY断开 */
#define PHY_10M_FLL                       (1<<1)      /* 10M全双工 */
#define PHY_10M_HALF                      (1<<2)      /* 10M半双工 */
#define PHY_100M_FLL                      (1<<3)      /* 100M全双工 */
#define PHY_100M_HALF                     (1<<4)      /* 100M半双工 */
#define PHY_AUTO                          (1<<5)      /* PHY自动模式，CMD10H_SET_PHY */

/* CH395 MAC过滤 */
#define MAC_FILT_RECV_BORADPKT            (1<<0)      /* 使能接收广播包 */
#define MAC_FILT_RECV_ALL                 (1<<1)      /* 使能接收所有数据包 */
#define MAC_FILT_RECV_MULTIPKT            (1<<2)      /* 使能接收多播包 */
#define MAC_FILT_RECV_ENABLE              (1<<3)      /* 使能接收 */
#define MAC_FILT_SEND_ENABLE              (1<<4)      /* 使能发送 */

/* 中断状态 */
/* 以下为GLOB_INT会产生的状态 */
#define GINT_STAT_UNREACH                 (1<<0)      /* 不可达中断 */
#define GINT_STAT_IP_CONFLI               (1<<1)      /* IP冲突 */
#define GINT_STAT_PHY_CHANGE              (1<<2)      /* PHY状态改变 */
#define GINT_STAT_DHCP                    (1<<3)      /* PHY状态改变 */
#define GINT_STAT_SOCK0                   (1<<4)      /* socket0 产生中断 */
#define GINT_STAT_SOCK1                   (1<<5)      /* socket1 产生中断 */
#define GINT_STAT_SOCK2                   (1<<6)      /* socket2 产生中断 */
#define GINT_STAT_SOCK3                   (1<<7)      /* socket3 产生中断 */
#define GINT_STAT_SOCK4                   (1<<8)      /* scoket4 产生中断 */
#define GINT_STAT_SOCK5                   (1<<9)      /* scoket5 产生中断 */
#define GINT_STAT_SOCK6                   (1<<10)     /* scoket6 产生中断 */
#define GINT_STAT_SOCK7                   (1<<11)     /* scoket7 产生中断 */

/* 以下为Sn_INT会产生的状态 */
#define SINT_STAT_SENBUF_FREE             (1<<0)      /* 发送缓冲区空闲 */
#define SINT_STAT_SEND_OK                 (1<<1)      /* 发送成功 */
#define SINT_STAT_RECV                    (1<<2)      /* socket端口接收到数据或者接收缓冲区不为空 */
#define SINT_STAT_CONNECT                 (1<<3)      /* 连接成功,TCP模式下产生此中断 */
#define SINT_STAT_DISCONNECT              (1<<4)      /* 连接断开,TCP模式下产生此中断 */
#define SINT_STAT_TIM_OUT                 (1<<6)      /* ARP和TCP模式下会发生此中断 */

/* 获取命令状态 */
#define CMD_ERR_SUCCESS                   0x00        /* 命令操作成功 */
#define CMD_RET_ABORT                     0x5F        /* 命令操作失败 */
#define CH395_ERR_BUSY                    0x10        /* 忙状态，表示当前正在执行命令 */
#define CH395_ERR_MEM                     0x11        /* 内存错误 */
#define CH395_ERR_BUF                     0x12        /* 缓冲区错误 */
#define CH395_ERR_TIMEOUT                 0x13        /* 超时 */
#define CH395_ERR_RTE                     0x14        /* 路由错误*/
#define CH395_ERR_ABRT                    0x15        /* 连接停止*/
#define CH395_ERR_RST                     0x16        /* 连接复位 */
#define CH395_ERR_CLSD                    0x17        /* 连接关闭/socket 在关闭状态 */
#define CH395_ERR_CONN                    0x18        /* 无连接 */
#define CH395_ERR_VAL                     0x19        /* 错误的值 */
#define CH395_ERR_ARG                     0x1a        /* 错误的参数 */
#define CH395_ERR_USE                     0x1b        /* 已经被使用 */
#define CH395_ERR_IF                      0x1c        /* MAC错误  */
#define CH395_ERR_ISCONN                  0x1d        /* 已连接 */
#define CH395_ERR_OPEN                    0X20        /* 已经打开 */
#define CH395_ERR_UNKNOW                  0xFA        /* 未知错误 */

/* PPP状态 */
#define CH395_PPP_SUCCESS                 0           /* 成功 */
#define CH395_PPPERR_PARM                 1           /* 无效参数 */
#define CH395_PPPERR_OPEN                 2           /* 无法打开PPP会话 */
#define CH395_PPPERR_DEVICE               3           /* 无效的PPP设备 */
#define CH395_PPPERR_ALLOC                4           /* 资源分配失败 */
#define CH395_PPPERR_USER                 5           /* 用户中断 */
#define CH395_PPPERR_CONNECT              6           /* 连接断开 */
#define CH395_PPPERR_AUTHFAIL             7           /* 挑战鉴别失败 */
#define CH395_PPPERR_PROTOCOL             8           /* 握手协议失败 */
#define CH395_PPPERR_TIME_OUT             9           /* 超时失败 */
#define CH395_PPPERR_CLOSE                10          /* 关闭失败 */

/* 不可达代码 */
#define UNREACH_CODE_HOST                 0           /* 主机不可达 */
#define UNREACH_CODE_NET                  1           /* 网络不可达 */
#define UNREACH_CODE_PROTOCOL             2           /* 协议不可达 */
#define UNREACH_CODE_PROT                 3           /* 端口不可达 */
/* 其他值请参考RFC792文档 */

/* 命令包头 */
#define SER_SYNC_CODE1                    0x57        /* 串口命令同步码1 */
#define SER_SYNC_CODE2                    0xAB        /* 串口命令同步码2 */

/* TCP状态 */
#define TCP_CLOSED                        0
#define TCP_LISTEN                        1
#define TCP_SYN_SENT                      2
#define TCP_SYN_RCVD                      3
#define TCP_ESTABLISHED                   4
#define TCP_FIN_WAIT_1                    5
#define TCP_FIN_WAIT_2                    6
#define TCP_CLOSE_WAIT                    7
#define TCP_CLOSING                       8
#define TCP_LAST_ACK                      9
#define TCP_TIME_WAIT                     10

/* GPIO寄存器地址 */
#define GPIO_DIR_REG                     0x80        /* 寄存器方向寄存器，1：输出;0:输入 */
#define GPIO_IN_REG                      0x81        /* GPIO输入寄存器 */
#define GPIO_OUT_REG                     0x82        /* GPIO输出寄存器 */
#define GPIO_CLR_REG                     0x83        /* GPIO输出清除: 0=keep, 1=clear */
#define GPIO_PU_REG                      0x84        /* GPIO上拉使能寄存器 */
#define GPIO_PD_REG                      0x85        /* GPIO下拉使能寄存器 */

/* 功能参数 */
#define FUN_PARA_FLAG_TCP_SERVER         (1<<1)      /* tcp server 多连接模式标志，0X44版本及以后支持 */
#define FUN_PARA_FLAG_LOW_PWR            (1<<2)      /* 低耗能模式 */
#define SOCK_CTRL_FLAG_SOCKET_CLOSE      (1<<3)      /* CH395不主动关闭Socket */
#define SOCK_DISABLE_SEND_OK_INT         (1<<4)      /* send ok中断控制位，为1表示关闭send ok中断 */

#ifdef __cplusplus
}
#endif
#endif
