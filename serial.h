#ifndef __SERIAL_H
#define __SERIAL_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "usbd_cdc_if.h"

/* Exported define -----------------------------------------------------------*/
#define FRAME_HEADER     0xA5    /* 帧头 */
#define FRAME_TYPE_STATUS 0x01   /* 状态帧类型 */
#define FRAME_TYPE_OFFSET 0x02   /* 偏差值帧类型 */

/* Exported types ------------------------------------------------------------*/
/* 接收数据包定义 */
typedef struct
{
    uint8_t header;        /* 帧头 */
    uint8_t type;          /* 类型 */
    union
    {
        uint8_t status;    /* 状态值 */
        struct {
            int16_t x_offset;  /* x方向偏移值 */
            int16_t y_offset;  /* y方向偏移值 */
        } offset;
    } data;
} Vision_Frame_t;

/* 机器人信息结构体 */
typedef struct
{
    union
    {
        struct
        {
            uint8_t header;        /* 帧头 */
            uint8_t type;          /* 类型 */
            uint8_t status;        /* 状态 */
            int16_t x_offset;      /* x方向偏移值 */
            int16_t y_offset;      /* y方向偏移值 */
        };
        uint8_t data[32];         /* 原始数据缓冲区 */
    } Robot_Info;
} Vision_data_rx;

/* Exported variables --------------------------------------------------------*/
extern Vision_data_rx RV_slove_revice;
extern uint8_t temp[60];
extern uint8_t count;

/* Exported functions prototypes ---------------------------------------------*/
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
void classdef_SendToPC(Sentry_VisionSendMsg_Usb *pack2vision);

#endif /* __SERIAL_H */