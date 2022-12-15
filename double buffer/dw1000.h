/*
		Public Library
		Source Path:	D:\work\Broadcast\Broadcast_PersonTag_nRF52840_V1\APP\HARDWARE\DW1000
*/

#ifndef _DW1000_H
#define _DW1000_H

#include <stdint.h>
#include "deca_device_api.h"
#include "deca_types.h"
#include "deca_regs.h"
#include "port.h"
#include "spi.h"
#include "nrf_nvmc.h"
#include "sys.h"
#include "signal.h"
#include "nrf_delay.h"
#include "tag_config.h"
#include "rtc.h"
#include "exti.h"
#include "uart.h"
#include "nrf_drv_saadc.h"
#include "sys_config.h"

#define NORM_MODE        0
#define TESTTX_MODE      1
#define TESTRANGE_MODE   2
#define TESTMESH_MODE    3

//#define OUTPUT_UWB_DATA

#define WATCH_TAG_TWI101 //手环标签

//#define TEST_DEBUG_VERSION
//#define JUMP_SLOT //跳跃式和探测式（newTof）二者取其一，不能共�
#define SEND_LONG_FINAL //send long final mode

typedef struct
{
    uint8 frameNumber;                          // 
    uint8 axisX[2]; 
    uint8 axisY[2]; 
    uint8 axisZ[2]; 
    uint8 batteryLevel; 						//
    uint8 posInterval;
    uint8 alarmStatus;           			    //
    uint8 refAnchorMAC[6]; 						//
    //uint8 rsv[2]; 						//

} ble_position_frame_t ;

typedef struct
{
    uint32_t getTimerCnt[2];                          // 
    uint16_t anchorID[2];                          // 
	uint8_t  arrayNumber;
} crt_rx_timer_t;


int32 init_dw1000(void);

void instance_clearposition(void);
u8 dwt_wakeup(void);


void reinit_dw1000(void);
void port_reinit_dw1000(void);


extern u8 uwbActiveFlag;

extern crt_rx_timer_t crtRxTimer;
void final_process_low(void);
void sendRxsignalPower(void);
/////////

/////////////
#endif
