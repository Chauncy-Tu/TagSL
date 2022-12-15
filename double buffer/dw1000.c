/*
		Public Library
		Source Path:	D:\work\Broadcast\Broadcast_PersonTag_nRF52840_V1\APP\HARDWARE\DW1000
*/

#include "dw1000.h"
#include "uwb_protocol.h"
#include "user_data_handler.h"
#include "UWBPosTof.h"
//#include "UWBPosBroadcastProcessor.h"
#include "user_ble_api.h"
#include "exti.h"
#include "timer.h"
#include "math.h"
#include "sync_calibration.h"
#include "icm426xx_user.h"
#include "drv_hrt.h"
#include "si117x_config.h"
#include "dw1000_int_handler.h"
//////////////////////////////



#ifdef OUTPUT_UWB_DATA
UwbRangeDataMsg uwbRangeData={0};
#endif

extern tag_status_t tag_status;

int32 init_dw1000()
{
	uint32 devID ;
	dwt_txconfig_t  configTX ;
	int result;  
	dwt_config_t dwtconfigData;
	uint8 msg[127] = "The quick brown fox jumps over the lazy dog. The quick brown fox jumps over the lazy dog. The quick brown fox jumps over the l";
	uint16 sleep_mode = 0;
	uint8 resetCnt = 0;

	port_reinit_dw1000();
	
	if(!userPlatformApi.dw1000_reinit_flag)
	{
		printf("read dw1000 devid\r\n");
	}
	else
		SPIMx_Init(SPIM2_INSTANCE,NRF_DRV_SPI_FREQ_2M);
	
	devID = dwt_readdevid();
	
	if(DWT_DEVICE_ID != devID) //if the read of device ID fails, the DW1000 could be asleep
	{
		DW_CSN_L;  //CS low
		nrf_delay_ms(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
		DW_CSN_H;  //CS high
		nrf_delay_ms(7);
		do{
			reset_DW1000();
			nrf_delay_ms(1);
			devID = dwt_readdevid();// SPI not working or Unsupported Device ID
			resetCnt++;
		}while((DWT_DEVICE_ID != devID)&&(resetCnt < 50));
		//clear the sleep bit - so that after the hard reset below the DW does not go into sleep
		dwt_softreset();
	}
    if(resetCnt>=50)tag_status.DW1000_IC=0;
    //reset the DW1000 by driving the RSTn line low
    //reset_DW1000();
	if(!userPlatformApi.dw1000_reinit_flag)printf("dw1000 devid:%x\r\n", devID);
	
//	if(tag_configdata.TOF_POSITION_MODE == NEW_TOF_POSITION_MODE)
//		disable_crc_error_handling();
    result = dwt_initialise(DWT_LOADUCODE | DWT_LOADLDOTUNE | DWT_LOADTXCONFIG | DWT_LOADANTDLY| DWT_LOADXTALTRIM);
	if (DWT_SUCCESS != result)
	{
		uart_printf("DW1000 ERROR\r\n");
		return (-1) ;   // device initialise has failed
	}
		
	if(SystemStatus != TX_POWER_MODE)
	{
		SPIMx_Init(SPIM2_INSTANCE,NRF_DRV_SPI_FREQ_8M);
	}
	
	dwt_configeventcounters(1); 
	dwt_setautorxreenable(0); //disable auto RX re-enable
	dwt_setdblrxbuffmode(1); //disable double RX buffer

	dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO), 1);
	dwt_setcallbacks(instance_txcallback, instance_rxcallback);
	
	devID = dwt_readdevid();
	
	if (DWT_DEVICE_ID != devID)   // Means it is NOT DW1000 device
	{
	 tag_status.DW1000_IC=0;
	uart_printf("DW1000 ERROR\r\n");
	 // SPI not working or Unsupported Device ID
	 return(-1) ;
	}
	tag_status.DW1000_IC = 1;

	dwt_configure(&UWBConfigData, DWT_LOADXTALTRIM) ;
	configTX.PGdly = tag_configdata.UWB_PGdly;
	configTX.power =(tag_configdata.TX_POWER<<24)+(tag_configdata.TX_POWER<<16)+(tag_configdata.TX_POWER<<8)+(tag_configdata.TX_POWER);  //0x69696969    //0x4F4F4F  //+
	dwt_configuretxrf(&configTX);
				
	/* Apply default antenna delay value. See NOTE 2 below. */
	dwt_setrxantennadelay(16495);//16495
	dwt_settxantennadelay(16495);
		
	if(SystemStatus ==TX_POWER_MODE)
	{
		UWB_PA_ON; 
		dwt_configcontinuousframemode(0x1000);
		dwt_writetxdata(127, (uint8 *)  msg, 0) ;
		dwt_writetxfctrl(127, 0);
		dwt_starttx(DWT_START_TX_IMMEDIATE);
		//RED_LED_ON;
		return 0;
	}
	else
	{
			/* Set expected response's delay and timeout. See NOTE 1 and 5 below.
		* As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
		dwt_enableframefilter(DWT_FF_NOTYPE_EN); //disable frame filtering
		dwt_setrxaftertxdelay(0); //no delay of turning on of RX
		dwt_setrxtimeout(0);
		dwt_setpreambledetecttimeout(0);
		sleep_mode = (DWT_LOADUCODE|DWT_PRESRV_SLEEP|DWT_CONFIG); //
		dwt_configuresleep(sleep_mode, DWT_WAKE_CS|DWT_SLP_EN);
		DW1000_Enter_Sleep_Check();
	}
	if(!userPlatformApi.dw1000_reinit_flag)printf("DW1000 OK\r\n");
	return devID;
}

u8 dwt_wakeup()
{
	uint8 reset_Cnt=0;
	static u8 wakeupFailTimes_temp = 0;
	//wake up device from low power mode
	//NOTE - in the ARM  code just drop chip select for 200us
	DW_CSN_L;  //CS low
	nrf_delay_us (500);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
	DW_CSN_H;  //CS high
	nrf_delay_us(2000 +  tagWorkStatus.wakeUpDelay100ms * 100); //此处的唤醒后预留的时间不够，需要1.5ms 原时间为nrf_delay_us(500);
	
	if(DWT_DEVICE_ID != dwt_readdevid()) //if the read of device ID fails, the DW1000 could be asleep
	{
		tagWorkStatus.wakeupFailTimes++;
		if(++wakeupFailTimes_temp > 5) //连续五次唤醒不成功
		{
			if(++tagWorkStatus.wakeUpDelay100ms > 20) //最多再多等待 2ms 不然就重新初始化DW1000
			{
				tagWorkStatus.wakeUpDelay100ms = 0; 
				wakeupFailTimes_temp = 0;
				userPlatformApi.dw1000_reinit_flag = 1;

			}
		}
		return 0;  //唤醒失败
	}
	else
	{
		wakeupFailTimes_temp = 0;

		return 1;
	}

}



extern calculateTofParamter_t TofParamter;


u8 uwbActiveFlag =0;

void user_ble_advertising(void)
{
	if(adv_timeout_flag == 1)
	{
		adv_timeout_flag=0;
		advertising_start(); 
	}
}


void final_process_low(void)
{
	uwbActiveFlag = 1;
	sendRxsignalPower();
	handler_user_data_to_flash();
	reload_time1_cc_channel0(tagTofConfig.blink_finalInterval);
	user_ble_advertising();
}





void reinit_dw1000(void)
{
	if(auto_correction_flag)
	{
		static int32 msgCarrierArray[5] = {0};
		static u8 	arrayCnt = 0;
		s8 uwbXtrimTemp;
		msgCarrierArray[arrayCnt++] = msgCarrierGloble;
		if(arrayCnt >= 5)
		{
			msgCarrierGloble = GetMedianNum(msgCarrierArray, 5);
			uwbXtrimTemp = tag_configdata.UWB_XTRIM + msgCarrierGloble/2074;  //3
			if(uwbXtrimTemp <= 0) 
				tag_configdata.UWB_XTRIM = 1;
			else
				tag_configdata.UWB_XTRIM = uwbXtrimTemp;
			userPlatformApi.user_flash_write_flag = 1; 
			userPlatformApi.reset_CPU_flag = 1; //重启程序
			auto_correction_flag = 0;
			arrayCnt =0;
		}
//		msgCarrierGloble = 0;
		//连续存五组数据，做中值滤波，
		return ;
	}
	else if((userPlatformApi.dw1000_reinit_flag ==1) && (++tagWorkStatus.reInitDW1000Times > 15) )  //重新初始化DW1000超过20次，那就直接重启芯片
	{
		saveDeviceStatus.selfResetFlag = 1;
		saveDeviceStatus.selfResetCnt ++; 
//		strLength = sprintf(logBuffer, "\r\nSelf reset:%d\r\n", saveDeviceStatus.selfResetCnt);
//		write_log_tack((u32 *)logBuffer,strLength);
		nrf_delay_ms(20);
		NVIC_SystemReset();
	}
	///重新初始化DW1000的过程需要关掉UWB
	stop_uwb_period_run();
	nrf_delay_ms(250);
	init_dw1000();
	start_uwb_period_run();
	//再次打开
}

void port_reinit_dw1000(void)
{
	if(hardwareVersion == TC102_TAG_HARDWARE)
	{
		nrf_delay_ms(200);
		UWB_PA_ON;
		nrf_delay_ms(200);
	}
	else
	{
		if(hardwareVersion == TH102_TAG_HARDWARE || hardwareVersion == TI102_TAG_HARDWARE)		
			UWB_PA_ON;
		DW1000_RSTn_L;
		nrf_delay_ms(5);
		DW1000_RSTn_H;
		nrf_delay_ms(50);
		dwt_softreset();
	}

}
