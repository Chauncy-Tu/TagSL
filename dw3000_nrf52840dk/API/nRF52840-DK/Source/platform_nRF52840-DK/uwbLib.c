/*! ----------------------------------------------------------------------------
 *  @file    uwbLib.c
 *  @brief   uwb ranging and directing lib
 *
 * @attention
 *
 * Copyright 2020 (c) Huayuen Ltd.
 *
 * All rights reserved.
 *
 * @author Huayuen
 */

#include "uwbLib.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <deca_vals.h>
#include <port.h>

typedef struct 
{
    char            role;
    signed char     verifyRes;

    char            SN[16];
    long            uID[4];

    dwt_config_t    uwbConfig;
    dwt_txconfig_t  uwbTxConfig;
    
    char*           uwbUserTxBuf;
    char*           uwbUserRxBuf;
    int             uwbUserDataNum;
    
    long            range;
    short           pdoa1;
    short           pdoa2;
    
    short           pdoaCal1;
    short           pdoaCal2;

    uwb_cb_t        cb_meas_done;
} status;

status statusData = {
    .verifyRes    = -1,
    .role         = ROLE_LISENTER,

    .uwbConfig    = {
                    5, /* Channel number. */
                    DWT_PLEN_128, /* Preamble length. Used in TX only. */
                    DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
                    9, /* TX preamble code. Used in TX only. */
                    9, /* RX preamble code. Used in RX only. */
                    DWT_SFD_DW_8, /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
                    DWT_BR_850K, /* Data rate. */
                    DWT_PHRMODE_STD, /* PHY header mode. */
                    DWT_PHRRATE_STD, /* PHY header rate. */
                    (129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
                    (DWT_STS_MODE_1_SDC), /* STS enabled */
                    DWT_STS_LEN_256, /* STS length see allowed values in Enum dwt_sts_lengths_e */
                    DWT_PDOA_M3 /* PDOA mode 3 */
    },
    
    .uwbTxConfig  = {
                    0x34, /* PG delay. */
                    0xfdfdfdfd, /* TX power. */
                    0x0 /*PG count*/   
    }
};

uwb_result uwbRes = {
    .flag         = FALSE,
    .firstFlag    = TRUE,
    .libVersion   = "00.00.01",
    .hwVersion    = "BI301-V1.00"
    //.hwVersion    = "BI301-V1.00",
    //.range        = 0,
    //.aoa          = 0
};

typedef struct
{
    uint8_t         frame_type;    //bit7~1: frame type, 0x00: sync; 0x10: blink, 0x20: response; 
                                   //bit0: round counter, 0:first round, 1:second round
    uint8_t         tag_sn;        //serial number of the tag or the anchor, set to 0
    uint8_t         frame_cnt;     //frame counter, increase by 1 every second for now
    uint8_t         slot_cnt;      //blink counter, increase by 1 every 8ms for now
    int32_t         t_reply;       //tx-rx
    uint8_t         user_data[10];  //resered for futuer use, such as the timing sync among multiple anchors
    uint8_t         crc[2];
} uwb_msg;

uint8_t assigned_slot_ = 2;  //TODO: this value should be different for different tags

uwb_msg tx_msg_;
uwb_msg rx_msg_;
uint8_t frame_cnt_ = 0;
uint8_t slot_cnt_ = 0;
uint8_t slot_cnt_rcvd_ = 0;
uint8_t round_cnt_ = 0;
uint8_t synced_ = 0;
int64_t tx_time_ = 0, rx_time_ = 0;
int32_t clkbias_local_, clkbias_match_; // clkbias_local_ is local, clkbias_match_ is matchmodule.
int16_t pdoa1_, pdoa2_;

// 3 nodes test
uint8_t frame_cnt_test[3]={0,0,0};
uint8_t slot_cnt_test[3] ={0,0,0};
uint8_t range_test[3]={0,0,0};
uint8_t aoa_test[3]={0,0,0};

uint8_t tag_num=3;
uint8_t tag_cnt=0;
uint8_t record_cnt=0;
//

#define UUS_TO_DWT_TIME 63898
#define SPEED_OF_LIGHT   (299702547)

#define SHA256_ROTL(a,b) (((a>>(32-b))&(0x7fffffff>>(31-b)))|(a<<b))
#define SHA256_SR(a,b) ((a>>b)&(0x7fffffff>>(b-1)))

#define SHA256_Ch(x,y,z) ((x&y)^((~x)&z))
#define SHA256_Maj(x,y,z) ((x&y)^(x&z)^(y&z))
#define SHA256_E0(x) (SHA256_ROTL(x,30)^SHA256_ROTL(x,19)^SHA256_ROTL(x,10))
#define SHA256_E1(x) (SHA256_ROTL(x,26)^SHA256_ROTL(x,21)^SHA256_ROTL(x,7))
#define SHA256_O0(x) (SHA256_ROTL(x,25)^SHA256_ROTL(x,14)^SHA256_SR(x,3))
#define SHA256_O1(x) (SHA256_ROTL(x,15)^SHA256_ROTL(x,13)^SHA256_SR(x,10))

char* str_SHA256(const char* str, long long length, char* sha256) 
{
    char pp[64];
    char *ppend;
    long l, i, W[64], T1, T2, A, B, C, D, E, F, G, H, H0, H1, H2, H3, H4, H5, H6, H7;
    H0 = 0x6a09e667, H1 = 0xbb67ae85, H2 = 0x3c6ef372, H3 = 0xa54ff53a;
    H4 = 0x510e527f, H5 = 0x9b05688c, H6 = 0x1f83d9ab, H7 = 0x5be0cd19;
    long K[64] = {
            0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
            0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
            0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
            0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
            0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
            0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
            0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
            0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
    };

    l = 64;
    for (i = 0; i < length; pp[i + 3 - 2 * (i % 4)] = str[i], i++);
    for (pp[i + 3 - 2 * (i % 4)] = 128, i++; i < l; pp[i + 3 - 2 * (i % 4)] = 0, i++);
    *((long*)(pp + l - 4)) = length << 3;
    *((long*)(pp + l - 8)) = length >> 29;

    char *p = pp;
    for (ppend = p + l; p < ppend; p += 64) {
        for (i = 0; i < 16; W[i] = ((long*)p)[i], i++);
        for (i = 16; i < 64; W[i] = (SHA256_O1(W[i - 2]) + W[i - 7] + SHA256_O0(W[i - 15]) + W[i - 16]), i++);
        A = H0, B = H1, C = H2, D = H3, E = H4, F = H5, G = H6, H = H7;
        for (i = 0; i < 64; i++) {
            T1 = H + SHA256_E1(E) + SHA256_Ch(E, F, G) + K[i] + W[i];
            T2 = SHA256_E0(A) + SHA256_Maj(A, B, C);
            H = G, G = F, F = E, E = D + T1, D = C, C = B, B = A, A = T1 + T2;
        }
        H0 += A, H1 += B, H2 += C, H3 += D, H4 += E, H5 += F, H6 += G, H7 += H;
    }

    *(long *)sha256 = H0;
    *(long *)(sha256 + 4) = H1;
    *(long *)(sha256 + 8) = H2;
    *(long *)(sha256 + 12) = H3;
    *(long *)(sha256 + 16) = H4;
    *(long *)(sha256 + 20) = H5;
    *(long *)(sha256 + 24) = H6;
    *(long *)(sha256 + 28) = H7;
    return sha256;
}

/*
This function is used to calculate aoa using one measurement of two pdoas
*/
static void calculate_aoa(void)
{
    statusData.pdoa1 = pdoa1_ + statusData.pdoaCal1;
    if (statusData.pdoa1 > 6434) statusData.pdoa1 -= 12868;
    if (statusData.pdoa1 < -6434) statusData.pdoa1 += 12868;
    statusData.pdoa2 = pdoa2_ + statusData.pdoaCal2;
    if (statusData.pdoa2 > 6434) statusData.pdoa2 -= 12868;
    if (statusData.pdoa2 < -6434) statusData.pdoa2 += 12868;
    
    int diff1 = abs(statusData.pdoa1 - uwbRes.pdoa1[slot_cnt_rcvd_]);
    int diff2 = 0;
    if (abs(statusData.pdoa1) > 5000 || diff1 > 6000) {
        if (statusData.pdoa1 > 0) {
            diff2 = abs(statusData.pdoa1 - 12868 - uwbRes.pdoa1[slot_cnt_rcvd_]);
            if (diff2 < diff1) {
                statusData.pdoa1 -= 12868; 
            }
        } else {
            diff2 = abs(statusData.pdoa1 + 12868 - uwbRes.pdoa1[slot_cnt_rcvd_]);
            if (diff2 < diff1) {
                statusData.pdoa1 += 12868;
            }
        }
    }
    diff1 = abs(statusData.pdoa2 - uwbRes.pdoa2[slot_cnt_rcvd_]);
    if (abs(statusData.pdoa2) > 5000 || diff2 > 6000) {
        if (statusData.pdoa2 > 0) {
            diff2 = abs(statusData.pdoa2 - 12868 - uwbRes.pdoa2[slot_cnt_rcvd_]);
            if (diff2 < diff1) {
                statusData.pdoa2 -= 12868; 
            }
        } else {
            diff2 = abs(statusData.pdoa2 + 12868 - uwbRes.pdoa2[slot_cnt_rcvd_]);
            if (diff2 < diff1) {
                statusData.pdoa2 += 12868;
            }
        }
    }
    uwbRes.pdoa1[slot_cnt_rcvd_] = statusData.pdoa1;
    uwbRes.pdoa2[slot_cnt_rcvd_] = statusData.pdoa2;
    
    uwbRes.aoa[slot_cnt_rcvd_] = (atan2(sqrt(3) * (double)statusData.pdoa1, 2 * (double)statusData.pdoa2 - (double)statusData.pdoa1))/3.1415926535*180;
}

static int set_uwb_parameter(char *authCode, char role, char *SN, char preambleLen, char dataRate, char stsLen, char pdoaMode, short pdoaCal1, short pdoaCal2, char *txbuf, char *rxbuf, uwb_cb_t callback) {
   
    long *u_id = (long *)0x0050084C;

//  for generate authcode  //
//    long u_id[4] = //{0x049CF62D,0xFFFFFF09,0xFFFF0037,0xFFFF0011};
//                   {0x049CF62D,0xFFFFFF09,0xFFFF0013,0xFFFF0010};
/////////////////////////////      

    char verifyCode[32];
    char originCode[64];
    
    statusData.uID[0] = u_id[0];
    statusData.uID[1] = u_id[1];
    statusData.uID[2] = u_id[2];
    statusData.uID[3] = u_id[3];    
    sprintf(originCode, "H%08lxua%08lxYu%08lxe%08lxn", u_id[0], u_id[1], u_id[2], u_id[3]);
    str_SHA256(originCode, 39, verifyCode);
    
//    printf("generate authcode\n0x%08lX, 0x%08lX, 0x%08lX, 0x%08lX,\n0x%08lX, 0x%08lX, 0x%08lX, 0x%08lX\r\n", 
//          ((long *)verifyCode)[0], ((long *)verifyCode)[1], ((long *)verifyCode)[2], ((long *)verifyCode)[3],
//          ((long *)verifyCode)[4], ((long *)verifyCode)[5], ((long *)verifyCode)[6], ((long *)verifyCode)[7]);
    
    statusData.verifyRes = 0;//memcmp(verifyCode, authCode, 32);
    if (statusData.verifyRes != 0) {
        return -1;
    }
    
    if (role == ROLE_TAG) {
        statusData.role = ROLE_TAG;
    } else if (role == ROLE_ANCHOR) {
        statusData.role = ROLE_ANCHOR;
    } else if (role == ROLE_LISENTER) {
        statusData.role = ROLE_LISENTER;
    } else {
        return -2;
    }
    
    memcpy(statusData.SN, SN, 16);

    if (preambleLen == DWT_PLEN_72) {
        statusData.uwbConfig.txPreambLength = DWT_PLEN_72;
    } else if (preambleLen == DWT_PLEN_32) {
        statusData.uwbConfig.txPreambLength = DWT_PLEN_32;
    } else if (preambleLen == DWT_PLEN_64) {
        statusData.uwbConfig.txPreambLength = DWT_PLEN_64;
    } else if (preambleLen == DWT_PLEN_128) {
        statusData.uwbConfig.txPreambLength = DWT_PLEN_128;
    } else if (preambleLen == DWT_PLEN_256) {
        statusData.uwbConfig.txPreambLength = DWT_PLEN_256;
    } else if (preambleLen == DWT_PLEN_512) {
        statusData.uwbConfig.txPreambLength = DWT_PLEN_512;
    } else {
        return -4;
    }

    if (dataRate == DWT_BR_850K) {
        statusData.uwbConfig.dataRate = DWT_BR_850K;
    } else if (dataRate == DWT_BR_6M8) {
        statusData.uwbConfig.dataRate = DWT_BR_6M8;
    } else {
        return -5;
    }

    if (stsLen == DWT_STS_LEN_32) {
        statusData.uwbConfig.stsLength = DWT_STS_LEN_32;
    } else if (stsLen == DWT_STS_LEN_64) {
        statusData.uwbConfig.stsLength = DWT_STS_LEN_64;
    } else if (stsLen == DWT_STS_LEN_128) {
        statusData.uwbConfig.stsLength = DWT_STS_LEN_128;
    } else if (stsLen == DWT_STS_LEN_256) {
        statusData.uwbConfig.stsLength = DWT_STS_LEN_256;
    } else if (stsLen == DWT_STS_LEN_512) {
        statusData.uwbConfig.stsLength = DWT_STS_LEN_512;
    } else {
        return -6;
    }

    if (pdoaMode == DWT_PDOA_M0) {
        statusData.uwbConfig.pdoaMode = DWT_PDOA_M0;
    } else if (pdoaMode == DWT_PDOA_M1) {
        statusData.uwbConfig.pdoaMode = DWT_PDOA_M1;
    } else if (pdoaMode == DWT_PDOA_M3) {
        statusData.uwbConfig.pdoaMode = DWT_PDOA_M3;
    } else {
        return -7;
    }
    
    statusData.pdoaCal1 = pdoaCal1;
    statusData.pdoaCal2 = pdoaCal2;
    
    statusData.uwbUserTxBuf = txbuf;
    statusData.uwbUserRxBuf = rxbuf;
    
    statusData.uwbUserRxBuf[0]  = 0xAA;
    statusData.uwbUserRxBuf[1]  = 0x55;
    statusData.uwbUserRxBuf[2]  = 0xA5;
    statusData.uwbUserRxBuf[3]  = 0x5A;
    statusData.uwbUserRxBuf[4]  = 0x00;
    statusData.uwbUserRxBuf[5]  = 0x00;
    statusData.uwbUserRxBuf[6]  = 0x00;
    statusData.uwbUserRxBuf[7]  = 0x0D;
    statusData.uwbUserRxBuf[8]  = 0x0A;
    statusData.uwbUserRxBuf[9]  = 0x0D;
    statusData.uwbUserRxBuf[10] = 0x0A;

    statusData.cb_meas_done = callback;
    return 0;
}

/*
This function is used to send the data in tx_msg_ with dw3000
*/
static int send_uwb_frame()
{
    uint64_t systime = 0;

    dwt_writesysstatuslo(0xFFFFFFFF);
    dwt_forcetrxoff();
    
    systime = (uint64_t)dwt_readsystimestamphi32() << 8;
    //uint32_t delaytxtime = ((systime + (300 * UUS_TO_DWT_TIME)) >> 8) & 0xFFFFFF00;
    uint32_t delaytxtime = ((systime + (1200 * UUS_TO_DWT_TIME)) >> 8) & 0xFFFFFF00;
    dwt_setdelayedtrxtime(delaytxtime);
    tx_time_ = (uint64_t)delaytxtime << 8;
    tx_msg_.t_reply = (rx_time_)? tx_time_ - rx_time_ : 0;
    
    dwt_writetxdata(sizeof(tx_msg_) - 2, (uint8_t *)&tx_msg_, 0);
    dwt_writetxfctrl(sizeof(tx_msg_), 0, 1);

    if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        dwt_writesysstatuslo(0xFFFFFFFF);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
                   
    LED_R(0);
    LED_G(0);
    
    return 0;

}

/*
This function is used to send sync message and only called in anchor mode
*/
static int send_uwb_sync()
{
    rx_time_ = 0;  //reset the rx_time_ to 0 as it's the start of a new superframe in anchor mode
    slot_cnt_ = 0;
    
    tx_msg_.frame_type = 0x00;  //sync message with round_cnt_ set to 0
    tx_msg_.tag_sn = 0; //set the tag SN to 0 for now
    tx_msg_.frame_cnt = frame_cnt_;
    tx_msg_.slot_cnt = slot_cnt_;
    tx_msg_.t_reply = 0;

    send_uwb_frame();
   
    return 0;
}

/*
This function is used to send blink message and only called in tag mode
*/
static int send_uwb_blink()
{
    tx_msg_.frame_type = 0x10 + round_cnt_;  //blink message
    tx_msg_.tag_sn = 0; //set the tag SN to 0 for now
    tx_msg_.frame_cnt = frame_cnt_;
    tx_msg_.slot_cnt = slot_cnt_;

    send_uwb_frame();
    return 0;
}

/*
This function is used to send response message and only called in anchor mode
*/
static int send_uwb_response()
{
    tx_msg_.frame_type = 0x20 + round_cnt_;  //response message
    tx_msg_.tag_sn = 0; //set the tag SN to 0 for now
    tx_msg_.frame_cnt = frame_cnt_;
    tx_msg_.slot_cnt = slot_cnt_rcvd_;

    send_uwb_frame();
    return 0;
}

static void tx_ok_cb(const dwt_cb_data_t *cb_data)
{
    if (statusData.role == ROLE_TAG)
        LED_R_TOGGLE();
}

static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
//    int goodSts = 0; /* Used for checking STS quality in received signal */
//    int16_t stsQual; /* This will contain STS quality index */
    
    (void)cb_data;

    rx_time_ = 0;
    uint16_t frame_len = dwt_getframelength();
    if(frame_len >= 128) { //not a valid message
        printf("invalid message with length %d", frame_len);
        return;
    }

    dwt_readrxdata((uint8_t *)&rx_msg_, frame_len, 0);
    uint8_t frame_type = rx_msg_.frame_type & 0xFE;
    round_cnt_ = rx_msg_.frame_type & 0x01; //reset round_cnt_ every time received the sync message
    uwbRes.offset[rx_msg_.slot_cnt] = dwt_readclockoffset();
    if(statusData.role == ROLE_TAG) {
        if(frame_type == 0x00) { //sync message
            //reset the system clock as a new super frame started
            INTC_ClearPendingIRQ(SysTick_IRQn);
            SystemCoreClockUpdate();
            SysTick_Config(SystemCoreClock / 1000 * SLOT_LENGTH_IN_MS);
            frame_cnt_ = rx_msg_.frame_cnt;
            slot_cnt_ = 0;
            tx_time_ = 0;
            rx_time_ = 0;
            synced_ = 1;
        }else if(frame_type == 0x10) {
            //blink message, do nothing  in tag mode
        }else if(frame_type == 0x20) { //response message, process it in tag mode
            dwt_readrxtimestamp((uint8_t *)&rx_time_);
            if(rx_msg_.frame_cnt == frame_cnt_ && rx_msg_.slot_cnt == slot_cnt_) {
                clkbias_local_ = rx_time_ - tx_time_;
                clkbias_match_ = rx_msg_.t_reply;
                if(round_cnt_ == 0) {
                    pdoa1_ = dwt_readpdoa();
                    RFswitch(2);
                } else {
                    pdoa2_ = dwt_readpdoa();
                    RFswitch(1);
                }
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                round_cnt_ = round_cnt_==0?1:0;
                if(round_cnt_) {
                    send_uwb_blink();
                }
                if (clkbias_match_ != 0) { // calc range if matchmate clk bias avaiable
                    float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
                    float tof = ((clkbias_local_ - clkbias_match_ * (1 - clockOffsetRatio)) / 2.0 - 16385) * DWT_TIME_UNITS;
                    float distance = tof * SPEED_OF_LIGHT * 1000 - 242;
                    uwbRes.range[slot_cnt_] = (short)distance;
                    calculate_aoa();
                }
                if (round_cnt_ == 0) {
                    rx_time_ = 0; //in case the tag will occupy more than one slot in a super frame
                    statusData.cb_meas_done(frame_cnt_, slot_cnt_);
                    //printf("[tag]frame:%d,slot:%d,round:%d,pdoa1:%d,pdoa2:%d,range:%d, aoa:%d, %d,%d",
                    //        frame_cnt_, slot_cnt_, round_cnt_, pdoa1_, pdoa2_, uwbRes.range[slot_cnt_], uwbRes.aoa[slot_cnt_], clkbias_local_, clkbias_match_);
                }
            }else{
                //tx_msg_.t_reply = 0;
                rx_time_ = 0;
            }
        }
    }else if(statusData.role == ROLE_ANCHOR) {
        if(frame_type == 0x00) {
            //sync message, do nothing
        }else if(frame_type == 0x10) {
            //blink message, process it in anchor mode
            dwt_readrxtimestamp((uint8_t *)&rx_time_);
            if(rx_msg_.frame_cnt == frame_cnt_) {
                slot_cnt_rcvd_ = rx_msg_.slot_cnt;
                clkbias_local_ = rx_time_ - tx_time_;
                clkbias_match_ = rx_msg_.t_reply;
                if(round_cnt_ == 0) {
                    pdoa1_ = dwt_readpdoa();
                    send_uwb_response(); //send the first round resposne using the first antenna??
                    RFswitch(2);
                } else {
                    pdoa2_ = dwt_readpdoa();
                    send_uwb_response(); //send the second round response using the second antenna??
                    RFswitch(1);
                }
                round_cnt_ = round_cnt_==0?1:0;
                if (clkbias_match_ != 0) { // calc range if matchmate clk bias avaiable
                    float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
                    float tof = ((clkbias_local_ - clkbias_match_ * (1 - clockOffsetRatio)) / 2.0 - 16385) * DWT_TIME_UNITS;
                    float distance = tof * SPEED_OF_LIGHT * 1000 - 242;
                    uwbRes.range[slot_cnt_rcvd_] = (short)distance;
                    calculate_aoa();
                    statusData.cb_meas_done(frame_cnt_, slot_cnt_rcvd_);
									
										// test 3 nodes
										frame_cnt_test[tag_cnt]=frame_cnt_;
										slot_cnt_test[tag_cnt]=slot_cnt_rcvd_;
										range_test[tag_cnt]=uwbRes.range[slot_cnt_rcvd_];
										aoa_test[tag_cnt]=uwbRes.aoa[slot_cnt_rcvd_];								
										tag_cnt++;
										if (tag_cnt>tag_num-1) 
										{
											
											for(int i=0;i<tag_num;i++)
											{
												//printf("\r\n[anchor]frame:%d,slot:%d,round:%d,pdoa1:%d,pdoa2:%d,range:%d, aoa:%d",
                          //frame_cnt_, slot_cnt_, round_cnt_, pdoa1_, pdoa2_, uwbRes.range[slot_cnt_], uwbRes.aoa[slot_cnt_]);
												//printf("\r\n[anchor]frame:%d,slot:%d,range:%d, aoa:%d, record_num:%d",
                          //frame_cnt_test[i], slot_cnt_test[i],range_test[i], aoa_test[i],record_cnt);
											}
											tag_cnt=0;
											record_cnt++;
											
										}
										//
									
                    //printf("[anchor]frame:%d,slot:%d,round:%d,pdoa1:%d,pdoa2:%d,range:%d, aoa:%d, %d,%d",
                    //        frame_cnt_, slot_cnt_, round_cnt_, pdoa1_, pdoa2_, uwbRes.range[slot_cnt_], uwbRes.aoa[slot_cnt_], clkbias_local_, clkbias_match_);
                }
            }else{
                // tx_msg_.t_reply = 0;
                rx_time_ = 0;
            }
        }else if (frame_type == 0x20) {
            //response message, do nothing
        }
    }else{
        printf("invalid role.");
    }

    return;
}

static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

int uwbLib_init(char *authCode, char role, char *SN, char preambleLen, char dataRate, char stsLen, char pdoaMode, short pdoaCal1, short pdoaCal2, char *uwbUserTxBuf, char *uwbUserRxBuf, uwb_cb_t callback) 
{
    int ret = set_uwb_parameter(authCode, role, SN, preambleLen, dataRate, stsLen, pdoaMode, pdoaCal1, pdoaCal2, uwbUserTxBuf, uwbUserRxBuf, callback);
    if (ret != 0) {
        return ret;
    }
    
    SPI_INIT(SPI_MODE_0, 1);
    IO_INIT();
    LED_R(1);
    Sleep(500);
    LED_R(0);
    LED_G(1);
        
    RFswitch(1);

    reset_DWIC();
    Sleep(2);
    if (dwt_probe() != DWT_SUCCESS) {
        return -101;
    }
        
    unsigned char checkcnt = 0;
    do {
        checkcnt++;
    } while (!dwt_checkidlerc() && checkcnt < 50);
    
    if (checkcnt >= 50) {
//        return -102;    
    }
        
    if (dwt_initialise(DWT_DW_IDLE /*| DWT_READ_OTP_PID*/) == DWT_ERROR) {
        return -103;
    }
        
    if (dwt_configure(&statusData.uwbConfig)) {
        return -104;
    }
        
    dwt_configuretxrf(&statusData.uwbTxConfig);
        
    dwt_setcallbacks(tx_ok_cb, rx_ok_cb, rx_err_cb, rx_err_cb, NULL, NULL);

    dwt_setinterrupt(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | DWT_INT_TXFRS_BIT_MASK, 0, DWT_ENABLE_INT);

    dwt_writesysstatuslo(DWT_INT_RCINIT_BIT_MASK | DWT_INT_SPIRDY_BIT_MASK);
    
    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);

    port_set_dwic_isr(dwt_isr);
    
    dwt_setrxtimeout(0);
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(0);
    
    LED_G(0);
    
    ret = 0;
    
    dwt_writesysstatuslo(0xFFFFFFFF);
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    return ret;
}

int uwbLib_run_slot(void)
{
    slot_cnt_ ++;
    if(slot_cnt_ >= SLOT_NUM_IN_FRAME) {
        slot_cnt_ = 0;
        frame_cnt_ ++;
    }
    if(statusData.role == ROLE_ANCHOR) {
        if(slot_cnt_ == 0) {
            //printf("new frame:%d", frame_cnt_);
            send_uwb_sync();
        }
    }else if(statusData.role == ROLE_TAG) {
			/*
        if(slot_cnt_ == assigned_slot_
            || slot_cnt_ == assigned_slot_ + 10  //this is for test
            || slot_cnt_ == assigned_slot_ + 20  //this is for test
            && synced_) {
		  */
				if(slot_cnt_ == assigned_slot_
						&& synced_) {
            send_uwb_blink();
        }
    }
    return 0;
}

int uwbLib_callback(void) {
    __set_PRIMASK(1);
    if (statusData.verifyRes != 0) {
        return -1;
    }
    process_deca_irq();
    __set_PRIMASK(0);
    if (uwbRes.flag) {
        return 2;
    } else {
        return 0;
    }
}
