/****************************************************************/
/*  macros for the global project */

#include <stdlib.h>
#include <math.h>
#include <stdio.h>

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16370
#define RX_ANT_DLY 16370

// TDMA
#define SLOT_LENGTH_IN_MS 5
#define FRAME_LENGTH_IN_S 1
#define SLOT_NUM_IN_FRAME FRAME_LENGTH_IN_S*1000/SLOT_LENGTH_IN_MS
// AGENT ROLE
#define Master_Anchor 0
#define Slave_Anchor  1
#define Tag           2
#define Sniffer       3
// TX DELAY
#define BLINK_DELAY    1500   // ¦Ìs
#define RESPONSE_DELAY 1000
#define SEQUENCE_DELAY 500

#define UUS_TO_DWT_TIME 63898
#define SPEED_OF_LIGHT   (299702547)


#pragma pack(1)

struct uwb_msg{

    uint8_t frame_type_;    //bit7~1: frame type, 0x00: sync; 0x10: blink, 0x20: response; 
                            //bit0: round counter, 0:first round, 1:second round
    
    uint8_t frame_cnt_;     //frame counter, increase by 1 every second for now
    uint8_t slot_cnt_;

    uint8_t tx_id_;  // the id of the transmitting agent
    uint8_t mt_id_;  // in slave anchor,id of the master anchor
    
    uint8_t temp1_;
    uint8_t temp2_;
    uint8_t temp3_;

    float pos_[2];   // position of the tx agent
    uint8_t slave_[4];  // slave_id
    
    int32_t         t_reply_;       //tx-rx
    uint8_t         user_data_[10];  //resered for futuer use, such as the timing sync among multiple anchors
    uint8_t         crc_[2];


};

struct Agent{
    char role_;
    uint8_t  id_;

    uint8_t slot_num_;

    float pTrue_[2];
    float p_[2];
    float vTrue_[2];

    struct uwb_msg tx_msg_,rx_msg_;

    int64_t tx_time_,rx_time_;

    uint8_t slave_[4];

    float map_[100][2];  //global map£ºposition

    bool p_new;         //position new

  
};


void agent_run_slot();

struct Agent agent_init(char role,uint8_t id);

void agent_sync();

void UCarray_copy(uint8_t *a,uint8_t *b,int len);
void Iarray_copy(int *a,int *b,int len);
void Farray_copy(float *a,float *b,int len);

void Fang_tdoa(void);
void tdoa_clear(void);

void Matrix_matmul(float *MatInput1,float *MatInput2,float *MatOutput, int m,int n,int p);
void Matrix_transpose(float *MatInput,float *MatOutput,int m,int n);
void Matrix_inverse(float *MatInput, float *MatOutput, int m,int n);
void matrix_printf(float **a);
float dist(float *p1, float *p2);