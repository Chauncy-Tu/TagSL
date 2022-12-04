/****************************************************************/
/*  macros for the global project */

// TDMA
#define SLOT_LENGTH_IN_MS 5
#define FRAME_LENGTH_IN_S 1
#define SLOT_NUM_IN_FRAME FRAME_LENGTH_IN_S*1000/SLOT_LENGTH_IN_MS
// AGENT ROLE
#define Master_Anchor 0
#define Slave_Anchor  1
#define Tag           2
// TX DELAY
#define BLINK_DELAY    500   // ¦Ìs
#define RESPONSE_DELAY 500
#define SEQUENCE_DELAY 300



#pragma pack(1)

struct uwb_msg{

    uint8_t frame_type_;    //bit7~1: frame type, 0x00: sync; 0x10: blink, 0x20: response; 
                            //bit0: round counter, 0:first round, 1:second round
    
    uint8_t frame_cnt;     //frame counter, increase by 1 every second for now
    uint8_t slot_cnt;

    uint8_t tx_id_;  // the id of the transmitting agent
    uint8_t mt_id_;  // in slave anchor,id of the
    
    uint8_t temp1;
    uint8_t temp2;
    uint8_t temp3;

    float pos_[2];   // position of the tx agent
    uint8_t slave_[4];  // slave_id
    
    int32_t         t_reply;       //tx-rx
    uint8_t         user_data[10];  //resered for futuer use, such as the timing sync among multiple anchors
    uint8_t         crc[2];


};

struct Agent{
    char role_;
    uint8_t  id_;

    uint8_t slot_num_;

    float pTrue_[2];
    float p_[2];
    float vTrue_[2];

    struct uwb_msg tx_msg_,rx_msg_;

    int slave_[4];
  
};


void agent_run_slot();

struct Agent agent_init(char role,uint8_t id);

void agent_sync();