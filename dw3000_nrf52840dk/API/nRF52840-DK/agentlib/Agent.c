#include "string.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <config_options.h>
#include <Agent.h>




uint8_t frame_cnt_ = 0;
uint8_t slot_cnt_ = 0;
uint32_t status_reg;

extern struct Agent agent;  // extern val, defined in main






/* This function is used to send the data in tx_msg_ with dw3000 */

static int send_uwb_frame_immediate()
{
    //dwt_writesysstatuslo(0xFFFFFFFF);
    dwt_forcetrxoff();  
    
    dwt_writetxdata(sizeof(agent.tx_msg_) - 2, (uint8_t *)&agent.tx_msg_, 0);
    dwt_writetxfctrl(sizeof(agent.tx_msg_), 0, 1);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        //dwt_writesysstatuslo(0xFFFFFFFF);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }

    
    return 0;

}





static int send_uwb_sync()
{
    //rx_time_ = 0;  //reset the rx_time_ to 0 as it's the start of a new superframe in anchor mode
    //slot_cnt_ = 0;
    
    agent.tx_msg_.frame_type_ = 0x00;    //sync message with round_cnt_ set to 0
    agent.tx_msg_.tx_id_ = agent.id_;   
    agent.tx_msg_.frame_cnt = frame_cnt_;
    agent.tx_msg_.slot_cnt = slot_cnt_;
    agent.tx_msg_.t_reply = 0;	
    send_uwb_frame_immediate();

    return 0;
   
}

static int send_uwb_blink()
{
}






struct Agent agent_init(char role,uint8_t id)
{
  struct Agent agent;
  agent.role_=role;
  agent.id_=id;
  agent.slot_num_=0;

  for(int i=0;i<2;i++)
  {
    agent.pTrue_[i]=0;
    agent.p_[i]=0;
    agent.pTrue_[i]=0;  
  }
  return agent;
}



void agent_run_slot(void)
{

 
    slot_cnt_ ++;
    if(slot_cnt_ >= SLOT_NUM_IN_FRAME)
    {
      slot_cnt_ = 0;
      frame_cnt_ ++;
      //send_uwb_sync();      		
    }

    if(agent.role_==Master_Anchor)
    {
      if(slot_cnt_==1)
      {
        send_uwb_sync();
      }
      else if(slot_cnt_==agent.slot_num_)
      {
        send_uwb_blink();
      }
      
    }
    else if(agent.role_==Slave_Anchor)
    {
      while(1)
      {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR )))
        { };

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
          printf("rx_ok");
        }
      }
      
      

    }
    else if(agent.role_==Tag)
    {
      

    }


    dwt_rxenable(DWT_START_RX_IMMEDIATE);

      /* Poll until a frame is properly received or an error/timeout occurs. See NOTE 3 below.
       * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
       * function to access it. */

       
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR )))
      { };

       
      

      if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
      {
          
          /* A frame has been received, copy it to our local buffer. */
          int frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
          printf("\n%s%d","frame_len:",frame_len);
      }
      

   

    

    
    

    //printf("\r\nagentID:%d",agent.id_);
}
