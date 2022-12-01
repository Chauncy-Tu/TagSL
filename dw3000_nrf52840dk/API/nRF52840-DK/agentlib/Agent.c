#include "string.h"
#include "math.h"
#include <stdio.h>
#include <deca_device_api.h>
#include <deca_spi.h>
//#include <deca_vals.h>
#include <port.h>

#include <stdlib.h>
#include <deca_regs.h>
#include <deca_types.h>
#include <config_options.h>
#include <Agent.h>



uint8_t frame_cnt_ = 0;
uint8_t slot_cnt_ = 0;



struct Agent agent_init(char role,uint8_t id)
{
  struct Agent agent;
  agent.role_=role;
  agent.id_=id;
  agent.slot_num=0;

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
    }


    
}