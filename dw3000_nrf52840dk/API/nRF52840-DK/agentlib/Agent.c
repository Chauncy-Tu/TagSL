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
//#include <Mycmatrix.h>
#include <cmatrix.h>
#include "app_usbd.h" 
#include "user_usbd.h"

uint8_t response_cnt_=0;
uint8_t frame_cnt_ = 0;
uint8_t slot_cnt_ = 0;
uint32_t status_reg;
uint8_t resp[4]={0,0,0,0};
int64_t rx_times_[4];

int64_t sniffer_rx_time[6]={0,0,0,0,0,0};
int sniffer_rx_cnt=0;


static struct M_RX{
  uint8_t m_id;
  int64_t rx_time;
};
static struct S_RX{
  uint8_t m_id;
  uint8_t s_id;
  int64_t rx_time;
};

static struct TDOA{
  uint8_t m_id;
  uint8_t s_id;
  int64_t tdoa;
};

static int m_rx_cnt=0;
static int s_rx_cnt=0;
static int tdoa_cnt=0;

static struct M_RX m_rx_array[4];
static struct S_RX s_rx_array[4];
static struct TDOA tdoa_array[4];

int tdoa_sum_cnt=0;
int64_t tdoa_cal=0;

int32_t t_reply_array[4];



static int slot_cnt_record[6]={0,0,0,0,0,0};
int slot_cnt_record_cnt=0;


extern struct Agent agent;  // extern val, defined in main




/* This function is used to send the data in tx_msg_ with dw3000 */

static int send_uwb_frame_immediate()
{
    dwt_forcetrxoff();  
    dwt_writetxdata(sizeof(agent.tx_msg_) - 2, (uint8_t *)&agent.tx_msg_, 0);
    dwt_writetxfctrl(sizeof(agent.tx_msg_), 0, 1);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
    
    return 0;
}


static int send_uwb_test()
{
    dwt_forcetrxoff(); 
    agent.tx_msg_.frame_type_ = 0x03;       // sync msg

    agent.tx_msg_.frame_cnt_ = frame_cnt_;
    agent.tx_msg_.slot_cnt_ = slot_cnt_;
    agent.tx_msg_.t_reply_ = 0;
    agent.tx_msg_.tx_id_ = agent.id_;
    
    Farray_copy(agent.tx_msg_.pos_,agent.pTrue_,sizeof(agent.pTrue_)/sizeof(agent.pTrue_[0]));   
     
    dwt_writetxdata(sizeof(agent.tx_msg_) - 2, (uint8_t *)&agent.tx_msg_, 0);
    dwt_writetxfctrl(sizeof(agent.tx_msg_), 0, 1);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
    
    return 0;
   
}


static int send_uwb_sync()
{
    dwt_forcetrxoff(); 
    agent.tx_msg_.frame_type_ = 0x00;       // sync msg

    agent.tx_msg_.frame_cnt_ = frame_cnt_;
    agent.tx_msg_.slot_cnt_ = slot_cnt_;
    agent.tx_msg_.t_reply_ = 0;
    agent.tx_msg_.tx_id_ = agent.id_;
    
    Farray_copy(agent.tx_msg_.pos_,agent.pTrue_,sizeof(agent.pTrue_)/sizeof(agent.pTrue_[0]));   
     
    dwt_writetxdata(sizeof(agent.tx_msg_) - 2, (uint8_t *)&agent.tx_msg_, 0);
    dwt_writetxfctrl(sizeof(agent.tx_msg_), 0, 1);
    if(dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
    }
    
    return 0;
   
}

static int send_uwb_blink()
{
    dwt_forcetrxoff();  
    agent.tx_msg_.frame_type_ = 0x01;       // blink msg
    agent.tx_msg_.frame_cnt_ = frame_cnt_;
    agent.tx_msg_.slot_cnt_ = slot_cnt_;
    agent.tx_msg_.t_reply_ = 0;
    agent.tx_msg_.tx_id_ = agent.id_;


    Farray_copy(agent.tx_msg_.pos_,agent.pTrue_,sizeof(agent.pTrue_)/sizeof(agent.pTrue_[0]));
    UCarray_copy(agent.tx_msg_.slave_,agent.slave_,sizeof(agent.slave_)/sizeof(agent.slave_[0]));

    uint64_t systime=0;
    systime = (uint64_t)dwt_readsystimestamphi32() << 8;
    uint32_t delaytxtime = ((systime + (BLINK_DELAY * UUS_TO_DWT_TIME)) >> 8) & 0xFFFFFF00;
    dwt_setdelayedtrxtime(delaytxtime);
    
    dwt_writetxdata(sizeof(agent.tx_msg_) - 2, (uint8_t *)&agent.tx_msg_, 0);
    dwt_writetxfctrl(sizeof(agent.tx_msg_), 0, 1);
    if (dwt_starttx(DWT_START_TX_DELAYED != DWT_SUCCESS)) {
    }
    
    return 0;
}

static int send_uwb_response(uint8_t mt_id, int seq)
{
    dwt_forcetrxoff();
    agent.tx_msg_.frame_type_ = 0x02;       // blink msg
    agent.tx_msg_.frame_cnt_ = frame_cnt_;
    agent.tx_msg_.slot_cnt_ = slot_cnt_;
      
    agent.tx_msg_.tx_id_ = agent.id_;
    agent.tx_msg_.mt_id_ = mt_id;

    Farray_copy(agent.tx_msg_.pos_,agent.pTrue_,sizeof(agent.pTrue_)/sizeof(agent.pTrue_[0]));
    
    //int64_t temp1=(agent.rx_time_ + ((RESPONSE_DELAY+seq*SEQUENCE_DELAY) * UUS_TO_DWT_TIME));
    uint32_t delaytxtime = ((agent.rx_time_ + ((RESPONSE_DELAY+seq*SEQUENCE_DELAY) * UUS_TO_DWT_TIME)) >> 8) & 0xFFFFFF00;
   
    dwt_setdelayedtrxtime(delaytxtime);
    agent.tx_time_ = ((uint64_t)(delaytxtime& 0xFFFFFFFEUL) << 8)+TX_ANT_DLY ;

    agent.tx_msg_.t_reply_ = (agent.rx_time_)? agent.tx_time_ - agent.rx_time_ : 0;
    // note:agent.tx_time_ specifies the time to specifies the time at which to send/start receiving, when the system time reaches this time (minus
    // the times it needs to send preamble etc.) then the sending of the frame begins.  The actual time at
    // which the frame??s RMARKER transits the antenna (the standard TX timestamp event) is given by the
    // starttime + the transmit antenna delay.


    //int32_t temp2=agent.tx_time_ - agent.rx_time_;
    dwt_writetxdata(sizeof(agent.tx_msg_) - 2, (uint8_t *)&agent.tx_msg_, 0);
    dwt_writetxfctrl(sizeof(agent.tx_msg_), 0, 1);

    if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE); //TODO
    }
    
    //nrf_delay_ms(1);
    //dwt_readtxtimestamp((uint8_t *)&agent.tx_time_);
    //int32_t temp3=agent.tx_time_- agent.rx_time_;

    //int temp4=0;

    return 0;
}

static int send_uwb_pos()
{
    dwt_forcetrxoff();  
    agent.tx_msg_.frame_type_ = 0x03;       // blink msg
    agent.tx_msg_.frame_cnt_ = frame_cnt_;
    agent.tx_msg_.slot_cnt_ = slot_cnt_;
    agent.tx_msg_.t_reply_ = 0;
    agent.tx_msg_.tx_id_ = agent.id_;


    Farray_copy(agent.tx_msg_.pos_,agent.p_,sizeof(agent.p_)/sizeof(agent.p_[0]));
    
    dwt_writetxdata(sizeof(agent.tx_msg_) - 2, (uint8_t *)&agent.tx_msg_, 0);
    dwt_writetxfctrl(sizeof(agent.tx_msg_), 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    //if (dwt_starttx(DWT_START_TX_IMMEDIATE != DWT_SUCCESS)) {
    //}
    
    return 0;
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
  }

  agent.p_new=0;
  return agent;
}

void agent_run_slot(void)
{
    slot_cnt_ ++;
    if(slot_cnt_ >= SLOT_NUM_IN_FRAME)
    {
      slot_cnt_ = 0;
      frame_cnt_ ++;
      //printf("\r\nframe_cnt:%d",frame_cnt_);
      //printf("\r\nresponse_cnt:%d",response_cnt_);
      //response_cnt_=0;

      //for(int i=0;i<4;i++)
      //{
      //  printf("\r\n id:%d",resp[i]);
      //  resp[i]=0;
      //}
      //for(int i=0;i<4;i++)
      //{
      //  printf("\r\n rx_time:%lld",rx_times_[i]);
      //  rx_times_[i]=0;
      //}
        
      
    }
    

    if(agent.role_==Master_Anchor)
    {
      if(slot_cnt_==0)
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
    }
    else if(agent.role_==Tag)
    {
      if(slot_cnt_==10*agent.id_)
      {

        send_uwb_pos();


        //printf("\r\nframe_cnt:%d",frame_cnt_);
        //printf("\r\nresponse_cnt:%d",response_cnt_);
        response_cnt_=0;

        

        //for(int i=0;i<4;i++)
        //{
        //  printf("\r\n id:%d",resp[i]);
        //  resp[i]=0;
        //}
        //for(int i=0;i<4;i++)
        //{
        //  printf("\r\n rx_time:%lld",rx_times_[i]);
        //  rx_times_[i]=0;
        //}


        //for(int i=0;i<6;i++)
        //{
        //  printf("\r\nslot_cnt_record%d:%d",i,slot_cnt_record[i]);
          
        //}
        //for(int i=0;i<6;i++)
        //{
        //  slot_cnt_record[i]=0;
        //}
        slot_cnt_record_cnt=0;
      }

    }
    else if(agent.role_==Sniffer)
    {
      //if(slot_cnt_==20)
      //{
      //  printf("\r\nframe_cnt:%d",frame_cnt_);
      //  for(int i=0;i<6;i++)
      //  {
      //    printf("\r\nrx_time(%d)=%lld;",i+1,(sniffer_rx_time[i]));
          
      //  }
      //  for(int i=0;i<6;i++)
      //  {
      //    sniffer_rx_time[i]=0;
      //  }
      //  for(int i=0;i<6;i++)
      //  {
      //    printf("\r\nslot_cnt_record(%d)=%d;",i,slot_cnt_record[i]);
          
      //  }
      //  for(int i=0;i<6;i++)
      //  {
      //    slot_cnt_record[i]=0;
      //  }
      //  slot_cnt_record_cnt=0;
        
      //  sniffer_rx_cnt=0;
      //}

    }


    


  
}

void rx_ok_cb(const dwt_cb_data_t *cb_data)
{

  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);
  uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
  if(frame_len >= 128) 
  { //not a valid message
      printf("invalid message with length %d", frame_len);
      return;
  }
  dwt_readrxtimestamp((uint8_t *)&agent.rx_time_);
  dwt_readrxdata((uint8_t *)&agent.rx_msg_, frame_len, 0);


  uint8_t frame_type = agent.rx_msg_.frame_type_;
  uint8_t tx_id=agent.rx_msg_.tx_id_;
  uint8_t mt_id=agent.rx_msg_.mt_id_;

  //printf("\r\nclockOffsetRatio%d:%lf",tx_id,(double)(clockOffsetRatio));
  uint8_t slave[4];
  UCarray_copy(slave, agent.rx_msg_.slave_,sizeof(agent.rx_msg_.slave_)/sizeof(agent.rx_msg_.slave_[0]));


  if(agent.role_==Slave_Anchor)
  {
    if(frame_type==0x00)
    {

      NRFX_IRQ_PENDING_CLEAR(SysTick_IRQn);
      SystemCoreClockUpdate();
      SysTick_Config(SystemCoreClock / 1000 * SLOT_LENGTH_IN_MS);
      slot_cnt_=0;
      frame_cnt_=agent.rx_msg_.frame_cnt_;

      
    }else if(frame_type==0x01)
    {
      //printf("\r\nblink rx ok");
      for(int i=0;i<4;i++)
      {
        if(slave[i]==agent.id_)
        {
          send_uwb_response(tx_id,i);
        }
      }
    }
    
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }
  else if(agent.role_==Tag)
  {

    if(slot_cnt_record_cnt>=0&&slot_cnt_record_cnt<6)
    {
      slot_cnt_record[slot_cnt_record_cnt]=agent.rx_msg_.slot_cnt_;
      slot_cnt_record_cnt++;
    }

    
    if(frame_type==0x00)  // sync frame
    {
      NRFX_IRQ_PENDING_CLEAR(SysTick_IRQn);
      SystemCoreClockUpdate();
      SysTick_Config(SystemCoreClock / 1000 * SLOT_LENGTH_IN_MS);   
      slot_cnt_=0;
      frame_cnt_=agent.rx_msg_.frame_cnt_;

      Farray_copy(agent.map_[tx_id],agent.rx_msg_.pos_,sizeof(agent.rx_msg_.pos_)/sizeof(agent.rx_msg_.pos_[0]));
      
    }else if(frame_type==0x01) // blink frame
    {
      if(m_rx_cnt<4&&m_rx_cnt>=0)
      {
        m_rx_array[m_rx_cnt].m_id=tx_id;
        m_rx_array[m_rx_cnt].rx_time=agent.rx_time_;
        m_rx_cnt++;
      }else{
        m_rx_cnt=0;
      }
      Farray_copy(agent.map_[tx_id],agent.rx_msg_.pos_,sizeof(agent.rx_msg_.pos_)/sizeof(agent.rx_msg_.pos_[0]));
      //printf("\r\nblink rx");
      
    }else if(frame_type==0x02)  //response frame
    {
      rx_times_[response_cnt_]=agent.rx_time_;  //TODO
      resp[response_cnt_]=tx_id;
      response_cnt_++;
      int32_t t_reply=agent.rx_msg_.t_reply_;
      Farray_copy(agent.map_[tx_id],agent.rx_msg_.pos_,sizeof(agent.rx_msg_.pos_)/sizeof(agent.rx_msg_.pos_[0]));


      for(int i=0;i<m_rx_cnt;i++)
      {
        if(m_rx_array[i].m_id==mt_id)
        {
          if(s_rx_cnt<4&&s_rx_cnt>=0)
          {
            s_rx_array[s_rx_cnt].m_id=mt_id;
            s_rx_array[s_rx_cnt].s_id=tx_id;
            s_rx_array[s_rx_cnt].rx_time=agent.rx_time_;
            s_rx_cnt++;
          }else{
            s_rx_cnt=0;
          }

          int64_t m_rxtime=m_rx_array[i].rx_time;
          int64_t s_rxtime=agent.rx_time_;
          
       

          if(tdoa_cnt<4&&tdoa_cnt>=0)
          {
            tdoa_array[tdoa_cnt].m_id=mt_id;
            tdoa_array[tdoa_cnt].s_id=tx_id;

            tdoa_array[tdoa_cnt].tdoa=((s_rxtime-m_rxtime)-(1.0-clockOffsetRatio)*t_reply);
            //tdoa_array[tdoa_cnt].tdoa=((s_rxtime-m_rxtime)-t_reply);

            if(tx_id==2)
            {
              int temp=1;
            }
            //TODO   cfo + or -
            //printf("\r\n tdoa:%lld",tdoa_array[tdoa_cnt].tdoa);
            
            //tdoa_cal=tdoa_array[tdoa_cnt].tdoa;
            
            tdoa_cnt++;
            
          }else{
            tdoa_cnt=0;
          }

          if(tdoa_cnt==4)
          {
            Fang_tdoa();
            tdoa_clear();

            break;
          }
          
        }
      }
    }
      
      

      

      
    }
    else if (agent.role_==Sniffer)
    {
      //if(frame_type==0x00)  // sync frame
      //{
      //  NRFX_IRQ_PENDING_CLEAR(SysTick_IRQn);
      //  SystemCoreClockUpdate();
      //  SysTick_Config(SystemCoreClock / 1000 * SLOT_LENGTH_IN_MS);   
      //  slot_cnt_=0;
      //  frame_cnt_=agent.rx_msg_.frame_cnt_; 
      //}
      //if(sniffer_rx_cnt>=0&&sniffer_rx_cnt<6)
      //{
      //sniffer_rx_time[sniffer_rx_cnt]=agent.rx_time_;
      //sniffer_rx_cnt++;
      //}
      //if(slot_cnt_record_cnt>=0&&slot_cnt_record_cnt<6)
      //{
      //  slot_cnt_record[slot_cnt_record_cnt]=agent.rx_msg_.slot_cnt_;
      //  slot_cnt_record_cnt++;
      //}


      if(frame_type==0x03)
      {
        Farray_copy(agent.map_[tx_id],agent.rx_msg_.pos_,sizeof(agent.rx_msg_.pos_)/sizeof(agent.rx_msg_.pos_[0]));
        printf("\r\n%d:%f %f",tx_id,agent.rx_msg_.pos_[0],agent.rx_msg_.pos_[1]);
      }

    }
    
    
  
  
 
  


}
void rx_to_cb(const dwt_cb_data_t *cb_data)
{
  (void)cb_data;
  //printf("\r\n rx_to");
}
void rx_err_cb(const dwt_cb_data_t *cb_data)
{
  (void)cb_data;
  //printf("\r\n rx_err");
}
void tx_ok_cb(const dwt_cb_data_t *cb_data)
{
  (void)cb_data;
  //printf("\r\n tx_ok");
}

void UCarray_copy(uint8_t *a,uint8_t *b,int len)  // a:destination b:source 
{
  for(int i=0;i<len;i++)
  {
    a[i]=b[i];
  }
}

void Iarray_copy(int *a,int *b,int len)  // a:destination b:source 
{
  for(int i=0;i<len;i++)
  {
    a[i]=b[i];
  }
}

void Farray_copy(float *a,float *b,int len)  // a:destination b:source 
{
  for(int i=0;i<len;i++)
  {
    a[i]=b[i];
  }
}

void Fang_tdoa(void)
{

  
  double A[4][3],b[4];
  double ri1[4],Ki1[4];

  //double A[4][3]={-100,100,99.2797,100,100,-48.3271,100,-100,31.7534,-100,-100,135.3482};
  //double b[4]={2535.8866,4.416121928898527e+03,4.747930262545749e+03,4.202134211714574e+02};

  //float t1[4]={25651,9921,18455,29495};
  //agent.map_[1][0]=0;
  //agent.map_[1][1]=0;
  //agent.map_[2][0]=-50;
  //agent.map_[2][1]=50;
  //agent.map_[3][0]=50;
  //agent.map_[3][1]=50;
  //agent.map_[4][0]=50;
  //agent.map_[4][1]=-50;
  //agent.map_[5][0]=-50;
  //agent.map_[5][1]=-50;


  //for(int i=0;i<4;i++)
  //{
  //  tdoa_array[i].tdoa=t1[i];
  //}
  



  for(int i=0;i<4;i++)
  {
    float origin[2]={0,0};
    

    ri1[i]=SPEED_OF_LIGHT*tdoa_array[i].tdoa*DWT_TIME_UNITS-dist(agent.map_[tdoa_array[i].m_id],agent.map_[tdoa_array[i].s_id]);
    Ki1[i]=dist(agent.map_[tdoa_array[i].s_id],origin)*dist(agent.map_[tdoa_array[i].s_id],origin)-dist(agent.map_[tdoa_array[i].m_id],origin)*dist(agent.map_[tdoa_array[i].m_id],origin);
    A[i][0]=2*(agent.map_[tdoa_array[i].s_id][0]-agent.map_[tdoa_array[i].m_id][0]);
    A[i][1]=2*(agent.map_[tdoa_array[i].s_id][1]-agent.map_[tdoa_array[i].m_id][1]);
    A[i][2]=2*ri1[i];

    b[i]=Ki1[i]-ri1[i]*ri1[i];
  }

  matrix *Mat_A, *Mat_AT, *Mat_ATA_inv, *Mat_b,*Mat_temp1,*Mat_temp2,*Mat_temp3;
  Mat_A = Mnew(4, 3);
  Mat_b = Mnew(4, 1);
  for(int i=0;i<4;i++)
  {

    Mat_b->A[i][0]=b[i];
    for(int j=0;j<3;j++)
    {
      Mat_A->A[i][j]=A[i][j];
    }
  }
  Mat_AT=Mtrans(Mat_A);
  Mat_temp1=Mmulti(Mat_AT,Mat_A);
  Mat_ATA_inv=Minv(Mat_temp1);
  Mat_temp2=Mmulti(Mat_ATA_inv,Mat_AT);
  Mat_temp3=Mmulti(Mat_temp2,Mat_b);
  for(int i=0;i<2;i++)
  {
    agent.p_[0]=(float)(Mat_temp3->A[0][0]);
    agent.p_[1]=(float)(Mat_temp3->A[1][0]);
  }
  
  agent.p_new=1;
  Mfree(Mat_A);
  Mfree(Mat_b);
  Mfree(Mat_AT);
  Mfree(Mat_ATA_inv);
  Mfree(Mat_temp1);
  Mfree(Mat_temp2);
  Mfree(Mat_temp3);

  printf("\r\n %f  %f",agent.p_[0],agent.p_[1]);
  
  for(int i=0;i<4;i++)
  {
    if(tdoa_array[i].s_id==2)
    {
      tdoa_sum_cnt++;
      tdoa_cal=tdoa_array[i].tdoa;
      printf("\r\ntdoa(%d)=%lld",tdoa_sum_cnt,tdoa_cal);
    }
  }
  
  






  //for(int i=0;i<4;i++)
  //{
  //  printf("\r\nr%d-r1:%fm",i+2,tdoa_array[i].tdoa*SPEED_OF_LIGHT*DWT_TIME_UNITS);
  //}
  
  //int temp=1;  
}

void tdoa_clear()
{
  for(int i=0;i<4;i++)
  {
    m_rx_array[i].m_id=0;
    m_rx_array[i].rx_time=0;
    s_rx_array[i].m_id=0;
    s_rx_array[i].s_id=0;
    s_rx_array[i].rx_time=0;
    tdoa_array[i].m_id=0;
    tdoa_array[i].s_id=0;
    tdoa_array[i].tdoa=0;
    m_rx_cnt=0;
    s_rx_cnt=0;
    tdoa_cnt=0;
  }
}


void Matrix_transpose(float *MatInput,float *MatOutput,int m,int n)
{
  for(int i=0;i<n;i++)
  {
    for(int j=0;j<m;j++)
    {
      *(MatOutput+m*i+j)=*(MatInput+n*j+i);
    }
  } 
  
}

void Matrix_matmul(float *MatInput1,float *MatInput2,float *MatOutput, int m,int n,int p)
{
  for(int i=0;i<m;i++)
  {
    for(int j=0;j<p;j++)
    {

      float temp=0;
      for(int k=0;k<n;k++)
      {
          temp+=(*(MatInput1+i*n+k))*(*(MatInput2+k*p+j));
      }
      *(MatOutput+i*p+j)=temp;
    }
  }
 
}

void matrix_printf(float **a)
{
  for(int i=0;i<2;i++)
  {
    for(int j=0;j<2;j++)
    {
      printf("\r\na[%d][%d]",i,j,**a);
    }
      
  }
  int temp=1;
}



void Matrix_inverse(float *MatInput,float *MatOutput, int m,int n)
{
  
}

float dist(float *p1, float *p2)
{
  float d=sqrt(pow((double)*p1-*p2,2)+pow((double)*(p1+1)-*(p2+1),2));
  return d;
}