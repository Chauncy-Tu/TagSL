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
//#include <cmatrix.h>

uint8_t response_cnt_=0;
uint8_t frame_cnt_ = 0;
uint8_t slot_cnt_ = 0;
uint32_t status_reg;

struct M_RX{
  uint8_t m_id;
  int64_t rx_time;
};
struct S_RX{
  uint8_t m_id;
  uint8_t s_id;
  int64_t rx_time;
};

struct TDOA{
  uint8_t m_id;
  uint8_t s_id;
  int64_t tdoa;
};

int m_rx_cnt=0;
int s_rx_cnt=0;
int tdoa_cnt=0;

struct M_RX m_rx_array[4];
struct S_RX s_rx_array[4];
struct TDOA tdoa_array[4];


int32_t t_reply_array[4];




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





static int send_uwb_sync()
{
    agent.tx_msg_.frame_type_ = 0x00;       // sync msg

    agent.tx_msg_.frame_cnt_ = frame_cnt_;
    agent.tx_msg_.slot_cnt_ = slot_cnt_;
    agent.tx_msg_.t_reply_ = 0;
    agent.tx_msg_.tx_id_ = agent.id_;
    
    Farray_copy(agent.tx_msg_.pos_,agent.pTrue_,sizeof(agent.pTrue_)/sizeof(agent.pTrue_[0]));
    
    dwt_forcetrxoff();  
    dwt_writetxdata(sizeof(agent.tx_msg_) - 2, (uint8_t *)&agent.tx_msg_, 0);
    dwt_writetxfctrl(sizeof(agent.tx_msg_), 0, 1);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
    
    return 0;
   
}

static int send_uwb_blink()
{
    agent.tx_msg_.frame_type_ = 0x01;       // blink msg
    agent.tx_msg_.frame_cnt_ = frame_cnt_;
    agent.tx_msg_.slot_cnt_ = slot_cnt_;
    agent.tx_msg_.t_reply_ = 0;
    agent.tx_msg_.tx_id_ = agent.id_;

    Farray_copy(agent.tx_msg_.pos_,agent.pTrue_,sizeof(agent.pTrue_)/sizeof(agent.pTrue_[0]));
    UCarray_copy(agent.tx_msg_.slave_,agent.slave_,sizeof(agent.slave_)/sizeof(agent.slave_[0]));
    
    
    dwt_forcetrxoff();  
    dwt_writetxdata(sizeof(agent.tx_msg_) - 2, (uint8_t *)&agent.tx_msg_, 0);
    dwt_writetxfctrl(sizeof(agent.tx_msg_), 0, 1);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
    
    return 0;
}

static int send_uwb_response(uint8_t mt_id, int seq)
{
    agent.tx_msg_.frame_type_ = 0x02;       // blink msg
    agent.tx_msg_.frame_cnt_ = frame_cnt_;
    agent.tx_msg_.slot_cnt_ = slot_cnt_;
    
    agent.tx_msg_.tx_id_ = agent.id_;
    agent.tx_msg_.mt_id_ = mt_id;

    Farray_copy(agent.tx_msg_.pos_,agent.pTrue_,sizeof(agent.pTrue_)/sizeof(agent.pTrue_[0]));
    
    
    
    uint64_t systime = 0;
    dwt_forcetrxoff();
    
    //systime = (uint64_t)dwt_readsystimestamphi32() << 8;
    uint32_t delaytxtime = ((agent.rx_time_ + ((RESPONSE_DELAY+seq*SEQUENCE_DELAY) * UUS_TO_DWT_TIME)) >> 8) & 0xFFFFFF00;
    dwt_setdelayedtrxtime(delaytxtime);
    agent.tx_time_ = (uint64_t)delaytxtime << 8;
    agent.tx_msg_.t_reply_ = (agent.rx_time_)? agent.tx_time_ - agent.rx_time_ : 0;


   

    //printf("\r\nt_reply:%d",agent.tx_msg_.t_reply_);
    
    dwt_writetxdata(sizeof(agent.tx_msg_) - 2, (uint8_t *)&agent.tx_msg_, 0);
    dwt_writetxfctrl(sizeof(agent.tx_msg_), 0, 1);

    if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
    //printf("\r\nt_reply:%d",agent.tx_msg_.t_reply_);


    
    
    

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
      printf("\r\nframe_cnt:%d",frame_cnt_);
      Fang_tdoa();
      //printf("\r\nresponse_cnt:%d",response_cnt_);
      
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
      

    }


  
}

void rx_ok_cb(const dwt_cb_data_t *cb_data)
{

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
  uint8_t slave[4];
  
  UCarray_copy(slave, agent.rx_msg_.slave_,sizeof(agent.rx_msg_.slave_)/sizeof(agent.rx_msg_.slave_[0]));


  if(agent.role_==Slave_Anchor)
  {
    if(frame_type==0x00)
    {

      NRFX_IRQ_PENDING_CLEAR(SysTick_IRQn);
      SystemCoreClockUpdate();
      SysTick_Config(SystemCoreClock / 1000 * SLOT_LENGTH_IN_MS);
      //printf("\r\nsync rx ok");

      
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

  }

  if(agent.role_==Tag)
  {
    if(frame_type==0x00)  // sync frame
    {
      NRFX_IRQ_PENDING_CLEAR(SysTick_IRQn);
      SystemCoreClockUpdate();
      SysTick_Config(SystemCoreClock / 1000 * SLOT_LENGTH_IN_MS);    
      Farray_copy(agent.map_[tx_id],agent.rx_msg_.pos_,sizeof(agent.rx_msg_.pos_)/sizeof(agent.rx_msg_.pos_[0]));
      //printf("\r\nsync rx");
      
    }else if(frame_type==0x01) // blink frame
    {
      m_rx_array[m_rx_cnt].m_id=tx_id;
      m_rx_array[m_rx_cnt].rx_time=agent.rx_time_;
      m_rx_cnt++;
      Farray_copy(agent.map_[tx_id],agent.rx_msg_.pos_,sizeof(agent.rx_msg_.pos_)/sizeof(agent.rx_msg_.pos_[0]));
      //printf("\r\nblink rx");
      
    }else if(frame_type==0x02)  //response frame
    {
      response_cnt_++;


      float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);
      //printf("\r\nclockOffsetRatio:%f",clockOffsetRatio);
      int32_t t_reply=agent.rx_msg_.t_reply_;


      //t_reply_array[s_rx_cnt]=t_reply;
      for(int i=0;i<m_rx_cnt;i++)
      {
        if(m_rx_array[i].m_id==mt_id)
        {
          s_rx_array[s_rx_cnt].m_id=mt_id;
          s_rx_array[s_rx_cnt].s_id=tx_id;
          s_rx_array[s_rx_cnt].rx_time=agent.rx_time_;
          s_rx_cnt++;

          int64_t m_rxtime=m_rx_array[i].rx_time;
          int64_t s_rxtime=agent.rx_time_;

          tdoa_array[tdoa_cnt].m_id=mt_id;
          tdoa_array[tdoa_cnt].s_id=tx_id;
          tdoa_array[tdoa_cnt].tdoa=((s_rxtime-m_rxtime-16385)-(1.0-clockOffsetRatio)*t_reply); 
          //TODO antenna delay,  cfo + or -

          tdoa_cnt++;
          if(tdoa_cnt==4)
          {
            Fang_tdoa();

            tdoa_clear();

            break;
          }
          
        }
      }
      
      

      Farray_copy(agent.map_[tx_id],agent.rx_msg_.pos_,sizeof(agent.rx_msg_.pos_)/sizeof(agent.rx_msg_.pos_[0]));
      
      
    }
    
  }
  //dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);


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

  
  





  
  double A[4][3]={-100,100,99.2797,100,100,-48.3271,100,-100,31.7534,-100,-100,135.3482};
  double b[4]={2535.8866,4.416121928898527e+03,4.747930262545749e+03,4.202134211714574e+02};

  for(int i=0;i<4;i++)
  {
    
    printf("\r\n %lf %lf %lf",A[i][0],A[i][1],A[i][2]);
   
  }



  //for(int i=0;i<4;i++)
  //{
  //  float origin[2]={0,0};
  //  ri1[i]=SPEED_OF_LIGHT*tdoa_array[i].tdoa*DWT_TIME_UNITS-dist(agent.map_[tdoa_array[i].m_id],agent.map_[tdoa_array[i].s_id]);
  //  Ki1[i]=dist(agent.map_[tdoa_array[i].s_id],origin)*dist(agent.map_[tdoa_array[i].s_id],origin)-dist(agent.map_[tdoa_array[i].m_id],origin)*dist(agent.map_[tdoa_array[i].m_id],origin);
  //  A[i][0]=2*(agent.map_[tdoa_array[i].s_id][0]-agent.map_[tdoa_array[i].m_id][0]);
  //  A[i][1]=2*(agent.map_[tdoa_array[i].s_id][1]-agent.map_[tdoa_array[i].m_id][1]);
  //  A[i][2]=2*ri1[i];

  //  b[i]=Ki1[i]-ri1[i]*ri1[i];
  //}

  matrix *Mat_A, *Mat_b;
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
  Minit(Mat_A);
  Minit(Mat_b);

  matrix *Mat_temp1,*Mat_temp2,*Mat_temp3;
  Mat_temp1 = Mnew(3, 3);
  Mat_temp2 = Mnew(3, 4);
  Mat_temp3 = Mnew(3, 1);

  int a[10000];

  Mat_temp1->A=AB(Mat_A->T->A,Mat_A->T->m,Mat_A->T->n,Mat_A->A,Mat_A->m,Mat_A->n);
  Minit(Mat_temp1);
  Mat_temp2->A=AB(Mat_temp1->inv->A,Mat_temp1->inv->m,Mat_temp1->inv->n,Mat_A->T->A,Mat_A->T->m,Mat_A->T->n);
  Minit(Mat_temp2);
  Mat_temp3->A=AB(Mat_temp2->A,Mat_temp2->m,Mat_temp2->n,Mat_b->A,Mat_b->m,Mat_b->n);
  Minit(Mat_temp3);
  

  for(int i=0;i<2;i++)
  {
    agent.p_[0]=(float)(Mat_temp3->A[0][0]);
    agent.p_[1]=(float)(Mat_temp3->A[1][0]);
  }

  Mfree(Mat_A);
  Mfree(Mat_b);
  Mfree(Mat_temp1);
  Mfree(Mat_temp2);
  Mfree(Mat_temp3);

  printf("\r\n %f  %f",agent.p_[0],agent.p_[1]);

  int temp1=1;
  






  //for(int i=0;i<4;i++)
  //{
  //  printf("\r\nr%d-r1:%fm",i+2,tdoa_array[i].tdoa*SPEED_OF_LIGHT*DWT_TIME_UNITS);
  //}




  


  //matrix* d;
  //d = Mnew(3, 3);
  //d->A[0][0] = 1;
  //d->A[0][1] = 2;
  //d->A[1][0] = 3;
  //d->A[1][1] = 4;
  //d->A[0][2] = 6.5;
  //d->A[1][2] = 0;
  //d->A[2][0] = 0;
  //d->A[2][1] = 8;
  //d->A[2][2] = 3.6;
  
  //Minit(d);
  //printf("\r\nmatrix:\r\n");
  //Mprintf(d);
  //printf("\r\ninverse matrix:\r\n");
  //Mprintf(d->inv);
  //printf("\r\ntranspose matrix:\r\n");
  //Mprintf(d->T);

  





  

  
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

float dist(float *p1, float *p2)
{
  return 0;
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

/*ÐÐÁÐÊ½*/
double hhlx(double** arr, int na)
{
	if (na == 1)
	{
		return arr[0][0];
	}
	else
	{
		double s = 0;
		for (int i = 0; i < na; i++)
		{
			if (arr[0][i] != 0)
			{
				double** arr1;
				arr1 = (double**)malloc((na - 1) * sizeof(double));
				for (int i = 0; i < na - 1; i++)
				{
					arr1[i] = (double*)malloc((na - 1) * sizeof(double));
				}
				for (int j = 1; j < na; j++)
				{
					for (int k = 0; k < na - 1; k++)
					{
						if (k >= i)
						{
							arr1[j - 1][k] = arr[j][k + 1];
						}
						else
						{
							arr1[j - 1][k] = arr[j][k];
						}
					}
				}
				s = s + hhlx(arr1, na - 1) * pow(-1, i) * arr[0][i];
				for (int i = 0; i < na - 1; i++)
				{
					free(arr1[i]);
				}
				free(arr1);
			}
		}
		return s;
	}
}

/*Äæ¾ØÕó*/
double** inv(double** a, int n)
{
	double det = hhlx(a, n);
	double** as;
	as = (double**)malloc(n * sizeof(double));
	for (int i = 0; i < n; i++)
	{
		as[i] = (double*)malloc(n * sizeof(double));
	}
	for (int is = 0; is < n; is++)
	{
		for (int js = 0; js < n; js++)
		{
			double** ab;
			ab = (double**)malloc((n - 1) * sizeof(double));
			for (int i = 0; i < n - 1; i++)
			{
				ab[i] = (double*)malloc((n - 1) * sizeof(double));
			}
			for (int i = 0; i < n - 1; i++)
			{
				for (int j = 0; j < n - 1; j++)
				{
					if (i >= is)
					{
						if (j >= js)
							ab[i][j] = a[i + 1][j + 1];
						else
							ab[i][j] = a[i + 1][j];
					}
					else
					{
						if (j >= js)
							ab[i][j] = a[i][j + 1];
						else
							ab[i][j] = a[i][j];
					}
				}
			}
			as[js][is] = pow(-1, (double)is + (double)js) * hhlx(ab, n - 1);

			for (int i = 0; i < n - 1; i++)
			{
				free(ab[i]);
			}
			free(ab);
		}
	}
	double** ai;
	ai = (double**)malloc(n * sizeof(double));
	for (int i = 0; i < n; i++)
	{
		ai[i] = (double*)malloc(n * sizeof(double));
	}
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			ai[i][j] = as[i][j] / det;
		}
	}
	return ai;
}

/*¾ØÕóÏà³Ë*/
double** AB(double** a, int ma, int na, double** b, int mb, int nb)
{
	if (na != mb)
	{
		printf("¼ÆËã´íÎó£¡");
		return NULL;
	}
	else
	{
		double** ab;
		ab = (double**)malloc(ma * sizeof(double));
		for (int i = 0; i < ma; i++)
		{
			ab[i] = (double*)malloc(nb * sizeof(double));
		}
		for (int i = 0; i < ma; i++)
		{
			for (int j = 0; j < nb; j++)
			{
				ab[i][j] = 0;
				for (int k = 0; k < na; k++)
				{
					ab[i][j] = ab[i][j] + a[i][k] * b[k][j];
				}
			}
		}
		return ab;
	}
}

/*¾ØÕó×ªÖÃ*/
double** TA(double** a, int ma, int na)
{
	double** ta;
	ta = (double**)malloc(na * sizeof(double));
	for (int i = 0; i < na; i++)
	{
		ta[i] = (double*)malloc(ma * sizeof(double));
	}
	for (int i = 0; i < na; i++)
	{
		for (int j = 0; j < ma; j++)
		{
			ta[i][j] = a[j][i];
		}
	}
	return ta;
}

matrix* Mnew(int m, int n)
{
	matrix* a;
	a = (matrix*)malloc(sizeof(matrix));
	a->A = (double**)malloc(m * sizeof(double));
	for (int i = 0; i < m; i++)
	{
		a->A[i] = (double*)malloc(n * sizeof(double));
	}
	a->m = m;
	a->n = n;
	return a;
}

void Minit(matrix* a)
{
	a->inv = Minv(a);
	a->inv->inv = a;
	a->T = Mtrans(a);
	a->T->T = a;
	a->inv->T = Mtrans(a->inv);
	a->T->inv = a->inv->T;
	a->inv->T->T = a->inv;
	a->T->inv->inv = a->T;
	a->inv->T->inv = a->T;
	a->T->inv->T = a->inv;
	a->det = hhlx(a->A, a->m);
	a->inv->det = 1 / a->det;
	a->T->det = a->det;
	a->inv->T->det = a->inv->det;
}

void Mprintf(matrix* a)
{
	for (int i = 0; i < a->m; i++)
	{
		for (int j = 0; j < a->n; j++)
		{
			printf("%f ", a->A[i][j]);
		}
		printf("\n");
	}
}

matrix* Minv(matrix* a)
{
	matrix* ai;
	ai = (matrix*)malloc(sizeof(matrix));
	ai->A = inv(a->A, a->m);
	ai->m = a->m;
	ai->n = a->m;
	return ai;
}

matrix* Mmulti(matrix* a, matrix* b)
{
	matrix* ab;
	ab = (matrix*)malloc(sizeof(matrix));
	ab->A = AB(a->A, a->m, a->n, b->A, b->m, b->n);
	ab->m = a->m;
	ab->n = b->n;
	return ab;
}

matrix* Mtrans(matrix* a)
{
	matrix* at;
	at = (matrix*)malloc(sizeof(matrix));
	at->A = TA(a->A, a->m, a->n);
	at->m = a->n;
	at->n = a->m;
	return at;
}

matrix* Mplus(matrix* a, matrix* b)
{
	matrix* c;
	c = (matrix*)malloc(sizeof(matrix));
	c->A = (double**)malloc(a->m * sizeof(double));
	for (int i = 0; i < a->m; i++)
	{
		c->A[i] = (double*)malloc(a->n * sizeof(double));
	}
	c->m = a->m;
	c->n = a->n;
	for (int i = 0; i < a->m; i++)
	{
		for (int j = 0; j < a->n; j++)
		{
			c->A[i][j] = a->A[i][j] + b->A[i][j];
		}
	}
	return c;
}

matrix* Mminus(matrix* a, matrix* b)
{
	matrix* c;
	c = (matrix*)malloc(sizeof(matrix));
	c->A = (double**)malloc(a->m * sizeof(double));
	for (int i = 0; i < a->m; i++)
	{
		c->A[i] = (double*)malloc(a->n * sizeof(double));
	}
	c->m = a->m;
	c->n = a->n;
	for (int i = 0; i < a->m; i++)
	{
		for (int j = 0; j < a->n; j++)
		{
			c->A[i][j] = a->A[i][j] - b->A[i][j];
		}
	}
	return c;
}

matrix* Mdotpro(matrix* a, matrix* b)
{
	matrix* c;
	c = (matrix*)malloc(sizeof(matrix));
	c->A = (double**)malloc(a->m * sizeof(double));
	for (int i = 0; i < a->m; i++)
	{
		c->A[i] = (double*)malloc(a->n * sizeof(double));
	}
	c->m = a->m;
	c->n = a->n;
	for (int i = 0; i < a->m; i++)
	{
		for (int j = 0; j < a->n; j++)
		{
			c->A[i][j] = (a->A[i][j]) * (b->A[i][j]);
		}
	}
	return c;
}

matrix* Mdiv(matrix* a, matrix* b)
{
	matrix* c;
	c = (matrix*)malloc(sizeof(matrix));
	c->A = (double**)malloc(a->m * sizeof(double));
	for (int i = 0; i < a->m; i++)
	{
		c->A[i] = (double*)malloc(a->n * sizeof(double));
	}
	c->m = a->m;
	c->n = a->n;
	for (int i = 0; i < a->m; i++)
	{
		for (int j = 0; j < a->n; j++)
		{
			c->A[i][j] = a->A[i][j] / b->A[i][j];
		}
	}
	return c;
}

void mfree(matrix* a)
{
	for (int i = 0; i < a->m; i++)
	{
		free(a->A[i]);
	}
	free(a->A);
	free(a);
}

void Mfree(matrix* a)
{
	mfree(a->inv->T);
	mfree(a->inv);
	mfree(a->T);
	mfree(a);
}