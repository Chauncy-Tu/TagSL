/*********************************************************************/
// Chenxin Tu  
// 2022.11.24

/*
File    : main.c
Purpose : Nordic nRF52840-DK build main entry point for the project.
*/

#include <stdio.h>
#include <stdlib.h>
#include <sdk_config.h>  // nRF52840 SDK

#include <boards.h>      
#include <port.h>        
#include <deca_spi.h>
#include <deca_device_api.h>
#include <deca_vals.h>
#include <deca_regs.h>

#include <app_usbd.h>

#include <Agent.h>
#include <user_usbd.h>


#define UNIT_TEST 0



/* function declaration */
void SysTick_Handler(void);
void test_run_info(unsigned char *data);
void sysTick_init();
void nRF52840_init();
void dw3000_init();


/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,  /* STS length, see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};

extern dwt_txconfig_t txconfig_options;

extern void rx_ok_cb(const dwt_cb_data_t *cb_data);
extern void rx_to_cb(const dwt_cb_data_t *cb_data);
extern void rx_err_cb(const dwt_cb_data_t *cb_data);
extern void tx_ok_cb(const dwt_cb_data_t *cb_data);


uint8_t slot_event_ = 0;

struct  Agent agent;


//char    AgentRole=Master_Anchor;
//uint8_t AgentID=1;


//char    AgentRole=Slave_Anchor;
//uint8_t AgentID=5;



char    AgentRole=Tag;
uint8_t AgentID=6;

float   AgentPos[2]={1,-1};

uint8_t AgentSlotNum=10;
uint8_t AgentSlave[4]={2,3,4,5};




int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/
 
    /* Reset of all peripherals (if attached). *

    /* USER CODE BEGIN Init */
    //user_init_usbd();
    /* USER CODE END Init */

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    // nRF52840 initialization
    nRF52840_init();

    user_init_usbd();
   
    // system tick initializaiton
    sysTick_init();
    
    //  DW3000 initialization 
    dw3000_init();

    //test
    if(UNIT_TEST)
    {
        //unit_test_main();
    }
    else
    {
        uint32_t dev_id = dwt_readdevid();
        printf("\r\n%X",dev_id);
        
    }
    
    agent=agent_init(AgentRole,AgentID);
    // assign position
    agent.pTrue_[0]=AgentPos[0];
    agent.pTrue_[1]=AgentPos[1];
    agent.slot_num_=AgentSlotNum;
    for(int i=0;i<4;i++)
    {
      agent.slave_[i]=AgentSlave[i];
    }
    printf("\r\nAgentRole:%d",agent.role_);
    printf("\r\nAgentId:%d",agent.id_);
    


    char a[3]="abc";




    while(1)
    {
       if(slot_event_)
       {
         agent_run_slot();
         slot_event_=0;			
       }
       //while(app_usbd_event_queue_process())
       //{
       //}

       //usb_send(a,3);

       //__WFE();

    }


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    //while (1)
    //{

    ///* USER CODE END WHILE */

    ///* USER CODE BEGIN 3 */
    //    printf("%d\r\n",1);

    //}
    /* USER CODE END 3 */

}

/******************************************************************/
/* Other functions in this file */

/*@brief  This function is simply a printf() call for a string. 
It is implemented differently on other platforms,but on the nRF52840-DK, a printf() call is .
*/
void test_run_info(unsigned char *data)
{
    printf("%s\n", data);
}

/*
* Function Name: SysTick Handler
* Description  : Decreament the g_ticks value */
void SysTick_Handler(void)     // invoke this function at the beginning of every slot
{
    slot_event_ = 1;
}


void nRF52840_init()
{
    /* Initialize all configured peripherals */
    bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS);

    /* Initialise the SPI for nRF52840-DK */
    nrf52840_dk_spi_init();

    /* Configuring interrupt*/
    dw_irq_init();

    /* Small pause before startup */
    nrf_delay_ms(2);

}

void sysTick_init()
{
    SystemCoreClockUpdate();
    int slotTickCnt = SystemCoreClock / 1000 * SLOT_LENGTH_IN_MS;   // SystemCoreClock:64000000 64MHz
    SysTick_Config(slotTickCnt);
}

void dw3000_init()
{
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        //test_run_info((unsigned char *)"INIT FAILED     ");
        while (1)
        { };
    }

    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. */

    dwt_setleds(DWT_LEDS_ENABLE| DWT_LEDS_INIT_BLINK) ;
    

    /* Configure DW IC. See NOTE 5 below. */
    if(dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    {
        //test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1)
        { };
    }

    /* Configure the TX spectrum parameters (power PG delay and PG Count) */
    dwt_configuretxrf(&txconfig_options);

    /*dw3000 callback function setting*/   
    dwt_setcallbacks(&tx_ok_cb, &rx_ok_cb, &rx_err_cb, &rx_err_cb, NULL, NULL);

    dwt_setinterrupt(SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK |
            SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK |
            SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK, 0, DWT_ENABLE_INT);

    //dwt_setinterrupt(SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK, 0, DWT_ENABLE_INT);
    /* Install DW IC IRQ handler. */
    port_set_dwic_isr(dwt_isr);

    dwt_setrxtimeout(0);
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(0);

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    

    if(agent.role_==Tag)
    {
      dwt_setdblrxbuffmode(DBL_BUF_STATE_EN,DBL_BUF_MODE_MAN);  //Enable double buff - Manual mode
      dwt_configciadiag(DW_CIA_DIAG_LOG_MIN);//Enable diagnostic mode - minimal
    }

    nrf_delay_ms(200);

    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    

    
}


/*************************** End of file ****************************/

