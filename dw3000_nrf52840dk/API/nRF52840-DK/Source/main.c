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
#include <Agent.h>

#define UNIT_TEST 0



uint8_t slot_event_ = 0;

char    AgentRole=Master_Anchor;
uint8_t AgentID=1;
float   AgentPos[2]={1,1};




extern void agent_run_slot();
extern struct Agent agent_init(char role,uint8_t id);

void test_run_info(unsigned char *data);
void SysTick_Handler(void);


int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/
 
    /* Reset of all peripherals (if attached). */



    /* USER CODE BEGIN Init */
    //user_init_usbd();
    /* USER CODE END Init */

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS);

    /* Initialise the SPI for nRF52840-DK */
    nrf52840_dk_spi_init();

    /* Configuring interrupt*/
    dw_irq_init();

    /* Small pause before startup */
    nrf_delay_ms(2);


    SystemCoreClockUpdate();
    int slotTickCnt = SystemCoreClock / 1000 * SLOT_LENGTH_IN_MS;   // SystemCoreClock:64000000 64MHz
    SysTick_Config(slotTickCnt);


    if(UNIT_TEST)
    {
        //unit_test_main();
    }
    else
    {
        uint32_t dev_id = dwt_readdevid();
        printf("\r\n%X",dev_id);
        
    }


    
    struct Agent agent=agent_init(AgentRole,AgentID);
    // assign position
    agent.pTrue_[0]=AgentPos[0];
    agent.pTrue_[1]=AgentPos[1];
    printf("AgentRole:%d",agent.role_);


    while(1)
    {
       if(slot_event_)
       {
         agent_run_slot();
         //printf("\r\nslot_cnt_:%d",slot_cnt);
         //slot_cnt++;
         slot_event_=0;			
       }
       
		
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










/*************************** End of file ****************************/

