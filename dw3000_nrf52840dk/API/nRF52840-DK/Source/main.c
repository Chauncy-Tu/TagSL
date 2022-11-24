/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 2014 - 2020 SEGGER Microcontroller GmbH             *
*                                                                    *
*           www.segger.com     Support: support@segger.com           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* conditions are met:                                                *
*                                                                    *
* - Redistributions of source code must retain the above copyright   *
*   notice, this list of conditions and the following disclaimer.    *
*                                                                    *
* - Neither the name of SEGGER Microcontroller GmbH                  *
*   nor the names of its contributors may be used to endorse or      *
*   promote products derived from this software without specific     *
*   prior written permission.                                        *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED.                                                        *
* IN NO EVENT SHALL SEGGER Microcontroller GmbH BE LIABLE FOR        *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Nordic nRF52840-DK build main entry point for simple exmaples.

*/

#include <stdio.h>
#include <stdlib.h>
#include <sdk_config.h>  // nRF52840 SDK
#include <boards.h>      // 
#include <port.h>        
#include <deca_spi.h>

#define UNIT_TEST 0

/*! ------------------------------------------------------------------------------------------------------------------
* @fn test_run_info()
*
* @brief  This function is simply a printf() call for a string. It is implemented differently on other platforms,
*         but on the nRF52840-DK, a printf() call is .
*
* @param data - Message data, this data should be NULL string.
*
* output parameters
*
* no return value
*/
void test_run_info(unsigned char *data)
{
    printf("%s\n", data);
}

/**********************************************1***********************
*
*       main()
*
*  Function description
*   Application entry point.
*/
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/
 
    /* Reset of all peripherals (if attached). */

    //build_examples();

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
    if(UNIT_TEST)
    {
        //unit_test_main();
    }
    else
    {
        uint32_t dev_id = dwt_readdevid();
        printf("\r\n%X",dev_id);
        //Run the selected example as selected in example_selection.h
        //
        //example_pointer();  //function pointer
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

/*************************** End of file ****************************/