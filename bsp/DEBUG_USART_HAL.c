#include "debug_usart_hal.h"
#include "stdio.h"
#include "u_flash.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"
//重定向打印串口

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}


#define  DEBUG_USART        huart1



ETH_TYPE  ether_st;





void debug_usart_send_byte(uint8_t p)
{

    HAL_UART_Transmit(&huart1, (uint8_t*)&p,1, 0xffff);

}

void eth_send_string(uint8_t *p) {

    while(*p!= '\0') {

        debug_usart_send_byte(*p);
        p++;
    }
//	length=sizeof(data)/sizeof(data[0])
}

void send_string_to_eth(uint8_t *p,uint16_t plen)
{
	 HAL_UART_Transmit(&huart1, p,plen, 0xffffffff);
	
}

/*****************************************************************
                       *接收HAL*
******************************************************************/
//开启DMA接收空闲中断
void  debug_usart_dma_open()
{
	   __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
     HAL_UART_Receive_DMA(&huart1,(uint8_t *)ether_st.RX_pData,200);  //不能启动打开
     
}
//开启接收空闲中断

void  ETH_UsartReceive_IDLE()
{
    uint32_t temp=0;
    if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET))
    {
			  
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);//
          HAL_UART_DMAStop(&huart1);
	
        temp = huart1.hdmarx->Instance->CNDTR;//	
          ether_st.RX_Size = data_len - temp;
          ether_st.RX_flag=1;
       HAL_UART_Receive_DMA(&huart1,(uint8_t *)ether_st.RX_pData, data_len);
    }
}

//
void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART3_IRQn 0 */
      ETH_UsartReceive_IDLE();

    /* USER CODE END USART3_IRQn 0 */
      HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART3_IRQn 1 */
	
    /* USER CODE END USART3_IRQn 1 */
}
extern uint8_t sn_code[12];
 uint8_t sn_codeq[12];
void process_usart_data()
{
	
	if(ether_st.RX_flag==1)
	{
	//	printf("123");
		if(strncmp((char *)ether_st.RX_pData, "sn:",3)==0)
		{
			
			for(uint16_t i=0;i<12;i++)
			{
		//	  printf("sn=%c",ether_st.RX_pData[i+3]) ; 
				sn_code[i]=ether_st.RX_pData[i+3];
			}				
			
    	Flash_Write_Num_Word(ADDR_FLASH_PAGE_512,(uint32_t *) sn_code, 3 );
	        Flash_Read_Word( ADDR_FLASH_PAGE_512, (uint32_t *)sn_code,3 ) ;
			
//			for(uint16_t i=0;i<12;i++)
//			{
//			  printf("sn=%c",sn_code[i]) ; 
//			
//			}	
			send_string_to_eth(sn_code,12);
		}
		
		ether_st.RX_flag=0;
	  ether_st.RX_Size=0;
	}
	
	
}


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "SX127X_Driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */
//int haha(void)
//{
//  /* USER CODE BEGIN 1 */

//  /* USER CODE END 1 */

//  /* MCU Configuration--------------------------------------------------------*/

//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();

//  /* USER CODE BEGIN Init */

//  /* USER CODE END Init */

//  /* Configure the system clock */
//  SystemClock_Config();

//  /* USER CODE BEGIN SysInit */

//  /* USER CODE END SysInit */

//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_ADC_Init();
//  MX_USART1_UART_Init();
//  MX_SPI1_Init();
//  MX_RTC_Init();
////  /* Initialize interrupts */
// 
//  /* USER CODE BEGIN 2 */
//	
//  while (1)
//  {
//		
//		 lora_process();
//		 check_rung_state();
//		 check_vol_task();  

//  }
//  /* USER CODE END 3 */
//}








