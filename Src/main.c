/* USER CODE BEGIN Header */
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
uint32_t  tim_count=0; 
extern uint8_t sleep_flag;
extern uint8_t reset_rang_key;
void SystemClock_Config(void);
extern uint8_t password_key;
uint8_t password_key_value=0;
extern  uint8_t runing_state_flag;
uint8_t init_sleep_flag=0;
void sleep_init();


  uint8_t dev_init_state=1;
  uint8_t reset_sleep_flag=0;
	uint8_t net_suceess_flag=0;

	


/* USER CODE END 0 */
void Test_Dev()
{
  Line_1A_WT588S(1);
	HAL_Delay(500);
 HAL_GPIO_WritePin(led_en_GPIO_Port,led_en_Pin,GPIO_PIN_RESET);	
		HAL_Delay(1000);
	HAL_GPIO_WritePin(led_en_GPIO_Port,led_en_Pin,GPIO_PIN_SET);	
}

void bsp_init_main_key() 
{
	   GPIO_InitTypeDef GPIO_InitStruct = {0};
		
    __HAL_RCC_GPIOA_CLK_ENABLE(); 	
		
	  GPIO_InitStruct.Pin = rang_key_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);		
		
		
			 GPIO_InitStruct.Pin = FANGCHAI_Pin ;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;  //一定要浮空
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);		
 // 	__HAL_RCC_GPIOA_CLK_DISABLE();
	     MX_RTC_Init();
			  MX_NVIC_Init();

}	

void Wake_Up_Io_Config()
{
	 GPIO_InitTypeDef GPIO_InitStruct = {0};
		
    __HAL_RCC_GPIOA_CLK_ENABLE(); 			
	  GPIO_InitStruct.Pin  = rang_key_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);		
	 
	  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
     HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	 
	
}

void stop_mode_config(void)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
{
 GPIO_InitTypeDef GPIO_InitStruct;
	
	
 __HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG();    //clear flag
 __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);                //clear Power wakeup flag
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableUltraLowPower();
  HAL_PWREx_EnableFastWakeUp();
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);	
	__HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = led_en_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(led_en_GPIO_Port, &GPIO_InitStruct);

////////  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

//////  /*Configure GPIO pins : PBPin PBPin PBPin PBPin 
//////                           PBPin PBPin PBPin */
//////  HAL_GPIO_WritePin(GPIOB, BELL_EN_Pin, GPIO_PIN_RESET); //
    HAL_GPIO_WritePin(GPIOB,  BELL_EN_Pin|speak_data_Pin, GPIO_PIN_RESET);  //下拉低功耗
  GPIO_InitStruct.Pin = lora_rst_Pin|LORA_DIO0_Pin|wr_24cxx_Pin|clk_24cxx_Pin 
                          |sda_24cxx_Pin|rang_led_Pin|speak_data_Pin|BELL_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	

    GPIO_InitStruct.Pin = key2_Pin|key3_Pin|key1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
////////  /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = speak_busy_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(speak_busy_GPIO_Port, &GPIO_InitStruct);
	

				
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();

}
extern uint8_t rang_key_flag;
extern uint8_t fangchai_flag;
void valve_enter_ed_stop_mode(void)
{
  //config main key with interrupt

	      sleep_flag=1;
     bsp_init_main_key() ;
	  
  __HAL_RCC_PWR_CLK_ENABLE();  
  __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
//	    HAL_Delay(5);
   //  sleep_init();
//		 printf("wkup...\r\n");
//	 printf("rang_key_flag=%d\r\n",rang_key_flag);
//	 printf("fangchai_flag=%d\r\n",fangchai_flag);
}
void sleep_open()
{

  SX127X_SleepMode(); //睡眠模式
	stop_mode_config();
	valve_enter_ed_stop_mode();
}

void Sleep_Into()
{
	
  //   __HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG();
	 //  __HAL_GPIO_EXTI_CLEAR_IT(0x7FFFF);
	    SX127X_SleepMode(); //睡眠模式
	     stop_mode_config();
//	   bsp_init_main_key() ;
	   Wake_Up_Io_Config();
  __HAL_RCC_PWR_CLK_ENABLE();  
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	
}

//void sleep_init()
//{
//    /* USER CODE BEGIN SysInit */
//	   HAL_UART_DeInit(&huart1);
//	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_All);

//	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_All);
//  HAL_GPIO_DeInit(GPIOH, GPIO_PIN_All);
// 
////       HAL_UART_DeInit(&hlpuart1);
//	     HAL_Init();
//  /* USER CODE BEGIN Init */

//  /* USER CODE END Init */

//  /* Configure the system clock */
//  SystemClock_Config();

//  /* USER CODE BEGIN SysInit */
//      HAL_UART_DeInit(&huart1);
//	     HAL_ADC_DeInit(&hadc);
//	     HAL_SPI_DeInit(&hspi1);
////		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15);
//  /* USER CODE END SysInit */
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();	
//  MX_ADC_Init();
//  MX_USART1_UART_Init();
//  MX_SPI1_Init();
//  MX_RTC_Init();
// 
////  /* Initialize interrupts */
//   MX_NVIC_Init();
//	// 	Init_Dev_Param();

//  /* USER CODE BEGIN 2 */
//	app_lora_config_init();
//}

void sleep_init()
{
    /* USER CODE BEGIN SysInit */
	   HAL_UART_DeInit(&huart1);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_All);

	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_All);
  HAL_GPIO_DeInit(GPIOH, GPIO_PIN_All);
 
//       HAL_UART_DeInit(&hlpuart1);
	     HAL_Init();
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
      HAL_UART_DeInit(&huart1);
	     HAL_ADC_DeInit(&hadc);
	     HAL_SPI_DeInit(&hspi1);
//		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15);
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();	
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_RTC_Init();

//  /* Initialize interrupts */
   MX_NVIC_Init();
	// 	debug_usart_dma_open();
  /* USER CODE BEGIN 2 */
	app_lora_config_init();
}
extern uint8_t reset_rang_flag;
void  Reset_Process()
{
	  

	  
			do{
				   switch(dev_init_state)
		        {
			       case 0 : 
							    
					
				              Sleep_Into();
						          sleep_init();
                     dev_init_state=1;
			        break;
			       case 1 :
			
			  	net_suceess_flag=factory_parameter_set();
					    	 process_usart_data();
				      if(reset_rang_key==1)
				      {
						   HAL_NVIC_SystemReset();
					      reset_rang_key=0;
				      }
				     break;
	
						}
			}while(net_suceess_flag!=1);
			dev_init_state=3;
				
}
			
	
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  	Init_Dev_Param();
	 debug_usart_dma_open();
	
	app_lora_config_init();
   Test_Dev();
	 	Reset_Process();
	 printf("进入sleep");

		  sleep_open();
  /* USER CODE END 2 */
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
	 while(1)
	 {
		 lora_process();
		 check_rung_state();
		 check_vol_task();  
	 }
		
		
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* RTC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RTC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  /* ADC1_COMP_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USER CODE BEGIN 4 */
uint32_t system_tick_count=0;
uint32_t sleep_tick_count=0;
/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	        if(dev_init_state==1)
        {
              system_tick_count++;
                if(system_tick_count%1000==0)
                    {
                        system_tick_count=0;
                        sleep_tick_count++;
                    }
               if(sleep_tick_count>60)
                 {
                     sleep_tick_count=0;
                     dev_init_state=0;
                 }
        }
        else
        {
            sleep_tick_count=0;
            system_tick_count=0;
        }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
