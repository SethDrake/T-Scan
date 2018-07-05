/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_ThreadCreation/Src/main.c
  * @author  MCD Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright © 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "st_logo1.h"
#include <ili9341.h>


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osThreadId LEDThread1Handle, LEDThread2Handle, LTDCThreadHandle, SDRAMThreadHandle;
LTDC_HandleTypeDef LtdcHandle;

__IO uint32_t ReloadFlag = 0;

/* Pictures position */
uint32_t Xpos1 = 0;
uint32_t Xpos2 = 0;
uint32_t Ypos1 = 0;
uint32_t Ypos2 = 160;

/* Private function prototypes -----------------------------------------------*/
static void LED_Thread1(void const *argument);
static void LED_Thread2(void const *argument);
static void LTDC_Thread(void const *argument);
static void SDRAM_Thread(void const *argument);
static void SystemClock_Config(void);
static void LCD_Config(void);
static void PicturesPosition(uint32_t* x1, 
	uint32_t* y1, 
	uint32_t* x2, 
	uint32_t* y2, 
	uint32_t index);

static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();  
  
  /* Configure LED3 and LED4 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  
  /* Configure the system clock to 168 MHz */
  SystemClock_Config();

  BSP_SDRAM_Init();

  //copy image to sdram
  BSP_SDRAM_WriteData(SDRAM_DEVICE_ADDR, (uint32_t *)&ST_LOGO_1, sizeof(ST_LOGO_1));

  LCD_Config();
  
  osThreadDef(LED3, LED_Thread1, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadDef(LED4, LED_Thread2, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadDef(LDTC, LTDC_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadDef(SDRAM, SDRAM_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  
  LEDThread1Handle = osThreadCreate (osThread(LED3), NULL);
  LEDThread2Handle = osThreadCreate (osThread(LED4), NULL);
  LTDCThreadHandle = osThreadCreate(osThread(LDTC), NULL);
  SDRAMThreadHandle = osThreadCreate(osThread(SDRAM), NULL);
  
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  for(;;);
}

/**
  * @brief  Toggle LED3 and LED4 thread
  * @param  thread not used
  * @retval None
  */
static void LED_Thread1(void const *argument)
{
  uint32_t count = 0;
  (void) argument;
  
  for(;;)
  {
    count = osKernelSysTick() + 5000;
    
    /* Toggle LED3 every 200 ms for 5 s */
    while (count >= osKernelSysTick())
    {
      BSP_LED_Toggle(LED3);
      
      osDelay(200);
    }
    
    /* Turn off LED3 */
    BSP_LED_Off(LED3);
    
    /* Suspend Thread 1 */
    //osThreadSuspend(NULL);
    
    count = osKernelSysTick() + 5000;
    
    /* Toggle LED3 every 400 ms for 5 s */
    while (count >= osKernelSysTick())
    {
      BSP_LED_Toggle(LED3);
      
      osDelay(400);
    }
    
    /* Resume Thread 2 */
    //osThreadResume(LEDThread2Handle);
  }
}

/**
  * @brief  Toggle LED4 thread
  * @param  argument not used
  * @retval None
  */
static void LED_Thread2(void const *argument)
{
  uint32_t count;
  (void) argument;
  
  for(;;)
  {
    count = osKernelSysTick() + 10000;
    
    /* Toggle LED4 every 500 ms for 10 s */
    while (count >= osKernelSysTick())
    {
      //BSP_LED_Toggle(LED4);

      osDelay(500);
    }
    
    /* Turn off LED4 */
    //BSP_LED_Off(LED4);
    
    /* Resume Thread 1 */
    //osThreadResume(LEDThread1Handle);
    
    /* Suspend Thread 2 */
    //osThreadSuspend(NULL);  
  }
}

static void LTDC_Thread(void const *argument)
{
	(void) argument;
  
	for (;;)
	{
		for (uint8_t index = 0; index < 40; index++)
		{
		  /* calculate new picture position */
			PicturesPosition(&Xpos1, &Ypos1, &Xpos2, &Ypos2, (index + 1));
      
			/* reconfigure the layer1 position  without Reloading*/
			HAL_LTDC_SetWindowPosition_NoReload(&LtdcHandle, Xpos1, Ypos1, 0);
			/* reconfigure the layer2 position  without Reloading*/
			HAL_LTDC_SetWindowPosition_NoReload(&LtdcHandle, Xpos2, Ypos2, 1);
			/* Ask for LTDC reload within next vertical blanking*/
			ReloadFlag = 0;
			HAL_LTDC_Reload(&LtdcHandle, LTDC_SRCR_VBR);
      
			while (ReloadFlag == 0)
			{
			  /* wait till reload takes effect (in the next vertical blanking period) */
			}
		}
		osDelay(1000);
		for (uint8_t index = 0; index < 40; index++)
		{
		  /* calculate new picture position */
			PicturesPosition(&Xpos1, &Ypos1, &Xpos2, &Ypos2, (index + 1));
      
			/* reconfigure the layer1 position  without Reloading*/
			HAL_LTDC_SetWindowPosition_NoReload(&LtdcHandle, Xpos2, Ypos2, 0);
			/* reconfigure the layer2 position  without Reloading*/
			HAL_LTDC_SetWindowPosition_NoReload(&LtdcHandle, Xpos1, Ypos1, 1);
			/* Ask for LTDC reload within next vertical blanking*/
			ReloadFlag = 0;
			HAL_LTDC_Reload(&LtdcHandle, LTDC_SRCR_VBR);
      
			while (ReloadFlag == 0)
			{
			  /* wait till reload takes effect (in the next vertical blanking period) */
			}
		}
		osDelay(1000);
	}
}


#define buf_size 8

static void SDRAM_Thread(void const *argument)
{
	(void) argument;  
	for (;;)
	{
		uint32_t testData[buf_size] = { 1, 2, 3, 4, 5, 6, 7, 8 };
		uint32_t testData2[buf_size];
		BSP_SDRAM_WriteData(CUSTOM_DATA_ADDR, testData, buf_size);
		BSP_SDRAM_ReadData(CUSTOM_DATA_ADDR, testData2, buf_size);
		osDelay(1000);
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  
	/* Enable Power Control clock */
	__HAL_RCC_PWR_CLK_ENABLE();
  
	/* The voltage scaling allows optimizing the power consumption when the device is 
		clocked below the maximum system frequency, to update the voltage scaling value 
		regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);
 
	/* Activate the Over-Drive mode */
	HAL_PWREx_EnableOverDrive();
 
	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

	/*##-2- LTDC Clock Configuration ###########################################*/  
	/* LCD clock configuration */
	/* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 MHz */
	/* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 MHz */
	/* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/4 = 48 MHz */
	/* LTDC clock frequency = PLLLCDCLK / RCC_PLLSAIDIVR_8 = 48/4 = 12 MHz */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 4;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct); 	
}

static void PicturesPosition(uint32_t* x1, uint32_t* y1, uint32_t* x2, uint32_t* y2, uint32_t index)
{
  /* picture1 position */
	*x1 = 0;
	*y1 = index * 4; 
  
	/* picture2 position */
	*x2 = 0;
	*y2 = 160 - index * 4;
}

/**
  * @brief  Reload Event callback.
  * @param  hltdc: pointer to a LTDC_HandleTypeDef structure that contains
  *                the configuration information for the LTDC.
  * @retval None
  */
void HAL_LTDC_ReloadEventCallback(LTDC_HandleTypeDef *hltdc)
{
	ReloadFlag = 1;
}

/**
  * @brief LCD Configuration.
  * @note  This function Configure the LTDC peripheral :
  *        1) Configure the Pixel Clock for the LCD
  *        2) Configure the LTDC Timing and Polarity
  *        3) Configure the LTDC Layer 1 :
  *           - direct color (RGB565) as pixel format  
  *           - The frame buffer is located at FLASH memory
  *           - The Layer size configuration : 240x160
  *        4) Configure the LTDC Layer 2 :
  *           - direct color (RGB565) as pixel format 
  *           - The frame buffer is located at FLASH memory
  *           - The Layer size configuration : 240x160                      
  * @retval
  *  None
  */
static void LCD_Config(void)
{  
	LTDC_LayerCfgTypeDef pLayerCfg;
	LTDC_LayerCfgTypeDef pLayerCfg1;

	  /* Initialization of ILI9341 component*/
	ili9341_Init();
  
	/* LTDC Initialization -------------------------------------------------------*/
  
	  /* Polarity configuration */
	  /* Initialize the horizontal synchronization polarity as active low */
	LtdcHandle.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	/* Initialize the vertical synchronization polarity as active low */ 
	LtdcHandle.Init.VSPolarity = LTDC_VSPOLARITY_AL; 
	/* Initialize the data enable polarity as active low */ 
	LtdcHandle.Init.DEPolarity = LTDC_DEPOLARITY_AL; 
	/* Initialize the pixel clock polarity as input pixel clock */  
	LtdcHandle.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  
	/* Timing configuration  (Typical configuration from ILI9341 datasheet)
	    HSYNC=10 (9+1)
	    HBP=20 (29-10+1)
	    ActiveW=240 (269-20-10+1)
	    HFP=10 (279-240-20-10+1)

	          VSYNC=2 (1+1)
	          VBP=2 (3-2+1)
	          ActiveH=320 (323-2-2+1)
	          VFP=4 (327-320-2-2+1)
	        */  

	          /* Timing configuration */
	          /* Horizontal synchronization width = Hsync - 1 */  
	LtdcHandle.Init.HorizontalSync = 9;
	/* Vertical synchronization height = Vsync - 1 */
	LtdcHandle.Init.VerticalSync = 1;
	/* Accumulated horizontal back porch = Hsync + HBP - 1 */
	LtdcHandle.Init.AccumulatedHBP = 29;
	/* Accumulated vertical back porch = Vsync + VBP - 1 */
	LtdcHandle.Init.AccumulatedVBP = 3; 
	/* Accumulated active width = Hsync + HBP + Active Width - 1 */ 
	LtdcHandle.Init.AccumulatedActiveH = 323;
	/* Accumulated active height = Vsync + VBP + Active Heigh - 1 */
	LtdcHandle.Init.AccumulatedActiveW = 269;
	/* Total height = Vsync + VBP + Active Heigh + VFP - 1 */
	LtdcHandle.Init.TotalHeigh = 327;
	/* Total width = Hsync + HBP + Active Width + HFP - 1 */
	LtdcHandle.Init.TotalWidth = 279;
  
	/* Configure R,G,B component values for LCD background color */
	LtdcHandle.Init.Backcolor.Blue = 0;
	LtdcHandle.Init.Backcolor.Green = 0;
	LtdcHandle.Init.Backcolor.Red = 0;

	LtdcHandle.Instance = LTDC;
  
	/* Layer1 Configuration ------------------------------------------------------*/
  
	  /* Windowing configuration */ 
	pLayerCfg.WindowX0 = 0;
	pLayerCfg.WindowX1 = 240;
	pLayerCfg.WindowY0 = 0;
	pLayerCfg.WindowY1 = 160;
  
	/* Pixel Format configuration*/ 
	pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  
	/* Start Address configuration : frame buffer is located at FLASH memory */
	//pLayerCfg.FBStartAdress = (uint32_t)&ST_LOGO_1;
	pLayerCfg.FBStartAdress = FRAMEBUFFER_ADDR; 
  
	/* Alpha constant (255 totally opaque) */
	pLayerCfg.Alpha = 255;
  
	/* Default Color configuration (configure A,R,G,B component values) */
	pLayerCfg.Alpha0 = 0;
	pLayerCfg.Backcolor.Blue = 0;
	pLayerCfg.Backcolor.Green = 0;
	pLayerCfg.Backcolor.Red = 0;
  
	/* Configure blending factors */
	pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
	pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  
	/* Configure the number of lines and number of pixels per line */
	pLayerCfg.ImageWidth = 240;
	pLayerCfg.ImageHeight = 160;

	/* Layer2 Configuration ------------------------------------------------------*/
  
	  /* Windowing configuration */
	pLayerCfg1.WindowX0 = 0;
	pLayerCfg1.WindowX1 = 240;
	pLayerCfg1.WindowY0 = 160;
	pLayerCfg1.WindowY1 = 320;
  
	/* Pixel Format configuration*/ 
	pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  
	/* Start Address configuration : frame buffer is located at FLASH memory */
	//pLayerCfg1.FBStartAdress = (uint32_t)&ST_LOGO_1;
	pLayerCfg1.FBStartAdress = FRAMEBUFFER_ADDR;
  
	/* Alpha constant (255 totally opaque) */
	pLayerCfg1.Alpha = 100;
  
	/* Default Color configuration (configure A,R,G,B component values) */
	pLayerCfg1.Alpha0 = 0;
	pLayerCfg1.Backcolor.Blue = 100;
	pLayerCfg1.Backcolor.Green = 0;
	pLayerCfg1.Backcolor.Red = 0;
  
	/* Configure blending factors */
	pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
	pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  
	/* Configure the number of lines and number of pixels per line */
	pLayerCfg1.ImageWidth = 240;
	pLayerCfg1.ImageHeight = 160;  
   
	/* Configure the LTDC */  
	if (HAL_LTDC_Init(&LtdcHandle) != HAL_OK)
	{
	  /* Initialization Error */
		Error_Handler(); 
	}

	  /* Configure the Background Layer*/
	if (HAL_LTDC_ConfigLayer(&LtdcHandle, &pLayerCfg, 0) != HAL_OK)
	{
	  /* Initialization Error */
		Error_Handler(); 
	}
  
	/* Configure the Foreground Layer*/
	if (HAL_LTDC_ConfigLayer(&LtdcHandle, &pLayerCfg1, 1) != HAL_OK)
	{
	  /* Initialization Error */
		Error_Handler(); 
	}  
}

static void Error_Handler(void)
{
	BSP_LED_On(LED4);
	while (1) {}
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
