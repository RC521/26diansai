/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_pll.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
APP_PLL_t System_PLL;
SPWM_t my_spwm;
DualLoop_t My_DualLoop;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 以前：volatile uint16_t adc_buffer[1];
// 现在：必须能装下 3 个通道的数据！
volatile uint16_t adc_buffer[3];
float VOLTAGE_SENSOR_SCALE = 1;// 硬件互感器的倍数,还原成220/311V
float CURRENT_SENSOR_SCALE = 1;// 硬件互感器的倍数,还原成真实的电流
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	//10kHz
	APP_PLL_Init(&System_PLL, 0.0001f);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 1);
	HAL_TIM_Base_Start_IT(&htim2);
	SPWM_Init(&my_spwm,100);
	DualLoop_Init(&My_DualLoop);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 在 HAL 库的 DMA 转换完成回调函数中
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        
        // ==========================================================
        // 1. 数据采集与物理量还原 (ADC Buffer -> 真实的 V / A)
        // ==========================================================
        // 假设通道顺序配置为：CH0(电网电压), CH1(输出电压), CH2(输出电流)
        
        // 提取电网电压 (给 PLL 用来追相位)
        float v_grid_pin = ((float)adc_buffer[0] - 2048.0f) * (3.3f / 4096.0f);
        float v_grid_real = v_grid_pin * VOLTAGE_SENSOR_SCALE; // 乘以硬件互感器的倍数，还原成真实的 220V/311V

        // 提取逆变器实际输出电压 (给电压外环用)
        float v_out_pin = ((float)adc_buffer[1] - 2048.0f) * (3.3f / 4096.0f);
        float v_out_real = v_out_pin * VOLTAGE_SENSOR_SCALE; 

        // 提取逆变器实际输出电流 (给电流内环用)
        float i_out_pin = ((float)adc_buffer[2] - 2048.0f) * (3.3f / 4096.0f);
        float i_out_real = i_out_pin * CURRENT_SENSOR_SCALE; 


        // ==========================================================
        // 2. 算法处理：节拍器 (PLL)
        // ==========================================================
        // 算出此刻电网的绝对相位 (System_PLL.theta)
        APP_PLL_Update(&System_PLL, v_grid_real);


        // ==========================================================
        // 3. 算法处理：能量泵 (电压电流双环)
        // ==========================================================
        // 假设我们要让系统死死稳住输出 220.0V (这里的220.0f是目标值)
        // 丢入真实的输出电压和电流，双环 PI 拼命计算，吐出最新的调制比！
        float dynamic_amplitude = DualLoop_Update(&My_DualLoop, 220.0f, v_out_real, i_out_real);


        // ==========================================================
        // 4. 算法融合：数学造波 (SPWM)
        // ==========================================================
        // 🚨 修正了你刚才代码里的 0.8f 漏洞！
        // 把 PLL 算出的节拍 (theta) 和双环算出的能量 (dynamic_amplitude) 乘在一起！
        SPWM_Update(&my_spwm, System_PLL.theta, dynamic_amplitude);


        // ==========================================================
        // 5. 硬件执行：推给寄存器，指挥 MOSFET 斩波
        // ==========================================================
        // TIM1_CH1 和 TIM1_CH2 互补输出控制全桥
        TIM1->CCR1 = my_spwm.CCR1_Value; 
        TIM1->CCR2 = my_spwm.CCR2_Value; 
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
