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
#include "app_config.h"
#include "app_ThreePhase.h"
#include "app_pfc.h"
#include "app_signal.h"
#include "arm_math.h"
#include "stm32f4xx_ll_adc.h"
#include <stdio.h>
#include <string.h>
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 以前：volatile uint16_t adc_buffer[1];
// 现在：必须能装下 3 个通道的数据！
/* ADC1: Ia, Va. ADC2: Ib, Vb. ADC3: Vdc. */
volatile uint16_t adc1_buffer[3];
volatile uint16_t adc2_buffer[2];
volatile uint16_t adc3_buffer[2];
float DC_BUS_SENSOR_SCALE = 1.0f;
float VOLTAGE_A_SCALE = 40.0f;
float VOLTAGE_B_SCALE = 40.0f;

#define ADC_REFERENCE_VOLTAGE 3.3f
#define ADC_FULL_SCALE         4095.0f
#define ADC_VREFINT_INDEX      2U
#define CURRENT_A_ADC_OFFSET   2020.7f
#define CURRENT_B_ADC_OFFSET   2047.5f
#define VOLTAGE_A_ADC_OFFSET   2048.0f
#define VOLTAGE_B_ADC_OFFSET   2048.0f
float VOLTAGE_SENSOR_SCALE = 1;// 硬件互感器的倍数,还原成220/311V
float CURRENT_SENSOR_SCALE = 1;// 硬件互感器的倍数,还原成真实的电流
float VOLTAGE_DC_SCALE = 1;// 硬件互感器的倍数,还原成真实的电压
float CURRENT_A_SCALE = 5;// 硬件互感器的倍数,还原成真实的电流
float CURRENT_B_SCALE = 5;// 硬件互感器的倍数,还原成真实的电流
float CURRENT_C_SCALE = 5;// 硬件互感器的倍数,还原成真实的电流










float ia_real, va_real, ib_real, vb_real, ic_real, vc_real = 0.0f;

/* PFC sensor raw values for OLED display */
volatile float pfc_ui_real  = 0.0f;
volatile float pfc_ii_real  = 0.0f;
volatile float pfc_vdc_real = 0.0f;

/* PFC input sensors: tune these independently from inverter-side sensors. */
float PFC_UI_ADC_OFFSET = 2047.5f;
float PFC_II_ADC_OFFSET = 2021.67f;
float PFC_UI_SENSOR_SCALE = 34.50f;
float PFC_II_SENSOR_SCALE = 4.6f;
volatile float adc_vdda = ADC_REFERENCE_VOLTAGE;
float PFC_VDC_SENSOR_SCALE = 27.0f;   //母线电压检测

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static float theta_3ph = 0.0f;

#define THREE_PHASE_FREQ_HZ  60.0f
#define DQ_THETA_OFFSET      (-0.5f * PI_VALUE)
#define THREE_PHASE_STEP \
    (2.0f * PI_VALUE * THREE_PHASE_FREQ_HZ * CONTROL_SAMPLE_TIME)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
APP_PLL_t System_PLL;
SPWM_t my_spwm;
DualLoop_t My_DualLoop;
RMS_t Vout_RMS;
APP_ThreePhase_t ThreePhase_Control;
APP_PFC_t Pfc_Control;
static volatile uint8_t adc_ready_mask;
static volatile uint8_t pfc_adc_ready_mask;
static APP_OffsetCal_t Pfc_Ui_OffsetCal;
static APP_OffsetCal_t Pfc_Ii_OffsetCal;
static APP_LowPass_t Pfc_Ui_Filter;
static APP_LowPass_t Pfc_Ii_Filter;
static APP_LowPass_t Pfc_Vdc_Filter;
static uint8_t  pfc_adc_calibration_done;
static uint32_t pfc_startup_settle_count;  /* 上电后跳过前 N 次采样，等模拟前端稳定 */
#if PFC_SIMULATION_ENABLE == 1
static volatile float pfc_sim_ui;
static volatile float pfc_sim_ii;
static volatile float pfc_sim_error;
static volatile uint8_t pfc_sim_print_ready;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1U, HAL_MAX_DELAY);
  return ch;
}

int fputc(int ch, FILE *f)
{
  (void)f;
  return __io_putchar(ch);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void ADC_UpdateVdda(void)
{
  uint32_t vrefint_raw = adc1_buffer[ADC_VREFINT_INDEX];

  if (vrefint_raw != 0U)
  {
    float new_vdda = (float)__LL_ADC_CALC_VREFANALOG_VOLTAGE(
                         vrefint_raw,
                         LL_ADC_RESOLUTION_12B) / 1000.0f;

    if ((new_vdda > 2.7f) && (new_vdda < 3.6f))
    {
      adc_vdda = adc_vdda * 0.95f + new_vdda * 0.05f;
    }
  }
}

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
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	//10kHz
	APP_PLL_Init(&System_PLL, CONTROL_SAMPLE_TIME);
  SPWM_Init(&my_spwm,100);
	DualLoop_Init(&My_DualLoop, CONTROL_SAMPLE_TIME);
  RMS_Init(&Vout_RMS);
  APP_ThreePhase_Init(&ThreePhase_Control, &my_spwm);
  APP_PFC_Init(&Pfc_Control);
  APP_PFC_SetPwmEnable(&Pfc_Control, 0U);
  APP_OffsetCal_Init(&Pfc_Ui_OffsetCal,
                     PFC_OFFSET_CAL_SAMPLES, PFC_UI_ADC_OFFSET);
  APP_OffsetCal_Init(&Pfc_Ii_OffsetCal,
                     PFC_OFFSET_CAL_SAMPLES, PFC_II_ADC_OFFSET);
  APP_LowPass_Init(&Pfc_Ui_Filter,
                   CONTROL_SAMPLE_TIME, PFC_UI_LPF_CUTOFF_HZ);
  APP_LowPass_Init(&Pfc_Ii_Filter,
                   CONTROL_SAMPLE_TIME, PFC_II_LPF_CUTOFF_HZ);
  APP_LowPass_Init(&Pfc_Vdc_Filter,
                   CONTROL_SAMPLE_TIME, PFC_VDC_LPF_CUTOFF_HZ);
  PR_Init(&My_DualLoop.Current_PR,
          0.1f, 10.0f,
          50.0f, 5.0f, CONTROL_SAMPLE_TIME,
          0.98f, -0.98f);
  //PR_Init(PR_Controller_t *pr, float kp, float kr, float target_freq_hz, float bandwidth_hz, float sample_time, float max_out, float min_out)

//接口函数声明
  TIM3->CCR1 = 50;
  TIM3->CCR2 = 50;
  TIM3->CCR3 = 50;
  TIM3->CCR4 = 50;
  TIM9->CCR1 = 50;
  TIM9->CCR2 = 50;
  TIM4->CCR1 = 50 + PFC_SOFTWARE_DEADTIME_COUNTS;
  TIM4->CCR2 = 50;
  TIM5->CCR2 = 50 + PFC_SOFTWARE_DEADTIME_COUNTS;
  TIM5->CCR3 = 50;

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buffer, 3);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buffer, 2);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc3_buffer, 2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim9);
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
#if PFC_PWM_TEST_ENABLE == 1
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
#endif

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim6);
  OLED_Init();
  OLED_Clear();
  OLED_Update();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ---- OLED 刷新 (10Hz，避免 I2C 阻塞主循环) ---- 
    {
      static uint32_t oled_last_tick = 0;
      uint32_t now = HAL_GetTick();

      if (now - oled_last_tick >= 100U) {
        oled_last_tick = now;

        if (pfc_adc_calibration_done == 0U) {
          OLED_Printf(0, 0, OLED_8X16, "PFC Calibrating...");
          OLED_Printf(0, 16, OLED_8X16, "Ui offset:%4.0f",
                      (double)Pfc_Ui_OffsetCal.Offset);
          OLED_Printf(0, 32, OLED_8X16, "Ii offset:%4.0f",
                      (double)Pfc_Ii_OffsetCal.Offset);
        } else {
          OLED_Printf(0, 0, OLED_8X16, "Ui:%.1fV", (double)pfc_ui_real);
          OLED_Printf(0, 16, OLED_8X16, "Ii:%.2fA", (double)pfc_ii_real);
          OLED_Printf(0, 32, OLED_8X16, "Vdc:%.1fV", (double)pfc_vdc_real);
        }
        OLED_Update();
      }
    }
    */


    /* ---- 串口打印: Ui, Iref, cos_theta, theta, Q, delta_omega, Freq (1ms) ----
       列1 Ui          : 电网电压
       列2 Iref        : 电流参考 (应为正弦波)
       列3 cos_theta   : cos(PLL.theta) (应为正弦波, 幅值1)
       列4 theta       : PLL.theta (应线性增长 0→2π 循环)
       列5 park.Q      : Park变换Q轴分量 (锁相后应≈0)
       列6 delta_omega : PI输出角速度补偿 (未饱和时应≈0)
       列7 Freq        : PLL实时频率 (应≈50Hz)

       诊断:
       - 列4 theta 如果不增长 → PLL角度积分有问题
       - 列3 cos_theta 如果是常数 → theta 没转或转速极慢
       - 列5 Q 如果很大 → PLL没锁住
       - 列6 如果饱和在±50 → PI参数/信号有问题 */
    {
#if PFC_SIMULATION_ENABLE == 1
      if (pfc_sim_print_ready != 0U) {
        pfc_sim_print_ready = 0U;
        printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
               (double)pfc_sim_ui,
               (double)pfc_sim_ii,
               (double)Pfc_Control.Debug_Iref,
               (double)pfc_sim_error,
               (double)Pfc_Control.Debug_Current_Control,
               (double)Pfc_Control.Debug_Modulation);
      }
#else

       printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
              (double)pfc_ui_real,
              (double)pfc_ii_real,
							(double)pfc_vdc_real,
              (double)Pfc_Control.Debug_Iref,
              (double)Pfc_Control.Debug_Current_Control,
              (double)Pfc_Control.Debug_Modulation
             );
			 
//			        printf("%.4f,%.4f\r\n",
//              (double)pfc_ui_real,
//              (double)pfc_ii_real
//             );
        
     
			// printf("%.3f,%d,%d,%d\r\n",
      //  (double)theta_3ph,
      //  my_spwm.CCR1_Value,
      //  my_spwm.CCR2_Value,
      //  my_spwm.CCR3_Value);

#endif
    }

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
#if PLL_USE == 1
// 在 HAL 库的 DMA 转换完成回调函数中
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5); // 仅作为示例，指示 ADC 转换完成
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
        APP_PLL_Update(&System_PLL, v_grid_real);

			  #if USE_MODE == 1
				

				float v_ref = 311.0f * arm_sin_f32(System_PLL.theta);//求瞬时电压值

				float control = PI_Update(&My_DualLoop.Voltage_PI, v_ref, v_out_real);

				SPWM_Update_ByControl(&my_spwm, control);			   
				 
				#endif	
				
				#if USE_MODE == 2
				

				float v_out_rms = RMS_Update(&Vout_RMS, v_out_real);

				float i_amp_ref = PI_Update(&My_DualLoop.Voltage_PI, 220.0f, v_out_rms);

				float i_ref = i_amp_ref * arm_sin_f32(System_PLL.theta);

				float control = PR_Update(&My_DualLoop.Current_PR, i_ref, i_out_real);

				SPWM_Update_ByControl(&my_spwm, control);  
				 
				#endif	
//        // ==========================================================
//        // 2. 算法处理：节拍器 (PLL)
//        // ==========================================================
//        // 算出此刻电网的绝对相位 (System_PLL.theta)
//        APP_PLL_Update(&System_PLL, v_grid_real);


//        // ==========================================================
//        // 3. 算法处理：能量泵 (电压电流双环)
//        // ==========================================================
//        // 假设我们要让系统死死稳住输出 220.0V (这里的220.0f是目标值)
//        // 丢入真实的输出电压和电流，双环 PI 拼命计算，吐出最新的调制比！
//        float dynamic_amplitude = DualLoop_Update(&My_DualLoop, 220.0f, v_out_real, i_out_real);


//        // ==========================================================
//        // 4. 算法融合：数学造波 (SPWM)
//        // ==========================================================
//        // 🚨 修正了你刚才代码里的 0.8f 漏洞！
//        // 把 PLL 算出的节拍 (theta) 和双环算出的能量 (dynamic_amplitude) 乘在一起！
//        SPWM_Update(&my_spwm, System_PLL.theta, dynamic_amplitude);


        // ==========================================================
        // 5. 硬件执行：推给寄存器，指挥 MOSFET 斩波
        // ==========================================================
        // TIM1_CH1 和 TIM1_CH2 互补输出控制全桥
        TIM1->CCR1 = my_spwm.CCR1_Value; 
        TIM1->CCR2 = my_spwm.CCR2_Value; 
    }
}
#endif	


#if KAI_HUAN_SAN_XIANG == 1
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
#if SVPWM_USE == 1
    float sin_theta;
    float cos_theta;
    float v_phase_peak_cmd;
    const float vdc_real = 60.0f;
#endif

    if (htim->Instance != TIM6) {
        return;
    }

    theta_3ph += THREE_PHASE_STEP;

    if (theta_3ph >= 2.0f * PI_VALUE) {
        theta_3ph -= 2.0f * PI_VALUE;
    }

#if SPWM_USE == 1

    SPWM_ThreePhase_Update(&my_spwm, theta_3ph, 0.87055f);

#elif SVPWM_USE == 1

    arm_sin_cos_f32(theta_3ph * 180.0f / PI_VALUE,
                    &sin_theta,
                    &cos_theta);

    v_phase_peak_cmd = 32.20f * 0.81649658f;

    SVPWM_Update(&my_spwm,
                 v_phase_peak_cmd * sin_theta,
                -v_phase_peak_cmd * cos_theta,
                 vdc_real);

#else
#error "Select either SPWM_USE or SVPWM_USE"
#endif

    TIM3->CCR1 = my_spwm.CCR1_Value;
    TIM3->CCR2 = my_spwm.CCR1_Value;

    TIM3->CCR3 = my_spwm.CCR2_Value;
    TIM3->CCR4 = my_spwm.CCR2_Value;

    TIM9->CCR1 = my_spwm.CCR3_Value;
    TIM9->CCR2 = my_spwm.CCR3_Value;
}
#endif

/* Three-phase closed-loop code moved to APP/app_ThreePhase.c. */
#if BI_HUAN_SAN_XIANG == 1
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        adc_ready_mask |= 0x01U;
    } else if (hadc->Instance == ADC2) {
        adc_ready_mask |= 0x02U;
    } else if (hadc->Instance == ADC3) {
        adc_ready_mask |= 0x04U;
    } else {
        return;
    }

    if (adc_ready_mask == 0x07U) {
        adc_ready_mask = 0U;
           //测试
            ia_real = ((float)adc1_buffer[0] - CURRENT_A_ADC_OFFSET) *
                (ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE) * CURRENT_A_SCALE;
            ib_real = ((float)adc2_buffer[0] - CURRENT_B_ADC_OFFSET) *
                (ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE) * CURRENT_B_SCALE;
            va_real = ((float)adc1_buffer[1] - VOLTAGE_A_ADC_OFFSET) *
                (ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE) * VOLTAGE_A_SCALE;
            vb_real = ((float)adc2_buffer[1] - VOLTAGE_B_ADC_OFFSET) *
                (ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE) * VOLTAGE_B_SCALE;    
        /*
        //三相电
        APP_ThreePhase_Update(
            &ThreePhase_Control,
            (float)adc3_buffer[0] *
                (ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE) * DC_BUS_SENSOR_SCALE,
            ((float)adc1_buffer[0] - CURRENT_A_ADC_OFFSET) *
                (ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE) * CURRENT_A_SCALE,
            ((float)adc2_buffer[0] - CURRENT_B_ADC_OFFSET) *
                (ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE) * CURRENT_B_SCALE,
            ((float)adc1_buffer[1] - VOLTAGE_A_ADC_OFFSET) *
                (ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE) * VOLTAGE_A_SCALE,
            ((float)adc2_buffer[1] - VOLTAGE_B_ADC_OFFSET) *
                (ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE) * VOLTAGE_B_SCALE);
                */
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        //APP_ThreePhase_TimerTick(&ThreePhase_Control);
    }
}
#endif

#if PFC_USE == 1
#if (PLL_USE == 1) || (BI_HUAN_SAN_XIANG == 1)
#error "PFC_USE cannot share HAL_ADC_ConvCpltCallback with PLL or three-phase closed loop"
#endif
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
#if PFC_SIMULATION_ENABLE == 0
    if (hadc->Instance == ADC1) {
        pfc_adc_ready_mask |= 0x01U;
    } else if (hadc->Instance == ADC2) {
        pfc_adc_ready_mask |= 0x02U;
    } else {
        return;
    }

    if (pfc_adc_ready_mask == 0x03U) {
        float ui_real;
        float ii_real;
        float vdc_real;

        pfc_adc_ready_mask = 0U;

        ADC_UpdateVdda();

        /* ---- 上电稳定延时：跳过前 5000 次采样 (~250ms) ----
         * ADC 基准电压、运放偏置、传感器供电需要时间建立，
         * 这段时间的采样值不可靠，不能用来做偏移校准。 */
#if PFC_AUTO_OFFSET_CAL_ENABLE == 1
        if (pfc_startup_settle_count < 5000U) {
            pfc_startup_settle_count++;
            return;
        }

        if (pfc_adc_calibration_done == 0U) {
            uint8_t ui_ready;
            uint8_t ii_ready;

            ui_ready = APP_OffsetCal_Update(&Pfc_Ui_OffsetCal,
                                             (float)adc1_buffer[0]);
            ii_ready = APP_OffsetCal_Update(&Pfc_Ii_OffsetCal,
                                             (float)adc1_buffer[1]);
            if ((ui_ready != 0U) && (ii_ready != 0U)) {
                PFC_UI_ADC_OFFSET = Pfc_Ui_OffsetCal.Offset;
                PFC_II_ADC_OFFSET = Pfc_Ii_OffsetCal.Offset;
                pfc_adc_calibration_done = 1U;

                ui_real = ((float)adc1_buffer[0] - PFC_UI_ADC_OFFSET) *
                          (adc_vdda / ADC_FULL_SCALE) *
                          PFC_UI_SENSOR_SCALE;
                ii_real = (PFC_II_ADC_OFFSET - (float)adc1_buffer[1]) *
                          (adc_vdda / ADC_FULL_SCALE) *
                          PFC_II_SENSOR_SCALE;
                vdc_real = (float)adc2_buffer[0] *
                           (adc_vdda / ADC_FULL_SCALE) *
                           PFC_VDC_SENSOR_SCALE;
                APP_PFC_CheckOverVoltage(&Pfc_Control, vdc_real);
                APP_LowPass_Reset(&Pfc_Ui_Filter, ui_real);
                APP_LowPass_Reset(&Pfc_Ii_Filter, ii_real);
                APP_LowPass_Reset(&Pfc_Vdc_Filter, vdc_real);
                APP_PFC_SetPwmEnable(&Pfc_Control, PFC_PWM_ENABLE);
            }
            return;
        }

#endif
        ui_real = ((float)adc1_buffer[0] - PFC_UI_ADC_OFFSET) *
                  (adc_vdda / ADC_FULL_SCALE) *
                  PFC_UI_SENSOR_SCALE;
        ii_real = (-PFC_II_ADC_OFFSET + (float)adc1_buffer[1]) *
                  (adc_vdda / ADC_FULL_SCALE) *
                  PFC_II_SENSOR_SCALE;
        vdc_real = (float)adc2_buffer[0] *
                   (adc_vdda / ADC_FULL_SCALE) *
                   PFC_VDC_SENSOR_SCALE;
        APP_PFC_CheckOverVoltage(&Pfc_Control, vdc_real);

        /* 保存原始值供主循环 OLED 显示 */
        pfc_ui_real  = ui_real;
        pfc_ii_real  = ii_real;
        pfc_vdc_real = vdc_real;

        APP_PFC_Update(
            &Pfc_Control,
            APP_LowPass_Update(&Pfc_Ui_Filter, ui_real),
            APP_LowPass_Update(&Pfc_Ii_Filter, ii_real),
            APP_LowPass_Update(&Pfc_Vdc_Filter, vdc_real));
    }
#else
    (void)hadc;
#endif
}
#if PFC_SIMULATION_ENABLE == 1
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static float pfc_sim_theta;
    static uint32_t pfc_sim_print_counter;
    float ui_sim;
    float ii_real;

    if (htim->Instance != TIM6) {
        return;
    }

    pfc_sim_theta += 2.0f * PI_VALUE * PFC_SIM_GRID_FREQ_HZ *
                     CONTROL_SAMPLE_TIME;
    if (pfc_sim_theta >= 2.0f * PI_VALUE) { 
        pfc_sim_theta -= 2.0f * PI_VALUE;
    }

    ui_sim = PFC_SIM_UI_PEAK * arm_sin_f32(pfc_sim_theta);
    ADC_UpdateVdda();
    ii_real = (PFC_II_ADC_OFFSET - (float)adc1_buffer[1]) *
              (adc_vdda / ADC_FULL_SCALE) * PFC_II_SENSOR_SCALE;

    pfc_ui_real = ui_sim;
    pfc_ii_real = ii_real;
    pfc_vdc_real = PFC_SIM_VDC;

    APP_PFC_Update(&Pfc_Control,
                   ui_sim,
                   APP_LowPass_Update(&Pfc_Ii_Filter, ii_real),
                   PFC_SIM_VDC);

    if (++pfc_sim_print_counter >= PFC_SIM_PRINT_DIVIDER) {
        pfc_sim_print_counter = 0U;
        pfc_sim_ui = ui_sim;
        pfc_sim_ii = Pfc_Control.Debug_Ii;
        pfc_sim_error = Pfc_Control.Debug_Iref - Pfc_Control.Debug_Ii;
        pfc_sim_print_ready = 1U;
    }
}
#endif
#endif


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
