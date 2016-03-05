/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "main.h"
#include <math.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
volatile uint8_t newData = 0;
volatile uint8_t rcRawData[16];
volatile uint8_t rcCounter = 0;
volatile uint16_t rcData[16];

int _write(int fd, char *pBuffer, int size)
{
	HAL_UART_Transmit(&huart1, pBuffer, size, 200);

	return size;
}

static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data) {
	return HAL_I2C_Mem_Write(&hi2c2, slave_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, (unsigned char *) data, length, 200);
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
	return HAL_I2C_Mem_Read(&hi2c2, slave_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, length, 200);
}

void delay_ms(unsigned long num_ms) {
	HAL_Delay(num_ms);
}

void get_ms(unsigned long *count) {
	if (count) *count = HAL_GetTick();
}

int reg_int_cb(struct int_param_s *int_param) {
	return 0;
}

int min(int a, int b) {
	return ((a<b)?a:b);
}

void __no_operation() {
	asm volatile("nop");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) newData = 1;
}

void sysTickMain(void) {
	static uint16_t prevRxXferCount;
	uint8_t i;

	if (prevRxXferCount != huart2.RxXferCount) {
		// Nollställ rcCounter om nytt tecken tagits emot.
		rcCounter = 0;
		prevRxXferCount = huart2.RxXferCount;
	}

	if (rcCounter == 5) {
		// Avbrott mer än 5 ms. Starta ny mottagning.
		__HAL_UART_FLUSH_DRREGISTER(&huart2);
		HAL_UART_Receive_IT(&huart2, (uint8_t *)rcRawData, 16);
	}

	if (rcCounter == 20) {
		// Timeout!
		for(i = 0; i < 16; i++) rcData[i] = 0;

		__HAL_UART_FLUSH_DRREGISTER(&huart2);
		HAL_UART_Receive_IT(&huart2, (uint8_t *)rcRawData, 16);
	}
	if (rcCounter < 255) rcCounter++;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uint8_t i;
	if (huart == &huart2) {
		// Tolka data
		for(i = 0; i < 7; i++) {
			rcData[(rcRawData[(i * 2) + 2] >> 3) & 0x0F] = ((rcRawData[(i * 2) + 2] & 0x07) * 0xFF) + rcRawData[(i * 2) + 3];
		}
	}
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	short sensors;
	unsigned char more;
	long quat[4];
	double pitch, yaw, roll;
	double qx, qy, qz, qw;
	char t = 'B';

	//extern void initialise_monitor_handles(void);

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  //initialise_monitor_handles();

  /*while(1) {
  	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
  	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
  	HAL_Delay(500);
  }*/

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  // 125-250 us, 24 MHz, 125 us = 3000, 250 us = 6000

  /*while(1) {
  	  htim4.Instance->EGR = TIM_EGR_UG;
  	  //while (htim4.Instance->EGR & TIM_EGR_UG); // Verkar inte behövas om timer clock är dubbelt så snabb som peripheral clock?
  	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ((rcData[0] / 2) * 3) + 2811);
  	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 4000);
  	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 5000);
  	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 5000);
  	  htim4.Instance->EGR = TIM_EGR_UG;
  	  //while (htim4.Instance->EGR & TIM_EGR_UG);
  	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

    	HAL_Delay(10); // Inte mer än 10 ms mellan pulser, annars verkar inte BLHeli starta upp
  }*/

  HAL_Delay(200);

  mpu_init(NULL);
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  dmp_load_motion_driver_firmware();
  dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
  dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT);
  dmp_set_fifo_rate(10);
  mpu_set_dmp_state(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  	if (newData) {
  			dmp_read_fifo(NULL, NULL, quat, NULL, &sensors, &more);
  			if (sensors == INV_WXYZ_QUAT) {
  				qw = ((double) quat[0]) / 1073741824.0;
  				qx = ((double) quat[1]) / 1073741824.0;
  				qy = ((double) quat[2]) / 1073741824.0;
  				qz = ((double) quat[3]) / 1073741824.0;

  				roll = atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz);
  				pitch = asin(2.0*(qx*qz - qw*qy));
  				yaw = atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);

  				printf("Yaw: %d Pitch: %d Roll: %d Throttle: %d\r\n", (int16_t)(yaw * (180.0 / M_PI)), (int16_t)(pitch * (180.0 / M_PI)), (int16_t)(roll * (180.0 / M_PI)), rcData[0]);
  			}

  			//if ((hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) && (hUsbDeviceFS.ep0_state != USBD_EP0_STATUS_IN)) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0); else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  			if (more == 0) newData = 0;
  	}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c2);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);

  HAL_TIM_MspPostInit(&htim4);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOD_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
