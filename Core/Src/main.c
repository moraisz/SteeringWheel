/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"
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
extern USBD_HandleTypeDef hUsbDeviceFS;
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
    MX_USB_DEVICE_Init();
    MX_TIM5_Init();
    MX_ADC1_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    // HAL_Delay(100); // Waiting load USB stack
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    // Tick variables for timing
    static uint32_t last_tick = 0;
    uint32_t current_tick = 0;

    // Array to hold the HID report data
    uint8_t report[6];

    // Variables for encoder readings
    int16_t encoder_last_axis = 0;
    int16_t encoder_axis = 0;
    int32_t encoder_position = 0;
    int16_t encoder_delta = 0;
    float encoder_angle = 0.0f;
    int16_t encoder_scaled = 0;

    // Start the ADC in DMA mode to read 4 channels
    static uint16_t adc_buffer[4];  // Para armazenar os valores dos 4 canais
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 4);

    uint16_t pot1 = 0; // Canal 3 (PA3)
    uint16_t pot2 = 0; // Canal 4 (PA4)

    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        current_tick = HAL_GetTick();
        if ((current_tick - last_tick) >= 2) { // Aproximadamente 2ms de intervalo
            last_tick = current_tick;
            encoder_axis = __HAL_TIM_GET_COUNTER(&htim5);  // Read the current position of the encoder
            encoder_delta = encoder_axis - encoder_last_axis;  // Calculate the difference (delta) since the last reading

            if (encoder_delta != 0)  // Only send a encoder report if there was movement
            {
                // Accumulate the encoder position and update the last axis
                encoder_position += encoder_delta;
                encoder_last_axis = encoder_axis;

                // Clamp the angle to the range [-450, 450] degrees
                encoder_angle = ((float)encoder_position / 2400.0f) * 360.0f;
                if (encoder_angle > 450.0f) encoder_angle = 450.0f;
                if (encoder_angle < -450.0f) encoder_angle = -450.0f;

                // Scale the angle to the int16_t range [-32768, 32767]
                encoder_scaled = (int16_t)((encoder_angle / 450.0f) * 32767.0f);
            }

            pot1 = adc_buffer[0]; // Canal 3 (PA3)
            pot2 = adc_buffer[1]; // Canal 4 (PA4)

            report[0] = encoder_scaled & 0xFF;
            report[1] = (encoder_scaled >> 8) & 0xFF;
            report[2] = pot1 & 0xFF;
            report[3] = (pot1 >> 8) & 0xFF;
            report[4] = pot2 & 0xFF;
            report[5] = (pot2 >> 8) & 0xFF;

            if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
                USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }
        }
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
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
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
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
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
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
