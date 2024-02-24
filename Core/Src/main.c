/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "MLX90640_API.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define  FPS2HZ   0x02
#define  FPS4HZ   0x03
#define  FPS8HZ   0x04
#define  FPS16HZ  0x05
#define  FPS32HZ  0x06
#define  MLX90640_ADDR 0x33

#define  TA_SHIFT 8 //Default shift for MLX90640 in open air

#define USB_SET_CMD	(0xAA)	/* usb cdc 控制下发此命令，则认为控制mlx90640 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

static uint16_t eeMLX90640[832];  
static float mlx90640To[768];
static int16_t usb_buffer[768 + 8];	//四个 0xFFFF帧头�?+ 四个 0xEFEF 帧尾
uint16_t frame[834];
float emissivity=0.95;

/* fps 设置 */
uint8_t	RefreshRate = FPS16HZ; 
uint8_t delay_fps = 16;	//默认16

enum type_def{
	type_null				= 0,
	type_set_fps,
	type_get_fps,
} ;

#define USB_CONTROL_DATA_SIZE	(12)
volatile struct {
	uint8_t flag;
	uint8_t type;
	uint8_t length;
	uint8_t data[USB_CONTROL_DATA_SIZE];
	uint8_t data_tx[USB_CONTROL_DATA_SIZE];
} usb_control_hand;

paramsMLX90640 mlx90640;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void usb_control_handler(void);
void mlx90640_refresh(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t usb_send_str[] = "this message from stm32!";
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
	int status;
	
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

	
	
	HAL_Delay(1500);	/* 等待mlx90640稳定 */

	status = MLX90640_SetRefreshRate(MLX90640_ADDR, RefreshRate);
	if (status != 0) printf("\r\nMLX90640_SetRefreshRate error with code:%d\r\n",status);
	
	status = MLX90640_SetChessMode(MLX90640_ADDR);
	if (status != 0) printf("\r\nMLX90640_SetChessMode error with code:%d\r\n",status);

  status = MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);
  if (status != 0) printf("\r\nload system parameters error with code:%d\r\n",status);

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0) printf("\r\nParameter extraction failed with error code:%d\r\n",status);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		usb_control_handler();
		mlx90640_refresh();
		
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

FILE __stdout;

int fputc(int c, FILE *f)
{
	uint8_t byte = (uint8_t)c;
	f = f;
	HAL_UART_Transmit(&huart1, (const uint8_t * )&byte, 1, 20);
	return 0;
}


/**
  * @brief  CDC 控制接口函数的转发函�?
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval null
  */
void CDC_Control_FS_forwarding(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
	if ((cmd == USB_SET_CMD) && (length > 0))
	{
		usb_control_hand.type = pbuf[0];
		if ((length - 1) <= USB_CONTROL_DATA_SIZE)
		{
			usb_control_hand.length = length - 1;
			for (uint8_t i = 0; i < (length - 1); i++)
			{
				usb_control_hand.data[i] = pbuf[1 + i];
			}
		}
		usb_control_hand.flag = 1;
	}
}


/**
  * @brief  处理usb控制端口的数�?
  * @retval null
  */
void usb_control_handler(void)
{
	if (usb_control_hand.flag)
	{
		switch (usb_control_hand.type)
		{
			case type_set_fps:
			{
				if (usb_control_hand.data[0] == 2)
				{
					if (MLX90640_SetRefreshRate(MLX90640_ADDR, FPS2HZ) == 0)
					{
						RefreshRate = FPS2HZ;
						delay_fps = 2;
					}
					else
					{
						printf("set fps error!\r\n");
					}
				}
				else if (usb_control_hand.data[0] == 4)
				{
					if (MLX90640_SetRefreshRate(MLX90640_ADDR, FPS4HZ) == 0)
					{
						RefreshRate = FPS4HZ;
						delay_fps = 4;
					}
					else
					{
						printf("set fps error!\r\n");
					}
				}
				else if (usb_control_hand.data[0] == 8)
				{
					if (MLX90640_SetRefreshRate(MLX90640_ADDR, FPS8HZ) == 0)
					{
						RefreshRate = FPS8HZ;
						delay_fps = 8;
					}
					else
					{
						printf("set fps error!\r\n");
					}
				}
				else if (usb_control_hand.data[0] == 16)
				{
					if (MLX90640_SetRefreshRate(MLX90640_ADDR, FPS16HZ) == 0)
					{
						RefreshRate = FPS16HZ;
						delay_fps = 16;
					}
					else
					{
						printf("set fps error!\r\n");
					}
				}
				else if (usb_control_hand.data[0] == 32)
				{
					if (MLX90640_SetRefreshRate(MLX90640_ADDR, FPS32HZ) == 0)
					{
						RefreshRate = FPS32HZ;
						delay_fps = 32;
					}
					else
					{
						printf("set fps error!\r\n");
					}
				}
				break;
			}
			case type_get_fps:
			{
				break;
			}
		}

		usb_control_hand.flag = 0;
	}
}


volatile static uint8_t usb_busy_flag = 0;
/**
  * @brief  采集 mlx 数据并上�?
  * @retval null
  */
void mlx90640_refresh(void)
{
	int status;

	status = MLX90640_GetFrameData(MLX90640_ADDR, frame);
	if (status < 0)
	{
		printf("GetFrame Error: %d\r\n",status);
		HAL_Delay(500);
		return;
	}

	float vdd = MLX90640_GetVdd(frame, &mlx90640);
	float Ta = MLX90640_GetTa(frame, &mlx90640);
	
	float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature

	MLX90640_CalculateTo(frame, &mlx90640, emissivity , tr, mlx90640To);
	
	for(int i = 0; i < 768; i++){
		usb_buffer[4 + i] = (int16_t)(mlx90640To[i]*100);
	}

	while (usb_busy_flag){HAL_Delay(20);}
	usb_buffer[0] = 0xFFFF;
	usb_buffer[1] = 0xFFFF;
	usb_buffer[2] = 0xFFFF;
	usb_buffer[3] = 0xFFFF;
	usb_buffer[(sizeof(usb_buffer)/2) - 1] = (int16_t)0xEFEF;
	usb_buffer[(sizeof(usb_buffer)/2) - 2] = (int16_t)0xEFEF;
	usb_buffer[(sizeof(usb_buffer)/2) - 3] = (int16_t)0xEFEF;
	usb_buffer[(sizeof(usb_buffer)/2) - 4] = (int16_t)0xEFEF;
	
	usb_busy_flag = 1;
	CDC_Transmit_FS((uint8_t *)usb_buffer, sizeof(usb_buffer));
	
	//HAL_Delay(20);
}


void usb_send_done_callback(void)
{
	usb_busy_flag = 0;
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
