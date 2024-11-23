/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "gpio.h"

// NAND Flash Command Definitions (based on datasheet)
#define NAND_CMD_READ1  0x00
#define NAND_CMD_READ2  0x30
#define NAND_CMD_PROGRAM1 0x80
#define NAND_CMD_PROGRAM2 0x10
#define NAND_CMD_ERASE1 0x60
#define NAND_CMD_ERASE2 0xD0
#define NAND_CMD_STATUS 0x70
#define NAND_CMD_RESET  0xFF

// NAND Control Pins
#define CLE_PIN GPIO_PIN_0  // Command Latch Enable
#define ALE_PIN GPIO_PIN_1  // Address Latch Enable
#define WE_PIN  GPIO_PIN_3  // Write Enable
#define RE_PIN  GPIO_PIN_4  // Read Enable
#define CE_PIN  GPIO_PIN_5  // Chip Enable
#define RYBY_PIN GPIO_PIN_6 // Ready/Busy

#define NAND_GPIO GPIOB // GPIO Port for control pins
#define NAND_DATA_PORT GPIOA // GPIO Port for data I/O


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
/* USER CODE BEGIN PFP */
void NAND_Init(void);
void NAND_WriteCommand(uint8_t cmd);
void NAND_WriteAddress(uint8_t addr);
void NAND_WriteData(uint8_t data);
uint8_t NAND_ReadData(void);
void NAND_WaitUntilReady(void);
void NAND_ReadPage(uint32_t pageAddr, uint8_t *buffer);
void NAND_WritePage(uint32_t pageAddr, uint8_t *buffer);

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
  /* USER CODE BEGIN 2 */
  NAND_Init();

  uint8_t writeData[4224] = {0xAA}; // Example data
  uint8_t readData[4224] = {0};

  // Write Example
  NAND_WritePage(0x000000, writeData);
  //Read Example
  NAND_ReadPage(0x000000, readData);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Initialize GPIO pins for NAND Flash communication
void NAND_Init(void) {
    HAL_GPIO_WritePin(NAND_GPIO, CLE_PIN | ALE_PIN | WE_PIN | RE_PIN | CE_PIN, GPIO_PIN_SET);
}

// Send a command to NAND Flash
void NAND_WriteCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(NAND_GPIO, CLE_PIN, GPIO_PIN_SET); // Enable CLE
    HAL_GPIO_WritePin(NAND_GPIO, ALE_PIN, GPIO_PIN_RESET); // Disable ALE

    *(volatile uint8_t *)&NAND_DATA_PORT->ODR = cmd; // Output command to data pins
    HAL_GPIO_WritePin(NAND_GPIO, WE_PIN, GPIO_PIN_RESET); // Pulse WE low
    HAL_GPIO_WritePin(NAND_GPIO, WE_PIN, GPIO_PIN_SET);

    HAL_GPIO_WritePin(NAND_GPIO, CLE_PIN, GPIO_PIN_RESET); // Disable CLE
}

// Send an address to NAND Flash
void NAND_WriteAddress(uint8_t addr) {
    HAL_GPIO_WritePin(NAND_GPIO, CLE_PIN, GPIO_PIN_RESET); // Disable CLE
    HAL_GPIO_WritePin(NAND_GPIO, ALE_PIN, GPIO_PIN_SET); // Enable ALE

    *(volatile uint8_t *)&NAND_DATA_PORT->ODR = addr; // Output address to data pins
    HAL_GPIO_WritePin(NAND_GPIO, WE_PIN, GPIO_PIN_RESET); // Pulse WE low
    HAL_GPIO_WritePin(NAND_GPIO, WE_PIN, GPIO_PIN_SET);

    HAL_GPIO_WritePin(NAND_GPIO, ALE_PIN, GPIO_PIN_RESET); // Disable ALE
}

// Write data to NAND Flash
void NAND_WriteData(uint8_t data) {
    *(volatile uint8_t *)&NAND_DATA_PORT->ODR = data; // Output data to data pins
    HAL_GPIO_WritePin(NAND_GPIO, WE_PIN, GPIO_PIN_RESET); // Pulse WE low
    HAL_GPIO_WritePin(NAND_GPIO, WE_PIN, GPIO_PIN_SET);
}

// Read data from NAND Flash
uint8_t NAND_ReadData(void) {
    HAL_GPIO_WritePin(NAND_GPIO, RE_PIN, GPIO_PIN_RESET); // Pulse RE low
    uint8_t data = *(volatile uint8_t *)&NAND_DATA_PORT->IDR; // Read data from data pins
    HAL_GPIO_WritePin(NAND_GPIO, RE_PIN, GPIO_PIN_SET);

    return data;
}

// Wait until NAND Flash is ready (RY/BY pin high)
void NAND_WaitUntilReady(void) {
    while (HAL_GPIO_ReadPin(NAND_GPIO, RYBY_PIN) == GPIO_PIN_RESET);
}

// Read a page from NAND Flash
void NAND_ReadPage(uint32_t pageAddr, uint8_t *buffer) {
    // Send READ command
    NAND_WriteCommand(NAND_CMD_READ1);

    NAND_WriteAddress((pageAddr >> 0) & 0xFF);
    NAND_WriteAddress((pageAddr >> 8) & 0xFF);
    NAND_WriteAddress((pageAddr >> 16) & 0xFF);
    NAND_WriteAddress(0x00); // Dummy cycles for column
    NAND_WriteAddress(0x00);

    // Confirm Read
    NAND_WriteCommand(NAND_CMD_READ2);

    // Wait until NAND is ready
    NAND_WaitUntilReady();

    // Read data from NAND
    for (int i = 0; i < 4224; i++) {
        buffer[i] = NAND_ReadData();
    }
}

// Write a page to NAND Flash
void NAND_WritePage(uint32_t pageAddr, uint8_t *buffer) {
    // Send PROGRAM command
    NAND_WriteCommand(NAND_CMD_PROGRAM1);

    // Send page address (5 cycles)
    NAND_WriteAddress((pageAddr >> 0) & 0xFF);
    NAND_WriteAddress((pageAddr >> 8) & 0xFF);
    NAND_WriteAddress((pageAddr >> 16) & 0xFF);
    NAND_WriteAddress(0x00); // Dummy cycles for column
    NAND_WriteAddress(0x00);

    // Write data to NAND
    for (int i = 0; i < 4224; i++) {
        NAND_WriteData(buffer[i]);
    }

    // Confirm Write
    NAND_WriteCommand(NAND_CMD_PROGRAM2);

    // Wait until NAND is ready
    NAND_WaitUntilReady();
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
