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
#include "fatfs.h"
#include "dubug_print.h"

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
I2S_HandleTypeDef hi2s2;


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_I2S2_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// from http://mazsola.iit.uni-miskolc.hu/~drdani/docs_arm/st/sw/Third_Party/fat_fs_R0.12c/doc/ja/readdir.html
FRESULT scan_files (
    char* path        /* 開始ノード (ワークエリアとしても使用) */
)
{
    FRESULT res;
    DIR dir;
    UINT i;
    static FILINFO fno;


    res = f_opendir(&dir, path);                       /* ディレクトリを開く */
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* ディレクトリ項目を1個読み出す */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* エラーまたは項目無しのときは抜ける */
            if (fno.fattrib & AM_DIR) {                    /* ディレクトリ */
                i = strlen(path);
                sprintf(&path[i], "/%s", fno.fname);
                res = scan_files(path);                    /* 一つ下へ */
                if (res != FR_OK) break;
                path[i] = 0;
            } else {                                       /* ファイル */
                //printf("%s/%s\n", path, fno.fname);
                debug_printf("%s/%s\n", path, fno.fname);
            }
        }
        f_closedir(&dir);
    }

    return res;
}

int sdio() {
  FATFS SDFatFs;  /* File system object for SD disk logical drive */
  FIL MyFile;     /* File object */
  char SDPath[4]; /* SD disk logical drive path */
  static uint8_t buffer[_MAX_SS]; /* a work buffer for the f_mkfs() */

  FRESULT res;                                          /* FatFs function common result code */
  uint32_t byteswritten, bytesread;                     /* File write/read counts */
  uint8_t wtext[] = "Hi, this is STM32 working with FatFs Rafale"; /* File write buffer */
  uint8_t rtext[100];                                   /* File read buffer */

  /*##-1- Link the SD disk I/O driver ########################################*/
  if(FATFS_LinkDriver(&SD_Driver, SDPath) != 0)
  {
      Error_Handler();
      return -1;
  }
  if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
  {
      Error_Handler();
      return -1;
  }
  //FRESULT fr;
  // debug_printf("mkfs\r\n");
  // fr = f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, buffer, sizeof(buffer));
  // if (fr != FR_OK)
  // {
  //    Error_Handler();
  // }
  scan_files(SDPath);
  debug_printf("fopen\r\n");
  if(f_open(&MyFile, "hello.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    Error_Handler();
    return -1;
  }
  /*##-5- Write data to the text file ################################*/
  debug_printf("hello\r\n");
  res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);

  if((byteswritten == 0) || (res != FR_OK))
  {
    /* 'STM32.TXT' file Write or EOF Error */
    Error_Handler();
    f_close(&MyFile);
    return 0;
  }

  debug_printf("fclose\r\n");
  f_close(&MyFile);

  debug_printf("fopen\r\n");
  if(f_open(&MyFile, "hello.txt", FA_READ) != FR_OK)
  {
    Error_Handler();
    return 0;
  }

  res = f_read(&MyFile, rtext, sizeof(rtext), (UINT*)&bytesread);

  if((bytesread == 0) || (res != FR_OK)) /* EOF or Error */
  {
    Error_Handler();
    f_close(&MyFile);
    return 0;
  }
  debug_printf("%s\r\n", rtext);
  f_close(&MyFile);

  if ((bytesread != byteswritten))
  {
    Error_Handler();
    return 0;
  }
  debug_printf("Success\r\n");

  /*##-11- Unlink the SD disk I/O driver ####################################*/
  FATFS_UnLinkDriver(SDPath);
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
  MX_I2S2_Init();
  //MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  SDIO_SD_Init();
  GPIO_Init();
  DEBUG_PRINT_Init();
  debug_printf("Hello STM32F401!!\r\n");
  sdio();
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  unsigned int count = 0;
  GPIO_PinState s, so;
  so = userSwRead();
  while (1)
  {
    s = userSwRead();
    if (s != so && s == GPIO_PIN_RESET) {
      ledToggle();
      debug_printf("Led Toggle %lu\r\n", count++);
    }
    so = s;
    HAL_Delay(50);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

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
  debug_printf("sdio Error \r\n");
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
