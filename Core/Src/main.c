/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_custom_hid_if.h"
#include "keycode.h"
#include "keymap.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct keyboardHIDBitmap_t {
    uint8_t report_id;
    uint8_t modifiers;
    uint8_t key_bit[REPORT_KEYBIT_BYTES];
};
struct keyboardHID_t {
  uint8_t report_id;
  uint8_t modifiers;
  uint8_t reserved;
  uint8_t key[6];
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
struct keyboardHIDBitmap_t keyboardHID;
uint8_t currentLayer = 0;
uint8_t bufferIndex = 0;
struct keyboardHIDBitmap_t keyboardHidBuffer[DEBOUNCE];
uint8_t keyPush[ROW_NUM][COL_NUM];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */
static void KeyPressingRaw(uint8_t col, uint8_t row);
static void KeyReleasingRaw(uint8_t col, uint8_t row);
static void KeyPressing(uint8_t code);
static void KeyReleasing(uint8_t code);
static inline void FnProc();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static volatile uint8_t timer_intr = 0;
static volatile uint8_t scan_intr = 0;
static volatile uint8_t fnKey = 0;
void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef* htim )
{
  // 50us interval
  if (htim->Instance == TIM13) {
    scan_intr = 1;
  }
  // 1ms interval
  if (htim->Instance == TIM14) {
    timer_intr = 1;
  }
}
static void ResetBitmapReport(struct keyboardHIDBitmap_t* report){
  report->modifiers = 0;
  for (int8_t i = 0; i < REPORT_KEYBIT_BYTES; i++) {
    report->key_bit[i] = 0;
  }
}
static void AddKeyBitmap(struct keyboardHIDBitmap_t* report, uint8_t code){
  if((code>>3)<REPORT_KEYBIT_BYTES)
    report->key_bit[code>>3] |= (1<<(code&0x07));
}
static void RemoveKeyBitmap(struct keyboardHIDBitmap_t* report, uint8_t code){
  if((code>>3)<REPORT_KEYBIT_BYTES)
    report->key_bit[code>>3] &= ~(1<<(code&0x07));
}
static void MatrixScan(){
  fnKey = 0;
  ResetBitmapReport(&keyboardHID);
//  while(scan_intr==0);
//  scan_intr=0;
  for(uint8_t col=0; col<COL_NUM; col++){
    while(scan_intr==0);
    scan_intr=0;
    for(uint8_t t=0; t<COL_NUM; t++){
      if(t==col){
        HAL_GPIO_WritePin(matrixColPort[t], matrixColPin[t], GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(matrixColPort[t], matrixColPin[t], GPIO_PIN_RESET);
      }
    }
    while(scan_intr==0);
    scan_intr=0;
    for(uint8_t row=0; row<ROW_NUM; row++){
      if(HAL_GPIO_ReadPin(matrixRowPort[row], matrixRowPin[row]) == GPIO_PIN_SET){
        KeyPressingRaw(col,row);
      } else {
        keyPush[row][col]=0;
      }
    }
  }
  FnProc();
}
static inline void KeyPressingRaw(uint8_t col, uint8_t row){
  if(keyPush[row][col]==0){
    keyPush[row][col]=currentLayer+1;
  }
  uint8_t code = keymap[keyPush[row][col]-1][row][col];
  if(code == KC_TRNS)
    code = keymap[keyPush[row][col]-2][row][col];
  KeyPressing(code);
}
static inline void KeyPressing(uint8_t code){
  switch (code) {
    case KC_LCTRL:
    case KC_LSHIFT:
    case KC_LALT:
    case KC_LGUI:
    case KC_RCTRL:
    case KC_RSHIFT:
    case KC_RALT:
    case KC_RGUI:
      keyboardHID.modifiers |= (1<<(code&0x07));
      break;
    case KC_FN1:
      fnKey = 1;
      break;
    default:
      AddKeyBitmap(&keyboardHID,code);
  }
}
static inline void FnProc(){
  if(fnKey){
    currentLayer = 1;
  } else {
    currentLayer = 0;
  }
}

static inline void AddBuffer(struct keyboardHIDBitmap_t bitmap){
  bufferIndex++;
  bufferIndex%=DEBOUNCE;
  keyboardHidBuffer[bufferIndex] = bitmap;
}

static inline struct keyboardHIDBitmap_t SumBuffer(){
  struct keyboardHIDBitmap_t t;
  ResetBitmapReport(&t);
  for(uint8_t i=0; i<DEBOUNCE; i++){
    t.modifiers |= keyboardHidBuffer[i].modifiers;
    for(uint8_t k=0; k<REPORT_KEYBIT_BYTES; k++){
      t.key_bit[k] |= keyboardHidBuffer[i].key_bit[k];
    }
  }
  return t;
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
  struct keyboardHID_t keyboardHIDNormal;
  keyboardHID.report_id = 2;
  ResetBitmapReport(&keyboardHID);

  for (int8_t i = 0; i < 6; ++i) {
    keyboardHIDNormal.key[i] = 0;
  }
  keyboardHIDNormal.report_id = 1;
  keyboardHIDNormal.modifiers =0 ;
  keyboardHIDNormal.reserved = 0;

  for(uint8_t i=0; i<ROW_NUM; i++){
    for(uint8_t j=0; j<COL_NUM; j++){
      keyPush[i][j]=0;
    }
  }

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART5_Init();
  MX_USB_DEVICE_Init();
  MX_TIM14_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT( &htim13 );
  HAL_TIM_Base_Start_IT( &htim14 );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  #pragma clang diagnostic push
  #pragma ide diagnostic ignored "EndlessLoop"
  //USBD_CUSTOM_HID_SendReport( &hUsbDeviceFS, (uint8_t*) &keyboardHID, sizeof( struct keyboardHIDBitmap_t ) );
  int cnt = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    while (timer_intr == 0);
    timer_intr = 0;
    MatrixScan();
    AddBuffer(keyboardHID);
    struct keyboardHIDBitmap_t sendReport = SumBuffer();
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *) &sendReport, sizeof(struct keyboardHIDBitmap_t));
//    cnt++;
//    if(cnt==1000) {
//      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *) &keyboardHID, sizeof(struct keyboardHIDBitmap_t));
//      //USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *) &keyboardHIDNormal, sizeof(struct keyboardHID_t));
//    }
//    if(cnt==2000){
//      for (int8_t i = 0; i < REPORT_KEYBIT_BYTES; ++i) {
//        keyboardHID.key_bit[i] = 0;
//      }
//
//      for (int8_t i = 0; i < 6; ++i) {
//        keyboardHIDNormal.key[i] = 0;
//      }
//      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *) &keyboardHID, sizeof(struct keyboardHIDBitmap_t));
//      //USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *) &keyboardHIDNormal, sizeof(struct keyboardHID_t));
//      keyboardHIDNormal.key[0] = 0x04;
//      keyboardHID.key[0]  = 0xf0;
//      keyboardHID.key[1]  = 0xff;
//      keyboardHID.key[2]  = 0xff;
//      keyboardHID.key[3]  = 0xff;
//      keyboardHID.key[4]  = 0xff;        //   -27
//      keyboardHID.key[5]  = 0b11100001;  // 28-2F
//      keyboardHID.key[6]  = 0xff;        // 30-37
//      keyboardHID.key[7]  = 0b00000001;  // 38-3F
//      keyboardHID.key[8]  = 0b00000000;  // 40-47
//      keyboardHID.key[9]  = 0b10000000;  // 48-4F
//      keyboardHID.key[10] = 0b11110111;  // 50-57
//      keyboardHID.key[11] = 0b11111111;  // 58-5F
//      keyboardHID.key[12] = 0b11111111;  // 60-67
//      cnt=0;
//    }
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
  RCC_OscInitStruct.PLL.PLLM = 16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 47;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 49;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, COL4_Pin|COL5_Pin|COL6_Pin|COL9_Pin
                          |COL8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, COL1_Pin|COL2_Pin|COL3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(COL7_GPIO_Port, COL7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ROW5_Pin */
  GPIO_InitStruct.Pin = ROW5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ROW5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COL4_Pin COL5_Pin COL6_Pin COL9_Pin
                           COL8_Pin */
  GPIO_InitStruct.Pin = COL4_Pin|COL5_Pin|COL6_Pin|COL9_Pin
                          |COL8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : COL1_Pin COL2_Pin COL3_Pin */
  GPIO_InitStruct.Pin = COL1_Pin|COL2_Pin|COL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW1_Pin ROW2_Pin ROW4_Pin ROW3_Pin */
  GPIO_InitStruct.Pin = ROW1_Pin|ROW2_Pin|ROW4_Pin|ROW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : COL7_Pin */
  GPIO_InitStruct.Pin = COL7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(COL7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW6_Pin ROW7_Pin ROW8_Pin */
  GPIO_InitStruct.Pin = ROW6_Pin|ROW7_Pin|ROW8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ROW9_Pin */
  GPIO_InitStruct.Pin = ROW9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ROW9_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
