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
#include "cmsis_os.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
      uint8_t data[8];
      uint32_t StdId;
    }MotorSend;//对应标志位的要发送的数据结构体
    typedef struct{
      uint8_t data[2];
      uint32_t StdId;
      uint8_t motor_byte;
    }Motor;//对应每个电机的结构体

    MotorSend _1FE={{0x00},0x1FE},
    _1FF={{0x00},0x1FF},
    _200={{0x00},0x200};
    Motor 
    GM6020={{0x00,0x00},0x1FE,0},
    fric1={{0x00,0x00},0x200,0},
    fric2={{0x00,0x00},0x200,2},
    fric3={{0x00,0x00},0x200,4},
    firc4={{0x00,0x00},0x200,6},
    lift={{0x00,0x00},0x1FF,4},
    load={{0x00,0x00},0x1FF,2};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void CAN_SendMessage(uint8_t *data, uint32_t StdId);
void TransferToMotorSend(Motor *motor){//根据电机的StdId来决定发送到哪个MotorSend结构体中
  if(motor->StdId==0x1FE){
    _1FE.data[motor->motor_byte]=motor->data[0];
    _1FE.data[motor->motor_byte+1]=motor->data[1];
  }else if(motor->StdId==0x1FF){
    _1FF.data[motor->motor_byte]=motor->data[0];
    _1FF.data[motor->motor_byte+1]=motor->data[1];
  }else if(motor->StdId==0x200){
    _200.data[motor->motor_byte]=motor->data[0];
    _200.data[motor->motor_byte+1]=motor->data[1];
  }
}
void CanSendMotor(MotorSend *motorsend){//发送对应的MotorSend结构体
CAN_SendMessage(motorsend->data,motorsend->StdId);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
    
    
    uint8_t high,low;
    int16_t current_value = 0;
    volatile int counter=0;
    volatile int counter1=0;
    int A=0;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t ReID=0x205;
    uint8_t ReData[8];
    int ARR =999;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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
  MX_CAN2_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  // 调整 CAN 接收优先级高于 TIM2 发送
/*   HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0); 
  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);

  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn); */

  // 必须添加这句来启动 CAN2
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
     // 启动失败处理
     Error_Handler();
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
     // 启动失败处理
     Error_Handler();
  }
  HAL_TIM_Base_Start_IT(&htim2); 
    high=0x00;
    low=0x00;
  htim2.Instance->ARR = 999;
  // 配置 CAN 接收过滤器(不过滤，接收所有)
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 14; // CAN2 的过滤器组起始通常为 14
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // 开启接收中断
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //HAL_Delay(1);
    //HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    //HAL_Delay(10);
    
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void CAN_SendMessage(uint8_t *data, uint32_t StdId) {
  //CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;

  // 配置发送消息头
  TxHeader.StdId = StdId;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &TxMailbox) != HAL_OK) {
    //Error_Handler();
  }
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK) {
    //Error_Handler();
  }
}

// CAN 接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];

  if (hcan->Instance == CAN2)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
      counter++;
      // TODO: 在这里处理接收到的数据 RxData
      if(RxHeader.StdId==ReID){
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        for(int i=0;i<8;i++){
          ReData[i]=RxData[i];
        }
        

      }
      
    }
  }
}

void MotorUpdate(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    /* GM6020.data[0]= (current_value >> 8) & 0xFF;
    GM6020.data[1]= current_value & 0xFF;
    fric1.data[0]= (current_value >> 8) & 0xFF;
    fric1.data[1]= current_value & 0xFF;
    fric2.data[0]= (current_value >> 8) & 0xFF;
    fric2.data[1]= current_value & 0xFF;
    fric3.data[0]= (current_value >> 8) & 0xFF;
    fric3.data[1]= current_value & 0xFF;
    firc4.data[0]= (current_value >> 8) & 0xFF;
    firc4.data[1]= current_value & 0xFF;
    lift.data[0]= (current_value >> 8) & 0xFF;
    lift.data[1]= current_value & 0xFF; */
    load.data[0]= (current_value >> 8) & 0xFF;
    load.data[1]= current_value & 0xFF;
    TransferToMotorSend(&GM6020);
    TransferToMotorSend(&fric1);
    TransferToMotorSend(&fric2);
    TransferToMotorSend(&fric3);
    TransferToMotorSend(&firc4);
    TransferToMotorSend(&lift);
    TransferToMotorSend(&load);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

void StartTask2(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    static int a = 0;
    a++;
    osDelay(1000);
    current_value = +A;
    osDelay(1000);
    current_value = -A;
    
    __HAL_TIM_SET_AUTORELOAD(&htim2, ARR);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM2)
  {
    CanSendMotor(&_1FE);
    CanSendMotor(&_1FF);
    CanSendMotor(&_200);
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
