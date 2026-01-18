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
#include "math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
  double Kp;
  double Ki;
  double Kd;
  double error;
  double error_last;
  double integral;
  double output;
  double maxOutput;
  double deadband;
  double intergralLimit;
  double setpoint;
  double *outputAdress;
  double *inputAdress;
}Pid;//
typedef struct{
  uint8_t data[8];
  uint32_t StdId;
}MotorSend;//对应标志位的要发送的数据结构体
enum PidMode{disable,currentMode,angleMode,speedMode,torqueMode};
typedef struct{
  uint16_t singleAngle;
  double rpm;
  double torque;
  int8_t tempr;
  double angle;
  enum PidMode pidMode;
}MotorState;//存储电机状态的结构体
typedef struct{
  double output;
  uint32_t StdId;
  uint8_t motor_byte;
  uint8_t enabled; //使能
  uint8_t maxTemp; // 温度阈值
  double maxTorque; // 转矩阈值
  MotorState motorState;
  Pid anglePid,speedPid,torquePid;
}Motor;//对应每个电机的结构体
    #define MOTOR_SEND_NUM 3
    MotorSend _1FE={{0x00},0x1FE},
    _1FF={{0x00},0x1FF},
    _200={{0x00},0x200};
    MotorSend *motor_send_array[MOTOR_SEND_NUM]={&_1FE,&_1FF,&_200};
    Motor 
    GM6020={0,0x1FE,0},
    fric1={0,0x200,0},
    fric2={0,0x200,2},
    fric3={0,0x200,4},
    fric4={0,0x200,6},
    lift={0,0x1FF,4},
    load={0,0x1FF,2};
    #define MOTOR_NUM 7
    //对应所有电机的结构体指针数组,记得按照反馈ID顺序排列，第一个反馈标志位是0x201
    Motor *motor_array[MOTOR_NUM]={&fric1,&fric2,&fric3,&fric4,&GM6020,&load,&lift};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void PidInit(Pid *pid,double Kp,double Ki,double Kd,double maxOutput,double deadband,double intergralLimit){//初始化PID
  pid->Kp=Kp;
  pid->Ki=Ki;
  pid->Kd=Kd;
  pid->maxOutput=maxOutput;
  pid->deadband=deadband;
  pid->intergralLimit=intergralLimit;
}
void MotorInit(Motor *motor,uint32_t StdId,uint8_t motor_byte,uint8_t maxTemp,double maxTorque){//初始化电机结构体
  motor->StdId=StdId;
  motor->motor_byte=motor_byte;
  motor->motorState.pidMode=disable;
  motor->motorState.singleAngle=0;
  motor->motorState.angle=0;
  motor->motorState.rpm=0;
  motor->motorState.torque=0;
  motor->motorState.tempr=0;
  motor->enabled = 1;
  motor->maxTemp = maxTemp;
  motor->maxTorque = maxTorque;
  
  // 初始化PID指针为空，防止未配置模式时误计算
  motor->anglePid.inputAdress = NULL;
  motor->anglePid.outputAdress = NULL;
  motor->speedPid.inputAdress = NULL;
  motor->speedPid.outputAdress = NULL;
  motor->torquePid.inputAdress = NULL;
  motor->torquePid.outputAdress = NULL;
}
void CAN_SendMessage(uint8_t *data, uint32_t StdId);
void TransferToMotorSend(Motor *motor){//根据电机的StdId来决定发送到哪个MotorSend结构体中
  int16_t out_int = (int16_t)motor->output;
  if(motor->StdId==0x1FE){
    _1FE.data[motor->motor_byte]=(uint8_t)(out_int>>8);
    _1FE.data[motor->motor_byte+1]=(uint8_t)(out_int);
  }else if(motor->StdId==0x1FF){
    _1FF.data[motor->motor_byte]=(uint8_t)(out_int>>8);
    _1FF.data[motor->motor_byte+1]=(uint8_t)(out_int);
  }else if(motor->StdId==0x200){
    _200.data[motor->motor_byte]=(uint8_t)(out_int>>8);
    _200.data[motor->motor_byte+1]=(uint8_t)(out_int);
  }
}
void CanSendMotor(MotorSend *motorsend){//发送对应的MotorSend结构体
CAN_SendMessage(motorsend->data,motorsend->StdId);
}
void RecReceiveMotor(Motor *motor,uint8_t *data){//接收对应的电机数据
  motor->motorState.rpm=(double)((int16_t)data[2]<<8|data[3]);
  motor->motorState.torque=(double)((int16_t)data[4]<<8|data[5]);
  motor->motorState.tempr=(int8_t)data[6];
  int deltaAngle = (int)(((uint16_t)data[0]<<8)|data[1])-(int)motor->motorState.singleAngle;
  motor->motorState.singleAngle = ((uint16_t)data[0]<<8)|data[1];
  if(abs(deltaAngle) < 4096) motor->motorState.angle += deltaAngle;

}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
    
    
    uint8_t high,low;
    int16_t current_value = 0;
    int A=0;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t ReID=0x205;
    uint8_t ReData[8];
    int ARR =999;
    int CanSendCounter=0;
    
    // 双环控制的外部速度环PID实体
    Pid outerSpeedPid;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void PidCalculate(Pid *pid);
void MotorSetOutput(Motor *motor, enum PidMode mode, double value);
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
  MX_TIM4_Init();
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
      // TODO: 在这里处理接收到的数据 RxData
      RecReceiveMotor(motor_array[RxHeader.StdId - 0x201],RxData);
    }
  }
}

void PidCalculate(Pid *pid){
  if (pid->inputAdress == NULL) return;
  double measure = *pid->inputAdress;
  pid->error = pid->setpoint - measure;
  pid->integral += pid->error;
  if(pid->integral > pid->intergralLimit) pid->integral = pid->intergralLimit;
  if(pid->integral < -pid->intergralLimit) pid->integral = -pid->intergralLimit;
  
  pid->output = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * (pid->error - pid->error_last);
  pid->error_last = pid->error;
  
  if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
  if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
  
  if (pid->outputAdress != NULL) *pid->outputAdress = pid->output;
}

void MotorSetOutput(Motor *motor, enum PidMode mode, double value){
  motor->motorState.pidMode = mode;
  switch (mode)
  {
  case disable:
    motor->output = 0;
    break;
  case currentMode:
    motor->output = value;
    break;
  case angleMode:
    motor->anglePid.setpoint = value;
    motor->anglePid.inputAdress = &motor->motorState.angle;
    motor->anglePid.outputAdress = &motor->output;
    break;
  case speedMode:
    motor->speedPid.setpoint = value;
    motor->speedPid.inputAdress = &motor->motorState.rpm;
    motor->speedPid.outputAdress = &motor->output;
    break;
  case torqueMode:
    motor->torquePid.setpoint = value;
    motor->torquePid.inputAdress = &motor->motorState.torque;
    motor->torquePid.outputAdress = &motor->output;
    break;
  default:
    break;
  }
}

void MotorUpdate(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  static int buzzer_tick = 0;
  int alarm_level = 0; // 0:None, 1:Temp, 2:Torque

  // 确保开启 TIM4 CH3 的 PWM 输出
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  /* Infinite loop */
  for(;;)
  {
    alarm_level = 0; // 每个循环重置报警等级，重新检测

    for(int i=0;i<MOTOR_NUM;i++){
      Motor *m = motor_array[i];
      // PID calculation moved to StartPidTask
      
      // Safety Checks
      // 如果超过温度阈值或者扭矩阈值
      if (m->motorState.tempr > m->maxTemp || abs((int)m->motorState.torque) > m->maxTorque) {
          m->enabled = 0; // 立即禁用电机
          
          // 判定报警类型优先级：扭矩(2) > 温度(1)
          if (abs((int)m->motorState.torque) > m->maxTorque) {
             alarm_level = 2; 
          } else if (alarm_level < 2) {
             alarm_level = 1;
          }
      }

      if (m->enabled == 0) {
          m->output = 0;
      }

      TransferToMotorSend(m);
    }

    // --- 蜂鸣器非阻塞报警逻辑 (1ms task cycle) ---
    buzzer_tick++;
    if (buzzer_tick >= 1000) buzzer_tick = 0; // 1秒周期

    if (alarm_level == 0) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // 正常情况静音
        buzzer_tick = 0; // 复位保持同步
    } else if (alarm_level == 1) { // 温度报警：短响一次
        // [0-200ms] 响
        if (buzzer_tick < 200) 
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500);
        else 
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    } else if (alarm_level == 2) { // 扭矩报警：短响两次
        // [0-100ms] 响, [100-200ms] 停, [200-300ms] 响
        if ((buzzer_tick < 100) || (buzzer_tick >= 200 && buzzer_tick < 300))
             __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500);
        else 
             __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    }

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

void StartTask2(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  // 1. 初始化电机基本结构 (StdId=0x1FE, MotorByte=0)
  MotorInit(&GM6020, 0x1FE,0,50,10000);

  // 2. 初始化 GM6020 角度环 PID (内环) 参数: Kp=10.0, MaxOut=10000
  PidInit(&GM6020.anglePid, 10.0, 0.0, 0.0, 10000.0, 0.0, 0.0);
  
  // 3. 设置 GM6020 为角度模式，目标角度初始为 0 (稍后由外环控制)
  MotorSetOutput(&GM6020, angleMode, 0);

  // 4. 初始化外部速度环 PID (外环)
  // 参数: Kp=1.0, Ki=0.0, Kd=0.0 (需根据实际调优), MaxOut=8191(角度最大值), Deadband=0, I_Limit=0
  PidInit(&outerSpeedPid, 1.0, 0.0, 0.0, 8191.0, 0.0, 0.0);
  
  // 5. 绑定 PID 指针：外环输出 -> 内环输入
  outerSpeedPid.inputAdress = &GM6020.motorState.rpm;       // 输入：当前 RPM
  outerSpeedPid.outputAdress = &GM6020.anglePid.setpoint;   // 输出：角度环的目标值(Setpoint)
  
  // 6. 设定外环目标值
  outerSpeedPid.setpoint = 100.0; // 例如：目标 100 RPM

  /* Infinite loop */
  for(;;)
  {//主程序在此处编写
    // 可以在这里改变 outerSpeedPid.setpoint 来动态调整速度
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}
void StartPidTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {//在此处计算PID参数
    // 计算自定义的 PID (需在电机PID之前计算，以便将输出作为电机PID的输入)
    PidCalculate(&outerSpeedPid);

    for(int i=0; i<MOTOR_NUM; i++){
       Motor *m = motor_array[i];
       switch(m->motorState.pidMode){
         case angleMode:
           PidCalculate(&m->anglePid);
           break;
         case speedMode:
           PidCalculate(&m->speedPid);
           break;
         case torqueMode:
           PidCalculate(&m->torquePid);
           break;
         default:
           break;
       }
    }
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
  {//由TIM2中断触发循环发送MotorSend存储池中存储的发送数据
    CanSendCounter++;
    for(int i=0;i<MOTOR_SEND_NUM;i++){
      CanSendMotor(motor_send_array[i]);
    }
/*     CanSendMotor(&_1FE);
    CanSendMotor(&_1FF);
    CanSendMotor(&_200); */
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
