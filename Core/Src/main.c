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
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
#include "usbd_cdc_if.h"
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
  uint8_t isStalled; // 是否堵转
  uint32_t stallTimer; // 堵转计时
}MotorState;//存储电机状态的结构体
typedef struct{
  double output;
  uint32_t StdId;
  uint8_t motor_byte;
  uint8_t enabled; //使能
  int8_t maxTemp; // 温度阈值
  double maxTorque; // 转矩阈值
  double stallOutput; // 堵转输出阈值 (Output)
  double stallSpeedThreshold; // 堵转速度阈值 (RPM)
  uint32_t stallTimeThreshold; // 堵转时间阈值(ms)
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
void MotorInit(Motor *motor,uint32_t StdId,uint8_t motor_byte)
  //,uint8_t maxTemp,double maxTorque, double stallOutput, double stallSpeedThreshold, uint32_t stallTimeThreshold)
{//初始化电机结构体
  motor->StdId=StdId;
  motor->motor_byte=motor_byte;
  motor->motorState.pidMode=disable;
  motor->motorState.singleAngle=0;
  motor->motorState.angle=0;
  motor->motorState.rpm=0;
  motor->motorState.torque=0;
  motor->motorState.tempr=0;
  motor->enabled = 1;
  motor->maxTemp = 35;
  motor->maxTorque = 5000;
  motor->stallOutput = 1000;
  motor->stallSpeedThreshold = 50;
  motor->stallTimeThreshold = 500;
  motor->motorState.isStalled = 0;
  motor->motorState.stallTimer = 0;
  
  // 初始化PID指针为空，防止未配置模式时误计算
  motor->anglePid.inputAdress = NULL;
  motor->anglePid.outputAdress = NULL;
  motor->speedPid.inputAdress = NULL;
  motor->speedPid.outputAdress = NULL;
  motor->torquePid.inputAdress = NULL;
  motor->torquePid.outputAdress = NULL;
}
void MotorSafetyInit(Motor *motor, uint8_t maxTemp, double maxTorque, double stallOutput, double stallSpeedThreshold, uint32_t stallTimeThreshold){
  motor->maxTemp = maxTemp;
  motor->maxTorque = maxTorque;
  motor->stallOutput = stallOutput;
  motor->stallSpeedThreshold = stallSpeedThreshold;
  motor->stallTimeThreshold = stallTimeThreshold;
}
void MotorRunToStall(Motor *motor, double speed){//以指定速度运行电机直到堵转
  //enum PidMode lastMode = motor->motorState.pidMode;
  motor->motorState.pidMode = speedMode;
  motor->speedPid.setpoint = speed;
  while(motor->motorState.isStalled==0){
    osDelay(1);
  }
  motor->motorState.isStalled=0;
  motor->motorState.stallTimer=0;
  motor->motorState.pidMode = disable;
  motor->speedPid.setpoint = 0;
}
void MotorRunToAngle(Motor *motor, double angle, double speed){//以指定角度运行电机
  if(motor->motorState.angle < angle){
    motor->motorState.pidMode = speedMode;
    motor->speedPid.setpoint = fabs(speed);
    while(motor->motorState.angle < angle){
      osDelay(1);
    }
  }else{
    motor->motorState.pidMode = speedMode;
    motor->speedPid.setpoint = -fabs(speed);
    while(motor->motorState.angle > angle){
      osDelay(1);
    }
  }
  motor->motorState.pidMode = angleMode;
  motor->anglePid.setpoint = angle;
  motor->speedPid.setpoint = 0;
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
  motor->motorState.rpm=(double)(int16_t)((data[2]<<8)|data[3]);
  motor->motorState.torque=(double)(int16_t)((data[4]<<8)|data[5]);
  motor->motorState.tempr=(int8_t)data[6];
  int deltaAngle = (int)(((uint16_t)data[0]<<8)|data[1])-(int)motor->motorState.singleAngle;
  motor->motorState.singleAngle = ((uint16_t)data[0]<<8)|data[1];
  if(abs(deltaAngle) < 4096) motor->motorState.angle += deltaAngle;

}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
    
    int alarm_level = 0; // 0:None, 1:Temp, 2:Torque, 3:EXTI//用于报警
    short int alarm_motor = -1; // 报警电机编号
    uint16_t alarm_counter = 0; // 报警计数器
    uint8_t CDC_Ctrl_state = 0; // CDC使能标志





    uint8_t high,low;
    int16_t current_value = 0;
    int A=0;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t ReID=0x205;
    uint8_t ReData[8];
    int ARR =999;
    int CanSendCounter=0;
    
    // 双环控制的外部速度环PID实体
    //Pid outerSpeedPid;

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
  MX_TIM3_Init();
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == KEY_Pin){
    alarm_level = 3; // 设置最高优先级报警（按键触发）
    for(int i=0;i<MOTOR_NUM;i++){
      motor_array[i]->enabled=0;
    }
  }
}

void CDC_Receive_Callback(uint8_t *Buf, uint32_t Len)
{
    // 解析自定义数据包
    // Byte 0: 0x00 (保留/帧头)
    // Byte 1: Motor ID (0-6)
    // Byte 2: Mode (0:Disable, 1:Current, 2:Angle, 3:Speed, 4:Torque)
    // Byte 3-4: Value (int16, Big Endian)
    if(CDC_Ctrl_state != 1) return; // 未连接时不处理
    if (Len >= 5 && Buf[0] == 0x00) {
        uint8_t motor_id = Buf[1];
        uint8_t mode_val = Buf[2];
        
        if (motor_id < MOTOR_NUM) {
            // 组合 16 位补码数值 (模拟大端序: High << 8 | Low)
            int16_t val_int16 = (int16_t)((Buf[3] << 8) | Buf[4]);
            double val_double = (double)val_int16;

            // 检查模式是否有效 (0 ~ 4)
            if (mode_val <= 4) {
                 // enum PidMode 定义: disable=0, currentMode=1, angleMode=2, speedMode=3, torqueMode=4
                 // 若为 disable 模式，val_double 不影响结果，内部会设为 0
                 MotorSetOutput(motor_array[motor_id], (enum PidMode)mode_val, val_double);
            }
        }
    }

    // 将接收到的数据回显（Echo）
    // 注意：CDC_Transmit_FS 如果正在忙碌可能会失败，实际应用可以使用缓冲区或重试机制
    CDC_Transmit_FS(Buf, Len);
}

void PidCalculate(Pid *pid){
  if (pid->inputAdress == NULL) return;
  double measure = *pid->inputAdress;
  pid->error = pid->setpoint - measure;
  if(fabs(pid->error)>pid->deadband)pid->integral += pid->error;
  
  if (pid->Ki != 0) {
    if(pid->integral*pid->Ki > pid->intergralLimit) pid->integral = pid->intergralLimit/pid->Ki;
    else if(pid->integral*pid->Ki < -pid->intergralLimit) pid->integral = -pid->intergralLimit/pid->Ki;
  }
  
  if(fabs(pid->error)>pid->deadband)pid->output = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * (pid->error - pid->error_last);
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
  static int buzzer_oneshot = 0; // 短响计时器
  
  alarm_level = 0;
  osDelay(1000);
  // 确保开启 TIM4 CH3 的 PWM 输出
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  /* Infinite loop */
  for(;;)
  {
    // alarm_level 不在每次循环重置，保持报警状态直到系统复位或手动清除

    for(int i=0;i<MOTOR_NUM;i++){
      Motor *m = motor_array[i];
      // PID calculation moved to StartPidTask
      
      // Safety Checks
      // 1. 堵转检测
      // 使用最终输出电流值(m->output)和转速(m->motorState.rpm)进行判断
      uint8_t prev_stalled = m->motorState.isStalled;
      if (fabs(m->output) > m->stallOutput && fabs(m->motorState.rpm) < m->stallSpeedThreshold) {
          m->motorState.stallTimer++;
      } else {
          m->motorState.stallTimer = 0;
          m->motorState.isStalled = 0;
      }
      
      if (m->motorState.stallTimer > m->stallTimeThreshold) {
          m->motorState.isStalled = 1;
      }

      // 检测到堵转状态上升沿 (0 -> 1)，触发一次短响
      if (prev_stalled == 0 && m->motorState.isStalled == 1) {
          buzzer_oneshot = 200; // 触发200ms短响
          
          if(i < 7) {
            // 现场计算频率 (Base: C3=130.81Hz)
            // C Major semitones: 0(C), 2(D), 4(E), 5(F), 7(G), 9(A), 11(B)
            int semitones_map[] = {0, 2, 4, 5, 7, 9, 11};
            float freq = 261.63f * pow(2.0f, semitones_map[i] / 12.0f);
            
            // ARR = (TimerClock / Frequency) - 1. TimerClock = 1MHz
            uint32_t reload = (uint32_t)(1000000.0f / freq) - 1;
            __HAL_TIM_SET_AUTORELOAD(&htim4, reload);
            // 同时更新占空比为50% (CCR = ARR/2)
             __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, reload / 2);
          }
      }

      // 2. 阈值保护 (温度 > 阈值 OR (转矩 > 阈值(瞬间)))
      // 注意：堵转(isStalled)不再触发禁用或报警
      if (m->motorState.tempr > m->maxTemp){
        m->enabled = 0; // 立即禁用电机
        if (alarm_level < 1) alarm_level = 1; // 仅升级报警等级，不降级
        alarm_motor = i+1;
        alarm_counter++;
      }
      
      if (fabs(m->motorState.torque) > m->maxTorque) {
        m->enabled = 0; // 立即禁用电机
        if (alarm_level < 2) alarm_level = 2; // 仅升级报警等级，不降级（扭矩优于温度）
        alarm_motor = i+1;
        alarm_counter++;
      }
      

      if (m->enabled == 0) {
          m->output = 0;
      }

      TransferToMotorSend(m);
    }

    // --- 蜂鸣器非阻塞报警逻辑 (1ms task cycle) ---
    // 兼容 1-10 次短响逻辑: 每次短响占用 200ms (100ms ON, 100ms OFF)
    // 最大 10 次需要 2000ms 周期。为了简化，如果 alarm_level > 0，则按 alarm_level * 200ms + 500ms(间隔) 计算周期
    // buzzer_tick 单位: ms
    
    buzzer_tick++;
    int cycle_period = 1000;
    if (alarm_level > 0) {
        cycle_period = alarm_level * 200 + 800; // 动态周期，保证响完后有一段静音
    }
    
    if (buzzer_tick >= cycle_period) buzzer_tick = 0; 
    
    if (buzzer_oneshot > 0) {
        // 短响期间保持当前频率输出
        buzzer_oneshot--;
        if (buzzer_oneshot == 0) {
            // 结束短响，恢复默认 1KHz (ARR = 999)
            __HAL_TIM_SET_AUTORELOAD(&htim4, 999);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // 暂歇
        }
    }
    else if (alarm_level == 0) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // 正常情况静音
        buzzer_tick = 0; // 复位保持同步
    } else {
        // 通用报警逻辑：响 alarm_level 次
        // 例如 alarm_level = 3: 
        // 0-100: ON, 100-200: OFF
        // 200-300: ON, 300-400: OFF
        // 400-500: ON, 500-600: OFF
        // >600: OFF (直到 cycle_period)
        
        // 判断当前时刻是否在所有响声的时间段内
        if (buzzer_tick < (alarm_level * 200)) {
            // 在响声时间段内，判断是 ON 还是 OFF
            if ((buzzer_tick % 200) < 100) {
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500); // ON
            } else {
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);   // OFF
            }
        } else {
            // 超过响声时间段，保持静音
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        }
    }

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

void StartTask2(void const * argument)
{
  ///////////////////////////////////////////////////
  //    lift全程angle:9610000 向上为正              //
  //    GM6020全程angle:490000 顺时针为正           //
  ///////////////////////////////////////////////////
  /* USER CODE BEGIN StartDefaultTask */
  // 1. 初始化电机基本结构 MotorInit(*motor, StdId, MotorByte);
  //MotorSafetyInit(*motor, maxTemp, maxTorque, stallOutput, stallSpeedThreshold, stallTimeThreshold);  
  MotorInit(&fric1, 0x200, 0);
  MotorSafetyInit(&fric1, 35, 5000, 500, 50, 500);
  MotorInit(&fric2, 0x200, 2);
  MotorSafetyInit(&fric2, 35, 5000, 500, 50, 500);
  MotorInit(&fric3, 0x200, 4);
  MotorSafetyInit(&fric3, 35, 5000, 500, 50, 500);
  MotorInit(&fric4, 0x200, 6);
  MotorSafetyInit(&fric4, 35, 5000, 500, 50, 500);
  MotorInit(&lift, 0x1FF, 4);
  MotorSafetyInit(&lift, 35, 5000, 500, 50, 500);
  //lift.enabled=0; // 升降电机初始禁用
  MotorInit(&load, 0x1FF, 2);
  MotorSafetyInit(&load, 35, 5000, 500, 50, 500);
  load.enabled=0; // 装弹电机初始禁用
  MotorInit(&GM6020, 0x1FE, 0);
  MotorSafetyInit(&GM6020, 35, 5000, 1000, 50, 500);

  // 2. 初始化 GM6020 角度环 PID (内环) 参数: Kp,Ki,Kd,MaxOut,Deadband,I_Limit
  PidInit(&GM6020.anglePid, 1.0, 0.0, 0.0, 4000.0, 0.0, 0.0);
  PidInit(&GM6020.speedPid, 12, 1, 0.0, 4000.0, 0.0, 1000);
  PidInit(&fric1.anglePid, 1, 1, 1000, 3000.0, 0.0, 1000);
  PidInit(&fric1.speedPid, 1, 0.01, 1, 3000.0, 0.0, 300);
  PidInit(&fric2.anglePid, 1, 1, 1000, 3000.0, 0.0, 1000);
  PidInit(&fric2.speedPid, 1, 0.01, 1, 3000.0, 0.0, 300);
  PidInit(&fric3.anglePid, 1, 1, 1000, 3000.0, 0.0, 1000);
  PidInit(&fric3.speedPid, 1, 0.01, 1, 3000.0, 0.0, 300);
  PidInit(&fric4.anglePid, 1, 1, 1000, 3000.0, 0.0, 1000);
  PidInit(&fric4.speedPid, 1, 0.01, 1, 3000.0, 0.0, 300);
  PidInit(&lift.anglePid, 1, 1, 1000, 3000.0, 0.0, 1000);
  PidInit(&lift.speedPid, 1, 0.01, 1, 3000.0, 0.0, 300);
  PidInit(&load.anglePid, 1, 1, 1000, 3000.0, 0.0, 1000);
  PidInit(&load.speedPid, 1, 0.01, 1, 3000.0, 0.0, 300);
  
  // 3. 设置初始输出为 0
  MotorSetOutput(&GM6020, speedMode, 0);
  MotorSetOutput(&fric1, angleMode, 0);
  MotorSetOutput(&fric2, speedMode, 0);
  MotorSetOutput(&fric3, angleMode, 0);
  MotorSetOutput(&fric4, angleMode, 0);
  MotorSetOutput(&lift, speedMode, 0);
  MotorSetOutput(&load, angleMode, 0);



  // 4. 初始化外部速度环 PID (外环)
  // 参数: Kp=1.0, Ki=0.0, Kd=0.0 (需根据实际调优), MaxOut=8191(角度最大值), Deadband=0, I_Limit=0
  //PidInit(&outerSpeedPid, 1.0, 0.0, 0.0, 8191.0, 0.0, 0.0);
  

  // 5. 绑定 PID 指针：外环输出 -> 内环输入
  //outerSpeedPid.inputAdress = &GM6020.motorState.rpm;       // 输入：当前 RPM
  //outerSpeedPid.outputAdress = &GM6020.anglePid.setpoint;   // 输出：角度环的目标值(Setpoint)
  
  // 6. 设定外环目标值
  //outerSpeedPid.setpoint = 100.0; // 例如：目标 100 RPM


  /////////////////////////以下为主程序////////////////////////////
  //上电初始化，GM6020和lift走到负方向限位
  MotorRunToStall(&GM6020,-300);
  GM6020.motorState.angle=0;
  MotorRunToAngle(&GM6020,245000,300);
  //MotorRunToStall(&lift,6000);
  MotorRunToStall(&lift,-6000);
  lift.motorState.angle=0;
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET); // 指示初始化完成
  osDelay(1000);
  MotorSetOutput(&fric1, speedMode, -4000);
  MotorSetOutput(&fric2, speedMode, -4000);
  MotorSetOutput(&fric3, speedMode, 4000);
  MotorSetOutput(&fric4, speedMode, 4000);
  osDelay(3000);
  MotorSetOutput(&fric1, speedMode, 0);
  MotorSetOutput(&fric2, speedMode, 0);
  MotorSetOutput(&fric3, speedMode, 0);
  MotorSetOutput(&fric4, speedMode, 0);
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET); // 指示准备完成
  CDC_Ctrl_state = 1; // CDC 连接完成，允许接收控制命令
  /* Infinite loop */
  for(;;)
  {//主程序在此处编写
    // 可以在这里改变 outerSpeedPid.setpoint 来动态调整速度
    //alarm_level = 2; // 测试报警
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
    //PidCalculate(&outerSpeedPid);

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
