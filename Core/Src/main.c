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
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "Filter.h"
#include "referee.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{//PID结构体
  double Kp;  
  double Ki;
  double Kd;
  double error;
  double error_last;
  double integral;
  double output;
  double maxOutput;
  double deadband;//误差死区，当近似达到目标就不再计算输出，防止持续微调
  double intergralLimit;
  double setpoint;
  double *outputAdress;
  double *inputAdress;
}Pid;//

typedef struct{
  uint8_t data[8];
  uint32_t StdId;
}MotorSend;//对应标志位的要发送的数据结构体

enum MotorMode{disable,currentMode,angleMode,speedMode,torqueMode,runToAngle,runToStallMode,speedTimeMode};

typedef struct{
  uint16_t singleAngle;
  double rpmRaw; 
  double torqueRaw;
  double rpm;
  double torque;
  int8_t tempr;
  double angle;
  filter_t rpmFilter;//转速滤波器
  filter_t torqueFilter;//转矩滤波器
  enum MotorMode motorMode;
  uint8_t isStalled; // 是否堵转
  uint32_t stallTimer; // 堵转计时
  uint32_t lastRxTick; // 最近一次收到CAN反馈的时间戳
  uint8_t isOnline; // CAN反馈在线状态
  uint32_t onlineSinceTick; // 最近一次上线时间戳，用于上电后保护判定延时
  uint8_t newDataFlag; // 新数据标志位
}MotorState;//存储电机状态的结构体

typedef struct{
  double output;
  uint32_t StdId;
  uint8_t motor_byte;//电机标志位
  uint8_t enabled; //使能
  int8_t maxTemp; // 温度阈值
  double maxTorque; // 转矩阈值
  double stallOutput; // 堵转输出阈值 (Output)
  double stallSpeedThreshold; // 堵转速度阈值 (RPM)
  uint32_t stallTimeThreshold; // 堵转时间阈值(ms)
  double targetAngle; // For runToAngle
  double runSpeed;    // For runToAngle
  uint32_t runTime;   // For speedTimeMode
  uint32_t runTimer;  // For speedTimeMode
  MotorState motorState;
  Pid anglePid,speedPid,torquePid;
}Motor;//对应每个电机的结构体

typedef struct{
  double yaw;
  double v1Speed;
  double v2Speed;
}dartParam;//每发飞镖的发射参数结构体

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
  #define MOTOR_SEND_NUM 3
  #define MOTOR_NUM 7
  #define RPM_KF_Q 0.7f//卡尔曼过程噪声，越大模型越不可信任，越依赖测量值
  #define RPM_KF_R 10.0f//卡尔曼测量噪声，越大测量值越不可信任，越依赖模型预测值
  #define TORQUE_FILTER_ALPHA 0.30f//转矩滤波器系数，调整转矩滤波，0到1之间，0表示不滤波，1表示完全滤波
  #define BUZZER_ALERT_FREQ_HZ 1450U // 日常报警蜂鸣器频率，避免与电调告警音相近
  #define MOTOR_OFFLINE_TIMEOUT_MS 100U // 超过该时间未收到反馈则判定离线
  #define MOTOR_ONLINE_WARMUP_MS 800U // 电机重新上电后的保护判定预热时间

  // 全自动模式（RunningTask=6）预设发射参数（4发）
  #define DART_AUTO_YAW_0   -72000
  #define DART_AUTO_V1_0    7500
  #define DART_AUTO_V2_0    4450
  #define DART_AUTO_YAW_1   -70100
  #define DART_AUTO_V1_1    7200
  #define DART_AUTO_V2_1    4450
  #define DART_AUTO_YAW_2   -68000
  #define DART_AUTO_V1_2    7050
  #define DART_AUTO_V2_2    4450
  #define DART_AUTO_YAW_3   -68000
  #define DART_AUTO_V1_3    7050
  #define DART_AUTO_V2_3    4450

  // 半自动模式（RunningTask=7）第一发预设参数
  #define DART_SEMI_FIRST_YAW   -68000
  #define DART_SEMI_FIRST_V1    7500
  #define DART_SEMI_FIRST_V2    4450

  // 比赛模式选择：0=关闭（默认手动CDC控制），6=全自动，7=半自动
  #define DART_COMPETITION_MODE 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void PidInit(Pid *pid,double Kp,double Ki,double Kd,double maxOutput,double deadband,double intergralLimit);//初始化PID
void MotorInit(Motor *motor,uint32_t StdId,uint8_t motor_byte);
//,uint8_t maxTemp,double maxTorque, double stallOutput, double stallSpeedThreshold, uint32_t stallTimeThreshold)
//初始化电机结构体
void MotorSafetyInit(Motor *motor, uint8_t maxTemp, double maxTorque, double stallOutput, double stallSpeedThreshold, uint32_t stallTimeThreshold);
void MotorFilterUpdate(Motor *motor);//在任务上下文中更新电机滤波结果
//初始化电机安全保护参数
void MotorRunToStall(Motor *motor, double speed);//以指定速度运行电机直到堵转
void MotorSetOutput(Motor *motor, enum MotorMode mode, double value);//设置输出
void MotorRunToAngleBlocking(Motor *motor, double angle, double speed);//以指定角度运行电机
void MotorRunSpeedTime(Motor *motor, double speed, uint32_t time);//以指定速度运行指定时间
void MotorRunSpeedTimeBlocking(Motor *motor, double speed, uint32_t time);//以指定速度运行指定时间(阻塞)
void MotorRunToSetTemp(Motor *motor, double targetTemp, double warmSpeed, double holdSpeed);//摩擦轮预热控温
void CAN_SendMessage(uint8_t *data, uint32_t StdId);//发送CAN消息
void TransferToMotorSend(Motor *motor);//根据电机的StdId来决定发送到哪个MotorSend结构体中
void CanSendMotor(MotorSend *motorsend);//发送对应的MotorSend结构体
void MotorCdcFeedback(uint8_t motor_SN);//发送motor_array对应序号电机的反馈信息到CDC
void RefereeCdcFeedback(void);//发送裁判系统缓存数据到CDC
void DartFireSingle(double yaw, double v1Speed, double v2Speed);//单发飞镖（阻塞）

void Singing();//整活
void SingingSome();//唱一小段
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  referee_t g_referee;//裁判系统对象
  ext_dart_info_t g_dart_info_cache;//缓存的飞镖信息
  ext_dart_client_cmd_t g_dart_cmd_cache;//飞镖操作系统命令缓存

    int alarm_level = 0; // 0:None, 1:Temp, 2:Torque, 3:EXTI//用于报警
    short int alarm_motor = -1; // 报警电机编号
    uint16_t alarm_counter = 0; // 报警计数器
    uint8_t CDC_Ctrl_state = 0; // CDC使能标志
    uint8_t RunningTask = 0;// 运行任务标志
    uint8_t MotorUpdateFlag = 1; // 电机更新标志
    uint8_t DartFireFlag = 0; // 飞镖发射标志,1表示已经发射过前两发弹丸，2表示已经发射过全部四发弹丸


    dartParam dartParam_array[4];//飞镖参数数组

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
    //对应所有电机的结构体指针数组,记得按照反馈ID顺序排列，第一个反馈标志位是0x201
    Motor *motor_array[MOTOR_NUM]={&fric1,&fric2,&fric3,&fric4,&GM6020,&load,&lift};

    //uint8_t high=0x00,low=0x00;//发送高低字节
    //int16_t current_value = 0;//发送值
    //int A=0;//振幅
    //uint32_t ReID=0x205;
    //uint8_t ReData[8];
    //int Arr =999;
    //以上是历史残留代码
    CAN_TxHeaderTypeDef TxHeader;
    int CanSendCounter=0;
    
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void PidCalculate(Pid *pid);

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
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
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
    // CAN1 当前未挂载设备，不启动
  HAL_TIM_Base_Start_IT(&htim2); 

  htim2.Instance->ARR = 999;//设置自动重装载值，决定发送频率，999对应100Hz
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

  HAL_Delay(3000);
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
  
  if (Referee_Init(&g_referee, NULL) != Referee_Ok)//初始化裁判机
  {
    Error_Handler();
  }
  if (Referee_StartUartReceive(&g_referee) != Referee_Ok)//启动UART1异步接收
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

//CAN发送函数
void CAN_SendMessage(uint8_t *data, uint32_t StdId) {
  //CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;

  // 配置发送消息头
  TxHeader.StdId = StdId;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;//数据长度，8字节
  TxHeader.TransmitGlobalTime = DISABLE;//不发送全局时间戳

  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &TxMailbox) != HAL_OK) {//发送函数
    //Error_Handler();
  }
}

//发送对应的MotorSend结构体
void CanSendMotor(MotorSend *motorsend){
CAN_SendMessage(motorsend->data,motorsend->StdId);
}

void RecReceiveMotor(Motor *motor,uint8_t *data){//接收对应的电机数据
  int16_t rpm_raw = (int16_t)((data[2]<<8)|data[3]);
  int16_t torque_raw = (int16_t)((data[4]<<8)|data[5]);

  motor->motorState.rpmRaw = (double)rpm_raw;
  motor->motorState.torqueRaw = (double)torque_raw;

  motor->motorState.tempr=(int8_t)data[6];
  if (motor->motorState.isOnline == 0U) {
    motor->motorState.onlineSinceTick = HAL_GetTick();
  }
  motor->motorState.lastRxTick = HAL_GetTick();
  motor->motorState.isOnline = 1;
  motor->motorState.newDataFlag = 1; // 标记收到新数据
  int deltaAngle = (int)(((uint16_t)data[0]<<8)|data[1])-(int)motor->motorState.singleAngle;//角度增量
  motor->motorState.singleAngle = ((uint16_t)data[0]<<8)|data[1];//记录当前角度
  if(abs(deltaAngle) < 4096) motor->motorState.angle += deltaAngle;
}

// 在任务上下文中做滤波，避免在中断里执行浮点运算
void MotorFilterUpdate(Motor *motor)//将滤波值更新到电机中
{
  uint8_t use_filter;

  if (motor == NULL) {
    return;
  }

  // 只有收到新数据才更新滤波，避免定时器与CAN接收异步导致差拍干扰
  if (motor->motorState.newDataFlag == 0) {
    return;
  }
  motor->motorState.newDataFlag = 0;

  use_filter = (motor == &fric1) || (motor == &fric2) || (motor == &fric3) || (motor == &fric4) || (motor == &lift);
  if (!use_filter) {
    motor->motorState.rpm = motor->motorState.rpmRaw;
    motor->motorState.torque = motor->motorState.torqueRaw;
    return;
  }

  if (motor->motorState.rpmFilter.magic != FILTER_MAGIC) {
    Filter_Init(&motor->motorState.rpmFilter, Fiter_Type_KF, RPM_KF_Q, RPM_KF_R); // rpm用卡尔曼滤波
  }
  if (motor->motorState.torqueFilter.magic != FILTER_MAGIC) {
    Filter_Init(&motor->motorState.torqueFilter, Fiter_Type_LPF, TORQUE_FILTER_ALPHA, 100.0f); // 100Hz采样率
  }

  Filter_update(&motor->motorState.rpmFilter, (float)motor->motorState.rpmRaw);
  Filter_update(&motor->motorState.torqueFilter, (float)motor->motorState.torqueRaw);

  motor->motorState.rpm = (double)Filter_get_output(&motor->motorState.rpmFilter);
  motor->motorState.torque = (double)Filter_get_output(&motor->motorState.torqueFilter);
}

//根据电机的StdId来决定发送到哪个MotorSend结构体中
void TransferToMotorSend(Motor *motor){
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

// CAN 接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//HAL库接收中断回调
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];

  if (hcan->Instance == CAN2)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
      // 仅处理电机反馈ID(0x201~0x207)，避免数组越界导致HardFault
      if (RxHeader.IDE == CAN_ID_STD && RxHeader.StdId >= 0x201 && RxHeader.StdId <= 0x207)
      {
        uint32_t idx = RxHeader.StdId - 0x201;
        if (idx < MOTOR_NUM)
        {
          RecReceiveMotor(motor_array[idx], RxData);
        }
      }
    }
  }
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

void MotorInit(Motor *motor,uint32_t StdId,uint8_t motor_byte)
  //,uint8_t maxTemp,double maxTorque, double stallOutput, double stallSpeedThreshold, uint32_t stallTimeThreshold)
{//初始化电机结构体
  motor->StdId=StdId;
  motor->motor_byte=motor_byte;
  motor->motorState.motorMode=disable;
  motor->motorState.singleAngle=0;
  motor->motorState.rpmRaw=0;
  motor->motorState.torqueRaw=0;
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
  motor->motorState.lastRxTick = HAL_GetTick();
  motor->motorState.isOnline = 0;
  motor->motorState.onlineSinceTick = 0;
  motor->targetAngle = 0;
  motor->runSpeed = 0;
  motor->runTime = 0;
  motor->runTimer = 0;
  
  // 初始化PID指针为空，防止未配置模式时误计算
  motor->anglePid.inputAdress = NULL;
  motor->anglePid.outputAdress = NULL;
  motor->speedPid.inputAdress = NULL;
  motor->speedPid.outputAdress = NULL;
  motor->torquePid.inputAdress = NULL;
  motor->torquePid.outputAdress = NULL;

  // 初始化滤波器
  (void)Filter_Init(&motor->motorState.rpmFilter, Fiter_Type_KF, RPM_KF_Q, RPM_KF_R);//rpm采用卡尔曼滤波
  (void)Filter_Init(&motor->motorState.torqueFilter, Fiter_Type_LPF, TORQUE_FILTER_ALPHA, 100.0f);
}
void MotorSafetyInit(Motor *motor, uint8_t maxTemp, double maxTorque, double stallOutput, double stallSpeedThreshold, uint32_t stallTimeThreshold){
  motor->maxTemp = maxTemp;
  motor->maxTorque = maxTorque;
  motor->stallOutput = stallOutput;
  motor->stallSpeedThreshold = stallSpeedThreshold;
  motor->stallTimeThreshold = stallTimeThreshold;
}

//初始化PID
void PidInit(Pid *pid,double Kp,double Ki,double Kd,double maxOutput,double deadband,double intergralLimit){
  pid->Kp=Kp;
  pid->Ki=Ki;
  pid->Kd=Kd;
  pid->maxOutput=maxOutput;
  pid->deadband=deadband;
  pid->intergralLimit=intergralLimit;
}

void dartParamInit(){
  dartParam_array[0].yaw   = DART_AUTO_YAW_0;
  dartParam_array[0].v1Speed = DART_AUTO_V1_0;
  dartParam_array[0].v2Speed = DART_AUTO_V2_0;
  dartParam_array[1].yaw   = DART_AUTO_YAW_1;
  dartParam_array[1].v1Speed = DART_AUTO_V1_1;
  dartParam_array[1].v2Speed = DART_AUTO_V2_1;
  dartParam_array[2].yaw   = DART_AUTO_YAW_2;
  dartParam_array[2].v1Speed = DART_AUTO_V1_2;
  dartParam_array[2].v2Speed = DART_AUTO_V2_2;
  dartParam_array[3].yaw   = DART_AUTO_YAW_3;
  dartParam_array[3].v1Speed = DART_AUTO_V1_3;
  dartParam_array[3].v2Speed = DART_AUTO_V2_3;
}

// 单发飞镖（阻塞）：yaw=角度(0~490000), v1Speed/v2Speed=摩擦轮转速
void DartFireSingle(double yaw, double v1Speed, double v2Speed){
  MotorRunToAngleBlocking(&GM6020, yaw, 300);
  MotorSetOutput(&fric1, speedMode, -v1Speed);
  MotorSetOutput(&fric2, speedMode, -v2Speed);
  MotorSetOutput(&fric3, speedMode, v1Speed);
  MotorSetOutput(&fric4, speedMode, v2Speed);
  MotorRunToStall(&load, -3000);
  MotorRunSpeedTimeBlocking(&lift, 30000, 3700);
  MotorSetOutput(&fric1, speedMode, 0);
  MotorSetOutput(&fric2, speedMode, 0);
  MotorSetOutput(&fric3, speedMode, 0);
  MotorSetOutput(&fric4, speedMode, 0);
  MotorRunSpeedTimeBlocking(&lift, -30000, 3000);
  MotorRunToStall(&lift, -6000);
  MotorRunToStall(&load, -3000);
}


//设置输出
void MotorSetOutput(Motor *motor, enum MotorMode mode, double value){
  motor->motorState.motorMode = mode;
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
  case speedTimeMode:
    motor->speedPid.setpoint = value;
    motor->speedPid.inputAdress = &motor->motorState.rpm;
    motor->speedPid.outputAdress = &motor->output;
    break;
  case torqueMode:
    motor->torquePid.setpoint = value;
    motor->torquePid.inputAdress = &motor->motorState.torque;
    motor->torquePid.outputAdress = &motor->output;
    break;
  case runToStallMode: // Non-blocking run to stall
    // Setup speed PID but user must ensure target is set via structure or external logic
     motor->speedPid.inputAdress = &motor->motorState.rpm;
     motor->speedPid.outputAdress = &motor->output;
     motor->motorState.isStalled = 0; // Clear stall flag
    break;
  default:
    break;
  }
}

//以指定速度运行电机直到堵转
void MotorRunToStall(Motor *motor, double speed){
  //enum MotorMode lastMode = motor->motorState.motorMode;
  motor->motorState.motorMode = speedMode;
  motor->speedPid.setpoint = speed;
  motor->speedPid.inputAdress = &motor->motorState.rpm;
  motor->speedPid.outputAdress = &motor->output;
  while(motor->motorState.isStalled==0){
    osDelay(1);
  }
  motor->motorState.isStalled=0;
  motor->motorState.motorMode=disable;
  motor->motorState.stallTimer=0;
  motor->speedPid.setpoint = 0;
  //MotorSetOutput(motor,angleMode, motor->motorState.angle);
  motor->output = 0;
}

//使电机到指定角度
void MotorRunToAngleBlocking(Motor *motor, double angle, double speed){
  if(motor->motorState.angle < angle){
    motor->motorState.motorMode = speedMode;
    motor->speedPid.setpoint = fabs(speed);
    motor->speedPid.inputAdress = &motor->motorState.rpm;
    motor->speedPid.outputAdress = &motor->output;
    while(motor->motorState.angle < angle){
      if(motor->motorState.isStalled) break; // 堵转检测
      osDelay(1);
    }
  }else{
    motor->motorState.motorMode = speedMode;
    motor->speedPid.setpoint = -fabs(speed);
    motor->speedPid.inputAdress = &motor->motorState.rpm;
    motor->speedPid.outputAdress = &motor->output;
    while(motor->motorState.angle > angle){
      if(motor->motorState.isStalled) break; // 堵转检测
      osDelay(1);
    }
  }
  motor->speedPid.setpoint = 0;
  motor->motorState.isStalled = 0; // Clear flag
  MotorSetOutput(motor, angleMode, angle);
}

//以指定速度运行指定时间
void MotorRunSpeedTime(Motor *motor, double speed, uint32_t time){
  motor->runSpeed = speed;
  motor->runTime = time;
  motor->runTimer = 0; // Reset timer
  MotorSetOutput(motor, speedTimeMode, 0);
}

//以指定速度运行指定时间(阻塞)
void MotorRunSpeedTimeBlocking(Motor *motor, double speed, uint32_t time){
  MotorRunSpeedTime(motor, speed, time);
  while(motor->runTimer < motor->runTime){
    osDelay(1);
  }
};

//摩擦轮预热控温：以 warmSpeed 升温至 targetTemp，之后用 holdSpeed 维持
//当 RunningTask 不再是 5 时退出并停止摩擦轮，允许发射任务接管
void MotorRunToSetTemp(Motor *motor, double targetTemp, double warmSpeed, double holdSpeed){
  // 先以 warmSpeed 快速升温
  MotorSetOutput(motor, speedMode, warmSpeed);
  while(motor->motorState.tempr < targetTemp){
    if(RunningTask != 5) goto exit;
    osDelay(10);
  }
  // 达到目标温度后进入恒温循环
  while(1){
    if(RunningTask != 5) goto exit;
    if(motor->motorState.tempr > targetTemp + 1){
      MotorSetOutput(motor, speedMode, 0);
      while(motor->motorState.tempr > targetTemp){
        if(RunningTask != 5) goto exit;
        osDelay(10);
      }
      MotorSetOutput(motor, speedMode, holdSpeed);
    } else if(motor->motorState.tempr < targetTemp - 1){
      MotorSetOutput(motor, speedMode, warmSpeed);
    } else {
      MotorSetOutput(motor, speedMode, holdSpeed);
    }
    osDelay(10);
  }
exit:
  MotorSetOutput(motor, speedMode, 0);
  return;
};



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == KEY_Pin){
    alarm_level = 3; // 设置最高优先级报警（按键触发）
    for(int i=0;i<MOTOR_NUM;i++){
      motor_array[i]->enabled=0;//将电机全部急停
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL) {
    return;
  }

  if (huart == g_referee.cfg.huart) {
    (void)Referee_OnUartRxCplt(&g_referee, HAL_GetTick());
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL) {
    return;
  }

  if (huart == g_referee.cfg.huart) {
    (void)Referee_StartUartReceive(&g_referee);
  }
}

//发送motor_array对应序号电机的反馈信息到CDC
void MotorCdcFeedback(uint8_t motor_SN){
  Motor *motor = motor_array[motor_SN];
  uint8_t feedback[100];
  feedback[0]=0x81;//由于约定CDC主机发送命令第一字节标志位从0x00开始，所以反馈从0x80开始(1000 0000 B)
  feedback[1]=motor_SN; // 电机序号
  feedback[2]=motor->motorState.singleAngle>>8;
  feedback[3]=motor->motorState.singleAngle&0xFF;
  feedback[4]=(int16_t)motor->motorState.rpm>>8;
  feedback[5]=(int16_t)motor->motorState.rpm&0xFF;
  feedback[6]=(int16_t)motor->motorState.torque>>8;
  feedback[7]=(int16_t)motor->motorState.torque&0xFF;
  feedback[8]=motor->motorState.tempr;
  feedback[9]=(uint8_t)((motor->enabled << 7) | (motor->motorState.isStalled << 6) | (motor->motorState.motorMode & 0x3F));
  feedback[10]=(int32_t)motor->motorState.angle>>24;
  feedback[11]=(int32_t)motor->motorState.angle>>16;
  feedback[12]=(int32_t)motor->motorState.angle>>8;
  feedback[13]=(int32_t)motor->motorState.angle&0xFF;
  CDC_Transmit_FS(feedback,14);//发送反馈数据包到上位机，长度14字节
}

void RefereeCdcFeedback(void){
  static uint8_t seq = 0;
  uint8_t feedback[45];
  referee_status_t status = Referee_GetStatus(&g_referee);
  uint8_t has_flags = 0;

  if (g_referee.dart.has_dart_info != 0U) {
    has_flags |= 0x01U;
  }
  if (g_referee.dart.has_dart_client_cmd != 0U) {
    has_flags |= 0x02U;
  }
  if (g_referee.robot.has_robot_status != 0U) {
    has_flags |= 0x04U;
  }
  if (g_referee.robot.has_robot_hp != 0U) {
    has_flags |= 0x08U;
  }
  if (g_referee.dart.has_dart_param != 0U) {
    has_flags |= 0x10U;
  }

  feedback[0] = 0x82;
  feedback[1] = (uint8_t)status;
  feedback[2] = g_dart_info_cache.dart_remaining_time;
  feedback[3] = (uint8_t)(g_dart_info_cache.dart_info >> 8);
  feedback[4] = (uint8_t)(g_dart_info_cache.dart_info & 0xFF);
  feedback[5] = g_dart_cmd_cache.dart_launch_opening_status;
  feedback[6] = g_dart_cmd_cache.reserved;
  feedback[7] = (uint8_t)(g_dart_cmd_cache.target_change_time >> 8);
  feedback[8] = (uint8_t)(g_dart_cmd_cache.target_change_time & 0xFF);
  feedback[9] = (uint8_t)(g_dart_cmd_cache.latest_launch_cmd_time >> 8);
  feedback[10] = (uint8_t)(g_dart_cmd_cache.latest_launch_cmd_time & 0xFF);
  feedback[11] = seq++;
  feedback[12] = has_flags;
  feedback[13] = (uint8_t)(g_referee.runtime.parse_ok_count >> 24);
  feedback[14] = (uint8_t)(g_referee.runtime.parse_ok_count >> 16);
  feedback[15] = (uint8_t)(g_referee.runtime.parse_ok_count >> 8);
  feedback[16] = (uint8_t)(g_referee.runtime.parse_ok_count & 0xFF);
  feedback[17] = (uint8_t)(g_referee.runtime.parse_err_count >> 24);
  feedback[18] = (uint8_t)(g_referee.runtime.parse_err_count >> 16);
  feedback[19] = (uint8_t)(g_referee.runtime.parse_err_count >> 8);
  feedback[20] = (uint8_t)(g_referee.runtime.parse_err_count & 0xFF);
  feedback[21] = g_referee.robot.robot_status.robot_id;
  feedback[22] = g_referee.robot.robot_status.robot_level;
  feedback[23] = (uint8_t)(g_referee.robot.robot_status.remain_HP >> 8);
  feedback[24] = (uint8_t)(g_referee.robot.robot_status.remain_HP & 0xFF);
  feedback[25] = (uint8_t)(g_referee.robot.robot_status.max_HP >> 8);
  feedback[26] = (uint8_t)(g_referee.robot.robot_status.max_HP & 0xFF);
  feedback[27] = (uint8_t)(g_referee.robot.robot_hp.ally_1_robot_HP >> 8);
  feedback[28] = (uint8_t)(g_referee.robot.robot_hp.ally_1_robot_HP & 0xFF);
  feedback[29] = (uint8_t)(g_referee.robot.robot_hp.ally_outpost_HP >> 8);
  feedback[30] = (uint8_t)(g_referee.robot.robot_hp.ally_outpost_HP & 0xFF);
  feedback[31] = (uint8_t)(g_referee.robot.robot_hp.ally_base_HP >> 8);
  feedback[32] = (uint8_t)(g_referee.robot.robot_hp.ally_base_HP & 0xFF);
  // 0x0301 飞镖参数（非消费式读取，不干扰 RunningTask=7 消费）
  feedback[33] = g_referee.dart.has_dart_param;
  feedback[34] = g_referee.dart.dart_param_item.dart_id;
  feedback[35] = (uint8_t)(g_referee.dart.dart_param_item.v1Speed >> 8);
  feedback[36] = (uint8_t)(g_referee.dart.dart_param_item.v1Speed & 0xFF);
  feedback[37] = (uint8_t)(g_referee.dart.dart_param_item.v2Speed >> 8);
  feedback[38] = (uint8_t)(g_referee.dart.dart_param_item.v2Speed & 0xFF);
  feedback[39] = (uint8_t)(g_referee.dart.dart_param_item.yaw >> 24);
  feedback[40] = (uint8_t)(g_referee.dart.dart_param_item.yaw >> 16);
  feedback[41] = (uint8_t)(g_referee.dart.dart_param_item.yaw >> 8);
  feedback[42] = (uint8_t)(g_referee.dart.dart_param_item.yaw & 0xFF);
  CDC_Transmit_FS(feedback, 43);
}

void CDC_Receive_Callback(uint8_t *Buf, uint32_t Len)
{
    // 解析自定义数据包
    // Byte 0: 0x00 (Header)
    // Byte 1: Motor ID (0-6)
    // Byte 2: Mode 
/*
    //////////////////////////////////////////////////
    CDC数据包解释
    指令数据包（发送给stm32）
    Byte 0: 0x00
      Byte 1: Motor ID (0-6)
      Byte 2: Mode (0-7)
        0: Disable
        1: Current Mode
          Byte 3-4: Value (int16, Big Endian)
        2: Angle Mode
          Byte 3-4: Value (int16, Big Endian)
        3: Speed Mode
          Byte 3-4: Value (int16, Big Endian)
        4: Torque Mode
          Byte 3-4: Value (int16, Big Endian)
        5: RunToStall (Non-blocking)
          Byte 3-4: Speed (int16, Big Endian)
        6: RunToAngle (Non-blocking)
          Byte 3-10: Angle (double, Big Endian from Python)
          Byte 11-12: Speed (int16, Big Endian)
        7: SpeedTimeMode (Non-blocking)
          Byte 3-4: Speed (int16, Big Endian)
          Byte 5-8: Time ms (uint32, Big Endian)
    Byte 0: 0x01 (System Command)
      Byte 1: 0x00 = Emergency Stop -> alarm_level=3, disable all motors
      Byte 1: Other = Set RunningTask
    Byte 0: 0x02 (Value Set)
      Byte 1: 0x00 (dartParam Set)//设置的参数是飞镖发射参数
        Byte 2: Dart ID (0-3)
          Byte 3-4: Dart's yaw value (double, Big Endian)
          Byte 5-6: Dart's v1Speed (double, Big Endian)
          Byte 7-8: Dart's v2Speed (double, Big Endian)
    ///////////////////////////////
    反馈数据包（发送给上位机）
    Byte 0: 0x81 (Header)
      Byte 1: Motor ID (0-6)
      Byte 2-3: Single Angle (uint16, Big Endian)
      Byte 4-5: RPM (int16, Big Endian)
      Byte 6-7: Torque (int16, Big Endian)
      Byte 8: Temp (int8)
      Byte 9: Flags (7: Enabled, 6: Stalled, 5-0: Mode)
      Byte 10-13: Total Angle (int32, Big Endian)
    ///////////////////////////////
    
    
    
    
    /////////////////////////////////////////////////
*/
    //if(CDC_Ctrl_state != 1) return; // 未连接时不处理
    if (Len >= 5 && Buf[0] == 0x00) {
        uint8_t motor_id = Buf[1];
        uint8_t mode_val = Buf[2];
        
        if (motor_id < MOTOR_NUM) {
             motor_array[motor_id]->enabled = 1;
             motor_array[motor_id]->motorState.isStalled = 0;
             motor_array[motor_id]->motorState.stallTimer = 0;
             if (mode_val <= 4) {
                 // 立即设置 (非阻塞)
                 // Byte 3-4: Value (int16, Big Endian)
                 int16_t val_int16 = (int16_t)((Buf[3] << 8) | Buf[4]);
                 double val_double = (double)val_int16;
                 
                 MotorSetOutput(motor_array[motor_id], (enum MotorMode)mode_val, val_double);
             
             } else if (mode_val == 0x05) {
                 // Mode 5: RunToStall (Non-blocking)
                 // Byte 3-4: Speed (int16, Big Endian)
                 int16_t val_int16 = (int16_t)((Buf[3] << 8) | Buf[4]);
                 
                 Motor *motor = motor_array[motor_id];
                 motor->runSpeed = (double)val_int16;
                 MotorSetOutput(motor, runToStallMode, 0);
                 
             } else if (mode_val == 0x06 && Len >= 13) {
                 // Mode 6: RunToAngle (Non-blocking)
                 // Byte 3-10: Angle (double, Big Endian from Python)
                 // Byte 11-12: Speed (int16, Big Endian)
                 
                 uint8_t temp[8];
                 // Reverse bytes for correct double representation if Python sends Big Endian
                 // STM32 is Little Endian
                 for(int i=0; i<8; i++) {
                     temp[i] = Buf[10 - i];
                 }
                 double angle_val;
                 memcpy(&angle_val, temp, 8);
                 
                 int16_t speed_int16 = (int16_t)((Buf[11] << 8) | Buf[12]);
                 
                 Motor *m = motor_array[motor_id];
                 m->targetAngle = angle_val;
                 m->runSpeed = (double)speed_int16;
                 // Manually setup for runToAngle
                 m->speedPid.inputAdress = &m->motorState.rpm;
                 m->speedPid.outputAdress = &m->output;
                 m->motorState.motorMode = runToAngle; 
             } else if (mode_val == 0x07 && Len >= 9) {
                 // Mode 7: SpeedTimeMode
                 // Byte 3-4: Speed (int16, Big Endian)
                 // Byte 5-8: Time ms (uint32, Big Endian)

                 int16_t val_int16 = (int16_t)((Buf[3] << 8) | Buf[4]);
                 uint32_t val_time = (uint32_t)((Buf[5] << 24) | (Buf[6] << 16) | (Buf[7] << 8) | Buf[8]);
                 MotorRunSpeedTime(motor_array[motor_id], (double)val_int16, val_time);
             }
        }
    }
    else if (Len >= 2 && Buf[0] == 0x01) {
        // System Command
        // Byte 1: 0x00 = Emergency Stop -> alarm_level=3, disable all motors
      // Byte 1: 0x01~0x7F = Set RunningTask
      // Byte 1: 0xFE = Clear safety latch and re-enable all motors
        
        if (Buf[1] == 0x00) {
            alarm_level = 3;
            for(int i=0; i<MOTOR_NUM; i++){
                motor_array[i]->enabled = 0;
            }
      } else if (Buf[1] == 0xFE) {
        alarm_level = 0;
        alarm_motor = -1;
        for(int i=0; i<MOTOR_NUM; i++){
          motor_array[i]->enabled = 1;
          motor_array[i]->motorState.isStalled = 0;
          motor_array[i]->motorState.stallTimer = 0;
        }
        } else {
            RunningTask = Buf[1];
        }
    }
    else if (Len >= 15 && Buf[0] == 0x02) {
        // Value Set
        // Byte 1: 0x00 (dartParam Set)
        if (Buf[1] == 0x00) {
            uint8_t dart_id = Buf[2];
            if (dart_id < 4) {
                 // Byte 3-10: Dart's yaw value (double, Big Endian from Python)
                 uint8_t temp[8];
                 for(int i=0; i<8; i++) temp[i] = Buf[10 - i]; // Reverse for Little Endian
                 
                 double yaw_val;
                 memcpy(&yaw_val, temp, 8);
                 
                 // Byte 11-12: Dart's v1Speed
                 int16_t v1_val = (int16_t)((Buf[11] << 8) | Buf[12]);
                 // Byte 13-14: Dart's v2Speed
                 int16_t v2_val = (int16_t)((Buf[13] << 8) | Buf[14]);
                 
                 dartParam_array[dart_id].yaw = yaw_val + 245000.0;//245000 is 0
                 dartParam_array[dart_id].v1Speed = (double)v1_val;
                 dartParam_array[dart_id].v2Speed = (double)v2_val;
            }
        }
    }

    // 将接收到的数据回显（Echo）
    // 注意：CDC_Transmit_FS 如果正在忙碌可能会失败，实际应用可以使用缓冲区或重试机制
    CDC_Transmit_FS(Buf, Len);
}

#define LENGTH 624
const uint8_t Music_Score[LENGTH]={//校歌乐谱数据，用在Singing函数中
	
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,60,60,65,65,69,69,74,74,74,74,74,74,74,74,74,74,74,74,72,72,72,72,72,72,72,72,72,72,70,70,69,69,69,69,69,69,67,67,67,67,69,69,62,62,62,62,62,0,64,64,64,64,62,62,60,60,60,60,60,60,72,72,72,70,70,70,69,69,69,67,67,67,67,67,67,69,69,69,65,65,65,65,65,0,65,65,0,65,65,0,65,65,65,0,0,0,60,60,0,60,60,60,65,65,65,65,65,65,67,67,67,67,67,67,69,69,69,69,69,69,67,67,67,69,69,69,67,67,67,67,67,67,65,65,65,65,65,65,64,64,64,62,62,62,62,62,62,64,64,64,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,62,62,0,62,62,62,67,67,67,67,67,67,69,69,69,69,69,69,70,70,70,70,70,70,74,74,0,74,74,74,72,72,72,72,72,72,65,65,65,65,65,65,70,70,70,69,69,69,69,69,69,65,65,65,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,69,69,69,70,70,70,72,72,72,72,72,72,72,72,72,72,72,0,72,72,72,72,72,72,70,70,0,70,70,70,69,69,69,69,69,69,65,65,65,65,65,65,64,64,64,64,64,64,65,65,65,65,65,65,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,67,67,67,69,69,69,70,70,70,70,70,70,70,70,70,70,70,70,74,74,74,74,74,74,72,72,72,74,74,74,72,72,72,72,72,72,70,70,70,70,70,70,69,69,69,67,67,67,67,67,67,69,69,69,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,74,74,74,74,74,74,74,74,0,74,74,74,72,72,0,72,72,72,65,65,65,67,67,67,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,74,74,74,74,74,74,74,74,0,74,74,74,72,72,0,72,72,72,65,65,65,67,67,67,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,70,70,70,0,70,70,69,69,69,65,65,65,62,62,62,62,62,62,62,62,62,62,62,62,70,70,70,0,70,70,69,69,69,65,65,65,62,62,62,62,62,62,60,60,0,60,60,60,65,65,65,65,65,65,69,69,69,69,69,69,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,72,72,0,72,72,72,72,72,72,70,70,70,69,69,69,67,67,67,67,67,67,69,69,69,65,65,65,65,65,65,65,65,65,65,65,65
	
};
void Singing(){
  MotorUpdateFlag=0;
  for(int i=0;i<LENGTH;i++){
    if(Music_Score[i]==0){
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    }else{
            // 现场计算频率 (Base: C3=130.81Hz, C4=261.63Hz)
            // C Major semitones: 0(C), 2(D), 4(E), 5(F), 7(G), 9(A), 11(B)
            //int semitones_map[] = {0, 2, 4, 5, 7, 9, 11};
            // Correct formula: freq = 440 * 2^((n-69)/12) OR 261.63 * 2^((n-60)/12)
            float freq = 261.63f * pow(2.0f, (float)(Music_Score[i] - 60) / 12.0f);
            
            // Limit frequency to avoid div by zero or extreme values
            if(freq < 20.0f) freq = 20.0f;
            if(freq > 20000.0f) freq = 20000.0f;

            // ARR = (TimerClock / Frequency) - 1. TimerClock = 1MHz
            uint32_t reload = (uint32_t)(1000000.0f / freq) - 1;
            __HAL_TIM_SET_AUTORELOAD(&htim4, reload);
            // 同时更新占空比为50% (CCR = ARR/2)
             __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, reload / 2);
    }
    osDelay(75);
    if(RunningTask!=3){
      MotorUpdateFlag=1;
      return;
    }        
  }
  MotorUpdateFlag=1;
  return;
}

#define LENGTH_SOME 93//校歌片段乐谱数据，用在SingingSome函数中
const uint8_t Music_Score_Some[LENGTH_SOME]={
	
	60,60,65,65,69,69,74,74,74,74,74,74,74,74,74,74,74,74,72,72,72,72,72,72,72,72,72,72,70,70,69,69,69,69,69,69,67,67,67,67,69,69,62,62,62,62,62,0,64,64,64,64,62,62,60,60,60,60,60,60,72,72,72,70,70,70,69,69,69,67,67,67,67,67,67,69,69,69,65,65,65,65,65,0,65,65,0,65,65,0,65,65,65
};
void SingingSome(){
  MotorUpdateFlag=0;
  for(int i=0;i<LENGTH_SOME;i++){
    if(Music_Score_Some[i]==0){
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    }else{
            // 现场计算频率 (Base: C3=130.81Hz, C4=261.63Hz)
            // C Major semitones: 0(C), 2(D), 4(E), 5(F), 7(G), 9(A), 11(B)
            //int semitones_map[] = {0, 2, 4, 5, 7, 9, 11};
            // Correct formula: freq = 440 * 2^((n-69)/12) OR 261.63 * 2^((n-60)/12)
            float freq = 261.63f * pow(2.0f, (float)(Music_Score_Some[i] - 60) / 12.0f);
            
            // Limit frequency to avoid div by zero or extreme values
            if(freq < 20.0f) freq = 20.0f;
            if(freq > 20000.0f) freq = 20000.0f;

            // ARR = (TimerClock / Frequency) - 1. TimerClock = 1MHz
            uint32_t reload = (uint32_t)(1000000.0f / freq) - 1;
            __HAL_TIM_SET_AUTORELOAD(&htim4, reload);
            // 同时更新占空比为50% (CCR = ARR/2)
             __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, reload / 2);
    }
    osDelay(50); 
  }
  MotorUpdateFlag=1;
  return;
}

static void BuzzerSetFrequencyHz(uint32_t freq_hz)
{
  uint32_t reload;

  if (freq_hz < 20U) {
    freq_hz = 20U;
  }
  if (freq_hz > 20000U) {
    freq_hz = 20000U;
  }

  reload = (1000000U / freq_hz) - 1U;
  __HAL_TIM_SET_AUTORELOAD(&htim4, reload);
}

static void BuzzerToneOn50Duty(void)
{
  uint32_t reload = __HAL_TIM_GET_AUTORELOAD(&htim4);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, reload / 2U);
}

////////////////////////////////////////////////////////////////////////////////////
//                                                                                //
//                                                                                //
//                      从此处开始以下为FreeRTOS Task函数                           //
//                                                                                //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

//电机状态更新Task，负责判断电机当前的状态然后决策电机的运动
void MotorUpdate(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  static int buzzer_tick = 0;
  static int buzzer_oneshot = 0; // 短响计时器
  
  alarm_level = 0;

  // 确保开启 TIM4 CH3 的 PWM 输出
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  /* Infinite loop */
  for(;;)
  {
    (void)Referee_Update(&g_referee, HAL_GetTick());//周期维护在线状态

    // 读取并缓存飞镖相关数据，供其他任务或调试输出使用
    (void)Referee_GetDartInfo(&g_referee, &g_dart_info_cache);
    (void)Referee_GetDartClientCmd(&g_referee, &g_dart_cmd_cache);
    
    // alarm_level 不在每次循环重置，保持报警状态直到系统复位或手动清除，
      //所以若电机因安全问题被禁用，请按复位键重置系统

    for(int i=0;i<MOTOR_NUM;i++){
      Motor *m = motor_array[i];//遍历每一个电机来更新数据
      uint32_t now = HAL_GetTick();
      uint8_t was_online = m->motorState.isOnline;
      uint8_t is_online_stable;

      if ((now - m->motorState.lastRxTick) > MOTOR_OFFLINE_TIMEOUT_MS) {
        m->motorState.isOnline = 0;
      }

      if (m->motorState.isOnline == 0) {
        // 电机掉电或总线断开时，清空观测量并复位堵转状态
        // 不强制清零输出，避免上位机已下发控制但因延迟上电被永久压制
        m->motorState.rpmRaw = 0;
        m->motorState.torqueRaw = 0;
        m->motorState.rpm = 0;
        m->motorState.torque = 0;
        m->motorState.stallTimer = 0;
        m->motorState.isStalled = 0;
      } else if (was_online == 0) {
        // 重新收到反馈后，自动解除离线期可能出现的保护锁定
        m->enabled = 1;
      }

      is_online_stable = (uint8_t)(m->motorState.isOnline && ((now - m->motorState.onlineSinceTick) >= MOTOR_ONLINE_WARMUP_MS));

      MotorFilterUpdate(m);
      // PID calculation moved to StartPidTask
      if(m->motorState.motorMode == disable){
        m->motorState.isStalled = 0; // Clear stalled flag
      }
      // Handle runToAngle status
      if (m->motorState.motorMode == runToAngle) {
          // 如果堵转，切换到 angleMode 并在当前位置保持
          if (m->motorState.isStalled) {
              MotorSetOutput(m, angleMode, m->motorState.angle);
              m->motorState.isStalled = 0; // Clear stalled flag to allow new moves
          } else {
              // 简单的 bang-bang 速度控制向目标角度移动
              // 或者更好的 P 控制以避免震荡，这里按用户原逻辑使用固定速度
              double diff = m->motorState.angle - m->targetAngle;
              
              if (fabs(diff) < 100) { // 阈值判定到达
                  MotorSetOutput(m, angleMode, m->targetAngle);
              } else {
                 // 继续运行
                 if (m->motorState.angle < m->targetAngle) {
                     m->speedPid.setpoint = fabs(m->runSpeed);
                 } else {
                     m->speedPid.setpoint = -fabs(m->runSpeed);
                 }
                 // 在 runToAngle 模式下，StartPidTask 会计算 speedPid
              }
          }
      } else if (m->motorState.motorMode == runToStallMode) {
          if (m->motorState.isStalled) {
              m->motorState.motorMode = disable;
              m->output = 0;
              m->speedPid.setpoint = 0;
              m->motorState.isStalled = 0;
          } else {
              m->speedPid.setpoint = m->runSpeed;
          }
      } else if (m->motorState.motorMode == speedTimeMode) {
          if (m->runTimer < m->runTime&&m->motorState.isStalled==0) {
              m->runTimer++;
              m->speedPid.setpoint = m->runSpeed;
          } else {
              m->motorState.motorMode = speedMode;
              m->output = 0;
              m->speedPid.setpoint = 0;
          }
      }

      // Safety Checks
      // 1. 堵转检测
      // 使用最终输出电流值(m->output)和转速(m->motorState.rpm)进行判断
      uint8_t prev_stalled = m->motorState.isStalled;
          if (is_online_stable && fabs(m->output) > m->stallOutput && fabs(m->motorState.rpm) < m->stallSpeedThreshold) {
          m->motorState.stallTimer++;
      } else {
          m->motorState.stallTimer = 0;
          m->motorState.isStalled = 0;
      }
      
      if (m->motorState.stallTimer > m->stallTimeThreshold) {
          m->motorState.isStalled = 1;
      }
      if(MotorUpdateFlag==1){
      // 检测到堵转状态上升沿 (0 -> 1)，触发一次短响
      if (prev_stalled == 0 && m->motorState.isStalled == 1) {
          buzzer_oneshot = 200; // 触发200ms短响
          
          if(i < 7) {
            static const uint16_t stall_freq_map[7] = {740U, 880U, 1040U, 1220U, 1360U, 1560U, 1760U};
            BuzzerSetFrequencyHz(stall_freq_map[i]);
            BuzzerToneOn50Duty();
          }
      }

      // 2. 阈值保护 (温度 > 阈值 OR (转矩 > 阈值(瞬间)))
      // 注意：堵转(isStalled)不再触发禁用或报警
      if (is_online_stable && m->motorState.tempr > m->maxTemp){
        m->enabled = 0; // 立即禁用电机
        if (alarm_level < 1) alarm_level = 1; // 仅升级报警等级，不降级
        alarm_motor = i+1;
        alarm_counter++;
      }
      
      if (is_online_stable && fabs(m->motorState.torque) > m->maxTorque) {
        m->enabled = 0; // 立即禁用电机
        if (alarm_level < 2) alarm_level = 2; // 仅升级报警等级，不降级（扭矩优于温度）
        alarm_motor = i+1;
        alarm_counter++;
      }
    }
      

      if (m->enabled == 0||MotorUpdateFlag == 0) {
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
        // 结束短响，恢复到新的报警频点并静音等待下次触发
        BuzzerSetFrequencyHz(BUZZER_ALERT_FREQ_HZ);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // 暂歇
        }
    }
    else if (alarm_level == 0) {
        if(MotorUpdateFlag == 1) {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // 正常情况静音
        }
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
            BuzzerSetFrequencyHz(BUZZER_ALERT_FREQ_HZ);
            BuzzerToneOn50Duty(); // ON
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

//CDC发送反馈报文Task
void StartCdcTask(void const * argument)//上位机通信任务，负责发送电机状态反馈数据包到上位机
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    for(int i=0;i<MOTOR_NUM;i++){
      MotorCdcFeedback(i);
      osDelay(1); // 关键修改：增加延时，等待USB发送完成
    }
    RefereeCdcFeedback();
    osDelay(1);
    osDelay(90); // 整体刷新频率控制
  }
  /* USER CODE END StartDefaultTask */
}


//主要程序编写在此处
void StartTask2(void const * argument)
{
  ///////////////////////////////////////////////////
  //    lift全程angle:9000000 向上为正              //
  //    GM6020全程angle:490000 顺时针为正           //
  ///////////////////////////////////////////////////
  /* USER CODE BEGIN StartDefaultTask */
  // 1. 初始化电机基本结构 MotorInit(*motor, StdId, MotorByte);
  //MotorSafetyInit(*motor, maxTemp, maxTorque, stallOutput, stallSpeedThreshold, stallTimeThreshold);  
  MotorInit(&fric1, 0x200, 0);
  MotorInit(&fric2, 0x200, 2);
  MotorInit(&fric3, 0x200, 4);
  MotorInit(&fric4, 0x200, 6);
  MotorInit(&lift, 0x1FF, 4);
  MotorSafetyInit(&lift, 35, 20000, 2000, 50,100);
  //lift.enabled=0; // 升降电机初始禁用
  MotorInit(&load, 0x1FF, 2);
  MotorSafetyInit(&load, 35, 5000, 500, 50, 100);
  //load.enabled=0; // 装弹电机初始禁用
  MotorInit(&GM6020, 0x1FE, 0);
  MotorSafetyInit(&GM6020, 35, 5000, 1000, 50, 100);

  // 2. 初始化 GM6020 角度环 PID (内环) 参数: Kp,Ki,Kd,MaxOut,Deadband,I_Limit
  PidInit(&GM6020.anglePid, 1.0, 0.0, 0.0, 4000.0, 0.0, 0.0);
  PidInit(&GM6020.speedPid, 12, 1, 0.0, 4000.0, 0.0, 1000);
  for(int i=0;i<4;i++){
    MotorSafetyInit(motor_array[i], 55, 35000, 500, 50, 100);
    PidInit(&motor_array[i]->speedPid, 60, 1, 1, 15000.0, 0.0, 15000);
  }
  PidInit(&fric1.anglePid, 1, 1, 1000, 3000.0, 0.0, 1000);
  PidInit(&fric2.anglePid, 1, 1, 1000, 3000.0, 0.0, 1000);
  PidInit(&fric3.anglePid, 1, 1, 1000, 3000.0, 0.0, 1000);
  PidInit(&fric4.anglePid, 1, 1, 1000, 3000.0, 0.0, 1000);
  PidInit(&lift.anglePid, 1, 1, 1000, 3000.0, 0.0, 1000);
  PidInit(&lift.speedPid, 1, 0.01, 1, 9000.0, 0.0, 300);
  PidInit(&load.anglePid, 1, 1, 1000, 3000.0, 0.0, 1000);
  PidInit(&load.speedPid, 2, 0.01, 1, 3000.0, 0.0, 300);
  
  // 3. 设置初始输出为 0
  MotorSetOutput(&GM6020, speedMode, 0);
  MotorSetOutput(&fric1, angleMode, 0);
  MotorSetOutput(&fric2, angleMode, 0);
  MotorSetOutput(&fric3, angleMode, 0);
  MotorSetOutput(&fric4, angleMode, 0);
  MotorSetOutput(&lift, speedMode, 0);
  MotorSetOutput(&load, speedMode, 0);

  dartParamInit(); // 初始化飞镖发射参数

  // 4. 初始化外部速度环 PID (外环)
  // 参数: Kp=1.0, Ki=0.0, Kd=0.0 (需根据实际调优), MaxOut=8191(角度最大值), Deadband=0, I_Limit=0
  //PidInit(&outerSpeedPid, 1.0, 0.0, 0.0, 8191.0, 0.0, 0.0);
  
  // 5. 绑定 PID 指针：外环输出 -> 内环输入
  //outerSpeedPid.inputAdress = &GM6020.motorState.rpm;       // 输入：当前 RPM
  //outerSpeedPid.outputAdress = &GM6020.anglePid.setpoint;   // 输出：角度环的目标值(Setpoint)
  
  // 6. 设定外环目标值
  //outerSpeedPid.setpoint = 100.0; // 例如：目标 100 RPM

  /////////////////////////以下为上电初始化程序////////////////////////////
  //上电初始化，GM6020和lift走到负方向限位
  //osDelay(1000);
  //SingingSome();//播放部分音乐以示启动
  MotorRunToStall(&GM6020,-300);
  GM6020.motorState.angle=0;
  MotorRunToAngleBlocking(&GM6020,245000,300);
  //MotorRunToStall(&lift,6000);
  MotorRunToStall(&lift,-6000);
  osDelay(100);
  lift.motorState.angle=0;
  lift.motorState.isStalled=0;
    MotorRunSpeedTimeBlocking(&lift,3000,500);
  //MotorRunToAngleBlocking(&lift,0,300);
  MotorRunToStall(&load,3000);
  osDelay(100);
  MotorRunToStall(&load,-3000);
  osDelay(100);
  load.motorState.angle=0;
  load.motorState.isStalled=0;
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET); // 指示初始化完成
  osDelay(1000);
  MotorSetOutput(&fric1, speedMode, -4000);
  MotorSetOutput(&fric2, speedMode, -4000);
  MotorSetOutput(&fric3, speedMode, 4000);
  MotorSetOutput(&fric4, speedMode, 4000);
  osDelay(1000);
  MotorSetOutput(&fric1, speedMode, 0);
  MotorSetOutput(&fric2, speedMode, 0);
  MotorSetOutput(&fric3, speedMode, 0);
  MotorSetOutput(&fric4, speedMode, 0);
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET); // 指示准备完成
  alarm_level=0;
  CDC_Ctrl_state = 1; // CDC 连接完成，允许接收控制命令
#if DART_COMPETITION_MODE != 0
  RunningTask = DART_COMPETITION_MODE; // 比赛模式下自动启动
#endif
  ////////////////////////////////////////////////////////////////////////////  
  /* Infinite loop */
  for(;;)
  {//RunningTask执行的任务在此处编写
    /* if(fabs(lift.motorState.rpm)>600){
      osDelay(1000);
      RunningTask=1;
    } */
    if(RunningTask==1){
      // 单发程序 - 立即执行，使用 dartParam_array[0] 的参数
      // 先转云台到位，再启动摩擦轮，保证精度同时减少摩擦轮运行时间
      MotorRunToAngleBlocking(&GM6020,dartParam_array[0].yaw,300);
      MotorSetOutput(&fric1, speedMode, -dartParam_array[0].v1Speed);
      MotorSetOutput(&fric2, speedMode, -dartParam_array[0].v2Speed);
      MotorSetOutput(&fric3, speedMode, dartParam_array[0].v1Speed);
      MotorSetOutput(&fric4, speedMode, dartParam_array[0].v2Speed);
      MotorRunToStall(&load,-3000);
      MotorRunSpeedTimeBlocking(&lift,30000,3700);
      MotorSetOutput(&fric1, speedMode, 0);
      MotorSetOutput(&fric2, speedMode, 0);
      MotorSetOutput(&fric3, speedMode, 0);
      MotorSetOutput(&fric4, speedMode, 0);
      MotorRunSpeedTimeBlocking(&lift,-30000,3000);
      MotorRunToStall(&lift,-6000);
      MotorRunToStall(&load,-3000);
      RunningTask=0;
    }
    if(RunningTask==2){
      // 单发程序 - 延迟10秒后执行，使用 dartParam_array[0] 的参数
      osDelay(10000);
      // 先转云台到位，再启动摩擦轮，保证精度同时减少摩擦轮运行时间
      MotorRunToAngleBlocking(&GM6020,dartParam_array[0].yaw,300);
      MotorSetOutput(&fric1, speedMode, -dartParam_array[0].v1Speed);
      MotorSetOutput(&fric2, speedMode, -dartParam_array[0].v2Speed);
      MotorSetOutput(&fric3, speedMode, dartParam_array[0].v1Speed);
      MotorSetOutput(&fric4, speedMode, dartParam_array[0].v2Speed);
      MotorRunToStall(&load,-3000);
      MotorRunSpeedTimeBlocking(&lift,30000,2000);
      MotorSetOutput(&fric1, speedMode, 0);
      MotorSetOutput(&fric2, speedMode, 0);
      MotorSetOutput(&fric3, speedMode, 0);
      MotorSetOutput(&fric4, speedMode, 0);
      MotorRunSpeedTimeBlocking(&lift,-30000,3000);
      MotorRunToStall(&lift,-6000);
      MotorRunToStall(&load,-3000);
      RunningTask=0;
    }
    if(RunningTask==3){
      //在此处写程序
      Singing();//播放音乐
      RunningTask=0;
    }
    if(RunningTask==4){
      //在此处写程序
      //连发程序
      // 先转云台到位，再启动摩擦轮，保证精度同时减少摩擦轮运行时间
      MotorRunToAngleBlocking(&GM6020,dartParam_array[0].yaw,300);
      MotorSetOutput(&fric1, speedMode, -dartParam_array[0].v1Speed);
      MotorSetOutput(&fric2, speedMode, -dartParam_array[0].v2Speed);
      MotorSetOutput(&fric3, speedMode, dartParam_array[0].v1Speed);
      MotorSetOutput(&fric4, speedMode, dartParam_array[0].v2Speed);
      MotorRunToStall(&load,-3000);
      MotorRunSpeedTimeBlocking(&lift,30000,2000);
      MotorRunToAngleBlocking(&GM6020,dartParam_array[1].yaw,300);
      MotorSetOutput(&fric1, speedMode, -dartParam_array[1].v1Speed);
      MotorSetOutput(&fric2, speedMode, -dartParam_array[1].v2Speed);
      MotorSetOutput(&fric3, speedMode, dartParam_array[1].v1Speed);
      MotorSetOutput(&fric4, speedMode, dartParam_array[1].v2Speed);
      MotorRunSpeedTimeBlocking(&lift,30000,1700);
      MotorSetOutput(&fric1, speedMode, 0);
      MotorSetOutput(&fric2, speedMode, 0);
      MotorSetOutput(&fric3, speedMode, 0);
      MotorSetOutput(&fric4, speedMode, 0);
      MotorRunSpeedTimeBlocking(&lift,-30000,3000);
      MotorRunToStall(&lift,-6000);

      MotorRunToAngleBlocking(&GM6020,dartParam_array[2].yaw,300);
      MotorSetOutput(&fric1, speedMode, -dartParam_array[2].v1Speed);
      MotorSetOutput(&fric2, speedMode, -dartParam_array[2].v2Speed);
      MotorSetOutput(&fric3, speedMode, dartParam_array[2].v1Speed);
      MotorSetOutput(&fric4, speedMode, dartParam_array[2].v2Speed);
      MotorRunToStall(&load,3000);
      MotorRunSpeedTimeBlocking(&lift,30000,2000);
      MotorRunToAngleBlocking(&GM6020,dartParam_array[3].yaw,300);
      MotorSetOutput(&fric1, speedMode, -dartParam_array[3].v1Speed);
      MotorSetOutput(&fric2, speedMode, -dartParam_array[3].v2Speed);
      MotorSetOutput(&fric3, speedMode, dartParam_array[3].v1Speed);
      MotorSetOutput(&fric4, speedMode, dartParam_array[3].v2Speed);
      MotorRunSpeedTimeBlocking(&lift,30000,1700);
      MotorSetOutput(&fric1, speedMode, 0);
      MotorSetOutput(&fric2, speedMode, 0);
      MotorSetOutput(&fric3, speedMode, 0);
      MotorSetOutput(&fric4, speedMode, 0);
      MotorRunSpeedTimeBlocking(&lift,-30000,3000);
      MotorRunToStall(&lift,-6000);
      MotorRunToStall(&load,-3000);
      
      RunningTask=0;
    }
    if(RunningTask==5){
      // 摩擦轮预热控温：同时控制所有4个摩擦轮
      // 以平均温度判断：低于目标升温，高于目标暂停
      // 退出后不清空 RunningTask，让主循环处理新设的任务值
      double targetTemp = 50;
      double warmSpeed = 1000;
      double holdSpeed = 600;
      
      // 先全速升温
      MotorSetOutput(&fric1, speedMode, -warmSpeed);
      MotorSetOutput(&fric2, speedMode, -warmSpeed);
      MotorSetOutput(&fric3, speedMode, warmSpeed);
      MotorSetOutput(&fric4, speedMode, warmSpeed);
      
      while(RunningTask == 5){
        // 取4个摩擦轮的平均温度作为判断依据
        double avgTemp = (fric1.motorState.tempr + fric2.motorState.tempr +
                         fric3.motorState.tempr + fric4.motorState.tempr) / 4.0;
        
        if(avgTemp > targetTemp + 1){
          // 温度过高，停止
          MotorSetOutput(&fric1, speedMode, 0);
          MotorSetOutput(&fric2, speedMode, 0);
          MotorSetOutput(&fric3, speedMode, 0);
          MotorSetOutput(&fric4, speedMode, 0);
        } else if(avgTemp < targetTemp - 1){
          // 温度偏低，升温
          MotorSetOutput(&fric1, speedMode, -warmSpeed);
          MotorSetOutput(&fric2, speedMode, -warmSpeed);
          MotorSetOutput(&fric3, speedMode, warmSpeed);
          MotorSetOutput(&fric4, speedMode, warmSpeed);
        } else {
          // 达到目标温度，保持
          MotorSetOutput(&fric1, speedMode, -holdSpeed);
          MotorSetOutput(&fric2, speedMode, -holdSpeed);
          MotorSetOutput(&fric3, speedMode, holdSpeed);
          MotorSetOutput(&fric4, speedMode, holdSpeed);
        }
        osDelay(10);
      }
      // 退出控温，摩擦轮归零
      MotorSetOutput(&fric1, speedMode, 0);
      MotorSetOutput(&fric2, speedMode, 0);
      MotorSetOutput(&fric3, speedMode, 0);
      MotorSetOutput(&fric4, speedMode, 0);
    }
    if(RunningTask==6){
      // 全自动模式：按一次Y发两发，再按一次Y再发两发
      // 第1次Y：发射第1、2发
      uint16_t last_launch_time = g_dart_cmd_cache.latest_launch_cmd_time;
      while(1){
        (void)Referee_GetDartClientCmd(&g_referee, &g_dart_cmd_cache);
        if(g_dart_cmd_cache.latest_launch_cmd_time != last_launch_time) break;
        osDelay(10);
      }
      // 第1发
      MotorRunToAngleBlocking(&GM6020, DART_AUTO_YAW_0 + 245000.0, 300);
      MotorSetOutput(&fric1, speedMode, -DART_AUTO_V1_0);
      MotorSetOutput(&fric2, speedMode, -DART_AUTO_V2_0);
      MotorSetOutput(&fric3, speedMode, DART_AUTO_V1_0);
      MotorSetOutput(&fric4, speedMode, DART_AUTO_V2_0);
      MotorRunToStall(&load,-3000);
      MotorRunSpeedTimeBlocking(&lift,30000,2000);

      osDelay(3000);// 等待3秒，确保弹丸发射完成
      // 第2发
      MotorRunToAngleBlocking(&GM6020, DART_AUTO_YAW_1 + 245000.0, 300);
      MotorSetOutput(&fric1, speedMode, -DART_AUTO_V1_1);
      MotorSetOutput(&fric2, speedMode, -DART_AUTO_V2_1);
      MotorSetOutput(&fric3, speedMode, DART_AUTO_V1_1);
      MotorSetOutput(&fric4, speedMode, DART_AUTO_V2_1);
      MotorRunSpeedTimeBlocking(&lift,30000,1700);
      MotorSetOutput(&fric1, speedMode, 0);
      MotorSetOutput(&fric2, speedMode, 0);
      MotorSetOutput(&fric3, speedMode, 0);
      MotorSetOutput(&fric4, speedMode, 0);
      MotorRunSpeedTimeBlocking(&lift,-30000,3000);
      MotorRunToStall(&lift,-6000);//换弹位置
      // 第2次Y：发射第3、4发
      last_launch_time = g_dart_cmd_cache.latest_launch_cmd_time;
      while(1){
        (void)Referee_GetDartClientCmd(&g_referee, &g_dart_cmd_cache);
        if(g_dart_cmd_cache.latest_launch_cmd_time != last_launch_time) break;
        osDelay(10);
      }
      // 第3发
      MotorRunToAngleBlocking(&GM6020, DART_AUTO_YAW_2 + 245000.0, 300);
      MotorSetOutput(&fric1, speedMode, -DART_AUTO_V1_2);
      MotorSetOutput(&fric2, speedMode, -DART_AUTO_V2_2);
      MotorSetOutput(&fric3, speedMode, DART_AUTO_V1_2);
      MotorSetOutput(&fric4, speedMode, DART_AUTO_V2_2);
      MotorRunToStall(&load,3000);
      MotorRunSpeedTimeBlocking(&lift,30000,2000);
      osDelay(3000);// 等待3秒，确保弹丸发射完成
      // 第4发
      MotorRunToAngleBlocking(&GM6020, DART_AUTO_YAW_3 + 245000.0, 300);
      MotorSetOutput(&fric1, speedMode, -DART_AUTO_V1_3);
      MotorSetOutput(&fric2, speedMode, -DART_AUTO_V2_3);
      MotorSetOutput(&fric3, speedMode, DART_AUTO_V1_3);
      MotorSetOutput(&fric4, speedMode, DART_AUTO_V2_3);
      MotorRunSpeedTimeBlocking(&lift,30000,1700);
      MotorSetOutput(&fric1, speedMode, 0);
      MotorSetOutput(&fric2, speedMode, 0);
      MotorSetOutput(&fric3, speedMode, 0);
      MotorSetOutput(&fric4, speedMode, 0);
      MotorRunSpeedTimeBlocking(&lift,-30000,3000);
      MotorRunToStall(&lift,6000);//回到原来位置
      RunningTask=0;
    }
    if(RunningTask==7){
      // 半自动模式：第1发使用预设参数，后续等待裁判系统0x0301数据
      // 等待操作手按Y键（latest_launch_cmd_time变化）后发射第1发
      uint16_t last_launch_time = g_dart_cmd_cache.latest_launch_cmd_time;
      while(1){
        (void)Referee_GetDartClientCmd(&g_referee, &g_dart_cmd_cache);
        if(g_dart_cmd_cache.latest_launch_cmd_time != last_launch_time) break;
        osDelay(10);
      }
      // 第1发：使用半自动预设参数
      DartFireSingle(DART_SEMI_FIRST_YAW, DART_SEMI_FIRST_V1, DART_SEMI_FIRST_V2);
      // 第2~4发：等待裁判系统通过0x0301下发参数，每收到一发立即打出
      for(int shot = 1; shot < 4; ){
        referee_dart_param_item_t param;
        int ret = Referee_GetDartParam(&g_referee, &param);
        if(ret == 0){
          // 收到有效参数，更新dartParam_array并发射
          uint8_t id = param.dart_id;
          if(id > 3) id = 0; // 防越界
          dartParam_array[id].yaw     = param.yaw;
          dartParam_array[id].v1Speed = param.v1Speed;
          dartParam_array[id].v2Speed = param.v2Speed;
          DartFireSingle(dartParam_array[id].yaw,
                         dartParam_array[id].v1Speed,
                         dartParam_array[id].v2Speed);
          shot++;
        }
        // 检查是否中途被切走
        if(RunningTask != 7) goto exit_task7;
        osDelay(5);
      }
      exit_task7:
      RunningTask=0;
    }
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
       switch(m->motorState.motorMode){
         case angleMode:
           PidCalculate(&m->anglePid);
           break;
         case speedMode:
         case runToAngle: // runToAngle 本质上是速度环控制
         case runToStallMode: // runToStall 本质也是速度环
         case speedTimeMode:
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
