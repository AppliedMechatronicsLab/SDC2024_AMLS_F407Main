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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "pid.h"
#include "math.h"
#include "stdint.h"
#include "stdio.h"
#include "PS2.h"
#include "BUTTON.h"
#include "pca9685.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PID_TypeDef RPID;
HAL_StatusTypeDef status;
#define Get_Red_Delay 500
#define Get_Green_Delay 800
#define Get_Ball_Up 1
#define Get_Ball_Down 2
#define Get_Ball_Stop 0
// pca_servo_st sv;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t PWM_Freq = 5000;
uint8_t i2c_arr[128];
uint8_t PCA_State = 0;

float Speed_Set = 40;

// variables for MPU data
uint8_t Buffer_Arr[7];
uint8_t Buffer;
bool check = 1;
float theta = 0;
float temp = 0;
uint8_t buff_index = 0;
uint8_t Reset_Angle_Byte[3] = {0xFF, 0xAA, 0x52};
uint8_t RST_ANGLE = 0;
uint8_t cw_flag = 0;
uint8_t ccw_flag = 0;
uint8_t tempspeed;

float Prev_Angle = 0;
float Angle_Raw = 0;
float Angle_Driff = 0;

uint8_t Re_Buff[11];
uint8_t Re_Buff_Index = 0;
uint8_t Re_Buff_Check = 0;

// variables for I2C
uint16_t SWERVE_DRIVE_1_ADDR = 0x08 << 1;
uint16_t SWERVE_DRIVE_2_ADDR = 0x14 << 1;
uint16_t Angle_2_Send = 0;
float temp_RX, temp_RY;
uint8_t init_check = 1;

// variables for PID
double Angle_Input, PIDOut, TempSetpoint;
uint32_t period = 0;
uint8_t Gygro_Move = 0;
// float Kp, Ki, Kd;
int8_t Out_Max, Out_Min;

// variables for joystick
float Speed = 0;
float Speed_L, Speed_R;
float gain = 0;
PS2Buttons PS2;
uint8_t Field_Control_Check = 0;
uint8_t Last_R1_Button_State = 0;
uint8_t R1_Button_State = 0;

uint8_t Rotate_90_Flag = 0;
uint8_t C_Button_Sate = 0;
uint8_t Last_C_Button_State = 0;

uint8_t Rotate_n90_Flag = 0;
uint8_t Sq_Button_Sate = 0;
uint8_t Last_Sq_Button_State = 0;

uint8_t Rotate_180_Flag = 0;
uint8_t X_Button_Sate = 0;
uint8_t Last_X_Button_State = 0;

uint8_t Get_Red_Flag = 0;
uint8_t Right_Button_State = 0;
uint8_t Last_Right_Button_State = 0;

uint8_t Get_Yellow_Flag = 0;
uint8_t Foward_Button_State = 0;
uint8_t Last_Foward_Button_State = 0;

uint8_t Get_Green_Flag = 0;
uint8_t Left_Button_State = 0;
uint8_t Last_Left_Button_State = 0;

uint8_t Get_Gift_Flag = 0;
uint8_t Select_Button_State = 0;
uint8_t Last_Select_Button_State = 0;

uint8_t Servo_Arm_P1 = 0;
uint8_t Servo_Arm_P2 = 0;
uint8_t L1_Button_State = 0;
uint8_t Last_L1_Button_State = 0;

uint8_t Up_Button_State = 0;
uint8_t Last_Up_Button_State = 0;

uint8_t Down_Button_State = 0;
uint8_t Last_Down_Button_State = 0;

uint8_t Vibration_Deg = 10;

uint8_t One_Time_Flag = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (float)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void Div16_To_8(int16_t data, uint8_t *data_array)
{
  *(data_array + 1) = (uint8_t)((data & 0xFF00) >> 8);
  *(data_array + 2) = (uint8_t)(data & 0x00FF);
}

void Buzzer_Beep(uint8_t times, uint8_t duration)
{
  for (uint8_t i = 0; i < times; i++)
  {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    HAL_Delay(duration);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_Delay(duration);
  }
}

void MPU_Data_Merge()
{
  Prev_Angle = Angle_Raw;
  Angle_Raw = (float)((Re_Buff[7] << 8 | Re_Buff[6]) / 32768.0 * 180);
  if (Angle_Raw - Prev_Angle > 250)
  {
    Angle_Driff += 360;
  }
  else if (Angle_Raw - Prev_Angle < -250)
  {
    Angle_Driff -= 360;
  }
  theta = Angle_Raw - Angle_Driff;
}

// 0 to disable PWM
void Motor_PWM(uint8_t channel, uint8_t duty)
{
  period = (SystemCoreClock / 10000) - 1;
  htim8.Instance->ARR = period;

  switch (channel)
  {
  case 1:
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    htim8.Instance->CCR1 = (period * duty) / 100;
    break;
  case 2:
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    htim8.Instance->CCR2 = (period * duty) / 100;
    break;
  case 3:
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    htim8.Instance->CCR3 = (period * duty) / 100;
    break;
  case 4:
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    htim8.Instance->CCR4 = (period * duty) / 100;
    break;
  case 0:
    htim8.Instance->CCR1 = 0;
    htim8.Instance->CCR2 = 0;
    htim8.Instance->CCR3 = 0;
    break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UNUSED(huart);
  if (huart->Instance == UART4)
  {
    Re_Buff[Re_Buff_Index] = Buffer;
    if (Re_Buff_Index == 0 && Re_Buff[0] != 0x55)
    {
      HAL_UART_Receive_IT(&huart4, &Buffer, 1);
    }
    else
    {
      Re_Buff_Index++;
    }

    if (Re_Buff_Index == 11)
    {
      if (Re_Buff[1] == 0x53)
      {
        MPU_Data_Merge();
      }
      Re_Buff_Index = 0;
    }
    HAL_UART_Receive_IT(&huart4, &Buffer, 1);
  }
}

void Swerve_Module_Send(int16_t U12_Input, uint8_t module)
{
  uint8_t I2C_Tx_Buffer[] = {0x05, 0x00, 0x00};
  Div16_To_8(U12_Input, I2C_Tx_Buffer);
  switch (module)
  {
  case 1:
    HAL_I2C_Master_Transmit(&hi2c1, 0x01 << 1, I2C_Tx_Buffer, 3, 15);
    break;
  case 2:
    HAL_I2C_Master_Transmit(&hi2c1, 0x02 << 1, I2C_Tx_Buffer, 3, 15);
    break;
  case 0:
    HAL_I2C_Master_Transmit(&hi2c1, 0x01 << 1, I2C_Tx_Buffer, 2, 15);
    HAL_I2C_Master_Transmit(&hi2c1, 0x02 << 1, I2C_Tx_Buffer, 2, 15);
    break;
  default:
    break;
  }
}

void PID_Set(float Kp, float Ki, float Kd)
{
  PID(&RPID, &Angle_Input, &PIDOut, &TempSetpoint, Kp, Ki, Kd, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&RPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&RPID, 1);
  PID_SetOutputLimits(&RPID, -40, 40);
}

void Get_Ball(int state)
{
  switch (state)
  {
  case 2:
    HAL_GPIO_WritePin(M3_EN_GPIO_Port, M3_EN_Pin, 1);
    Motor_PWM(3, 70);
    Motor_PWM(4, 100);
    break;
  case 1:
    HAL_GPIO_WritePin(M3_EN_GPIO_Port, M3_EN_Pin, 1);
    Motor_PWM(3, 100);
    Motor_PWM(4, 70);
    break;
  case 0:
    Motor_PWM(3, 100);
    Motor_PWM(4, 100);
    HAL_GPIO_WritePin(M3_EN_GPIO_Port, M3_EN_Pin, 0);
    break;
  }
}
/* USER CODE END 0 */


/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset
  of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  PCA9685_Init(&hi2c1);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
  PS2_Init(&htim2, &PS2);

  // function to scan i2c address

  for (uint8_t i = 0; i < 128; i++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 100) == HAL_OK)
    {
      i2c_arr[i] = 1;
    }
    else
    {
      i2c_arr[i] = 0;
    }
  }

  PID_Set(1.5, 0.8, 0.005);
  Angle_Input = 0;
  HAL_UART_Receive_IT(&huart4, &Buffer, 1);

  for (uint8_t i = 0; i < 5; i++)
  {
    Swerve_Module_Send(0, 0);
    HAL_Delay(10);
  }

  if (status == HAL_OK)
  {
    PCA_State = 1;
  }
  else
  {
    PCA_State = 0;
  }

  MX_TIM8_Init();
  while (init_check)
  {
    uint8_t Status_B9 = 0;
    uint8_t Status_B8 = 0;
    PS2_Update();
    Swerve_Module_Send(180, 0);
    HAL_GPIO_TogglePin(USL3_GPIO_Port, USL3_Pin);
    HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
    HAL_Delay(500);
    Status_B9 = HAL_GPIO_ReadPin(BUTTON_9_GPIO_Port, BUTTON_9_Pin);
    Status_B8 = HAL_GPIO_ReadPin(BUTTON_8_GPIO_Port, BUTTON_8_Pin);
    while (Status_B9 == 0)
    {
      HAL_GPIO_WritePin(USL4_GPIO_Port, USL4_Pin, 1);
      Buzzer_Beep(3, 500);
      HAL_Delay(500);
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
      for (int16_t i = 0; i >= 180; i++)
      {
        Swerve_Module_Send(i, 1);
        Swerve_Module_Send(-i, 2);
        HAL_Delay(8);
      }
      HAL_Delay(10);
      for (int16_t i = 180; i >= 0; i--)
      {
        Swerve_Module_Send(i, 1);
        Swerve_Module_Send(-i, 2);
        HAL_Delay(8);
      }
      Swerve_Module_Send(0, 0);
      HAL_Delay(10);
      for (int16_t i = 0; i >= 180; i++)
      {
        Swerve_Module_Send(i, 2);
        Swerve_Module_Send(-i, 1);
        HAL_Delay(8);
      }
      HAL_Delay(10);
      for (int16_t i = 180; i >= 0; i--)
      {
        Swerve_Module_Send(i, 2);
        Swerve_Module_Send(-i, 1);
        HAL_Delay(8);
      }
      Swerve_Module_Send(0, 0);
      Buzzer_Beep(2, 50);

      Motor_PWM(1, 15);
      Motor_PWM(2, 15);
      HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 1);
      HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 1);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 0);
      HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 0);
      HAL_Delay(1000);
      Buzzer_Beep(2, 100);
      Motor_PWM(1, 0);
      Motor_PWM(2, 0);
      Swerve_Module_Send(90, 1);
      Swerve_Module_Send(90, 2);
      HAL_Delay(1000);
      Motor_PWM(1, 15);
      Motor_PWM(2, 15);
      HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 1);
      HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 1);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 0);
      HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 0);
      HAL_Delay(1000);
      Motor_PWM(1, 0);
      Motor_PWM(2, 0);

      HAL_Delay(600);
      PCA9685_SetServoAngle(8, 115);
      PCA9685_SetServoAngle(9, 19);
      HAL_Delay(1500);
      PCA9685_SetServoAngle(8, 0);
      PCA9685_SetServoAngle(9, 150);
      PCA9685_SetServoAngle(10, 50);
      HAL_Delay(1000);
      PCA9685_SetServoAngle(10, 0);

      HAL_GPIO_WritePin(USL4_GPIO_Port, USL4_Pin, 0);
      Swerve_Module_Send(0, 2);
      Swerve_Module_Send(0, 1);
      Buzzer_Beep(5, 50);
      Status_B9 = 1;
    }

    if (Status_B8 == 0)
    {
      Speed_Set = 50;
      for (uint8_t i = 0; i < 6; i++)
      {
        HAL_GPIO_TogglePin(USL4_GPIO_Port, USL4_Pin);
        HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
        HAL_Delay(300);
      }
      HAL_GPIO_WritePin(USL4_GPIO_Port, USL4_Pin, 1);
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
    }

    if (PS2.L2)
    {
      init_check = 0;
      HAL_GPIO_WritePin(USL3_GPIO_Port, USL3_Pin, 0);
      HAL_UART_Transmit(&huart4, Reset_Angle_Byte, sizeof(Reset_Angle_Byte), 10);
      Buzzer_Beep(1, 100);
      break;
    }
    else
    {
      init_check = 1;
    }
  }
  Buzzer_Beep(2, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    PS2_Update();
    temp_RX = map(PS2.RX, 0, 255, -100, 100);
    Speed = map(PS2.LY, 0, 255, -Speed_Set, Speed_Set);

    L1_Button_State = PS2.L1;
    if (L1_Button_State != Last_L1_Button_State)
    {
      Last_L1_Button_State = L1_Button_State;
      if (L1_Button_State == 0)
      {
        Servo_Arm_P1 = (Servo_Arm_P1 == 1) ? 0 : 1;
      }
    }

    Up_Button_State = PS2.UP;
    if (Up_Button_State != Last_Up_Button_State)
    {
      Last_Up_Button_State = Up_Button_State;
      if (Up_Button_State == 0)
      {
        Servo_Arm_P2 = (Servo_Arm_P2 == 1) ? 0 : 1;
      }
    }

    R1_Button_State = PS2.R1;
    if (R1_Button_State != Last_R1_Button_State)
    {
      Last_R1_Button_State = R1_Button_State;
      if (R1_Button_State == 0)
      {
        Field_Control_Check = (Field_Control_Check == 1) ? 0 : 1;
      }
    }

    C_Button_Sate = PS2.CIRCLE;
    if (C_Button_Sate != Last_C_Button_State)
    {
      Last_C_Button_State = C_Button_Sate;
      if (C_Button_Sate == 0)
      {
        Rotate_90_Flag = (Rotate_90_Flag == 1) ? 0 : 1;
      }
    }

    Sq_Button_Sate = PS2.SQUARE;
    if (Sq_Button_Sate != Last_Sq_Button_State)
    {
      Last_Sq_Button_State = Sq_Button_Sate;
      if (Sq_Button_Sate == 0)
      {
        Rotate_n90_Flag = (Rotate_n90_Flag == 1) ? 0 : 1;
      }
    }

    X_Button_Sate = PS2.CROSS;
    if (X_Button_Sate != Last_X_Button_State)
    {
      Last_X_Button_State = X_Button_Sate;
      if (X_Button_Sate == 0)
      {
        Rotate_180_Flag = (Rotate_180_Flag == 1) ? 0 : 1;
      }
    }

    Right_Button_State = PS2.RIGHT;
    if (Right_Button_State != Last_Right_Button_State)
    {
      Last_Right_Button_State = Right_Button_State;
      if (Right_Button_State == 0)
      {
        Get_Red_Flag = (Get_Red_Flag == 1) ? 0 : 1;
      }
    }

    Foward_Button_State = PS2.UP;
    if (Foward_Button_State != Last_Foward_Button_State)
    {
      Last_Foward_Button_State = Foward_Button_State;
      if (Foward_Button_State == 0)
      {
        Get_Yellow_Flag = (Get_Yellow_Flag == 1) ? 0 : 1;
      }
    }

    Left_Button_State = PS2.LEFT;
    if (Left_Button_State != Last_Left_Button_State)
    {
      Last_Left_Button_State = Left_Button_State;
      if (Left_Button_State == 0)
      {
        Get_Green_Flag = (Get_Green_Flag == 1) ? 0 : 1;
      }
    }

    Select_Button_State = PS2.TRIANGLE;
    if (Select_Button_State != Last_Select_Button_State)
    {
      Last_Select_Button_State = Select_Button_State;
      if (Select_Button_State == 0)
      {
        Get_Gift_Flag = (Get_Gift_Flag == 1) ? 0 : 1;
      }
    }

    if (Servo_Arm_P1 || Servo_Arm_P2)
    {
      if (Servo_Arm_P1)
      {
        if (PS2.L2)
        {
          PCA9685_SetServoAngle(13, Vibration_Deg);
          PCA9685_SetServoAngle(11, 0);
          PCA9685_SetServoAngle(12, 180);
          if (Vibration_Deg == 10)
          {
            Vibration_Deg = 30;
          } else if (Vibration_Deg == 30)
          {
            Vibration_Deg = 10;
          }
        }
        else
        {
          PCA9685_SetServoAngle(13, 50);
          PCA9685_SetServoAngle(11, 30);
          PCA9685_SetServoAngle(12, 150);
        }
      }
      else
      {
        PCA9685_SetServoAngle(13, 135);
        PCA9685_SetServoAngle(11, 10);
        PCA9685_SetServoAngle(12, 170);
      }
    }
    else
    {
      PCA9685_SetServoAngle(13, 40);
      PCA9685_SetServoAngle(11, 110);
      PCA9685_SetServoAngle(12, 70);
    }

    // if (PS2.TRIANGLE)
    // {
    //   HAL_UART_Transmit(&huart4, Reset_Angle_Byte, sizeof(Reset_Angle_Byte), 10);
    //   Prev_Angle = 0;
    //   Angle_Raw = 0;
    //   Angle_Driff = 0;
    //   theta = 0;
    //   Buzzer_Beep(1, 100);
    //   MX_TIM8_Init();
    // }

    if (Get_Gift_Flag)
    {
      PCA9685_SetServoAngle(8, 118);
      PCA9685_SetServoAngle(9, 25);
      PCA9685_SetServoAngle(10, 150);
    }
    else
    {
      PCA9685_SetServoAngle(8, 0);
      PCA9685_SetServoAngle(9, 155);
      PCA9685_SetServoAngle(10, 0);
    }

    // if (Get_Red_Flag)
    // {
    //   Buzzer_Beep(2, 80);
    //   Swerve_Module_Send(90, 1);
    //   Swerve_Module_Send(90, 2);
    //   HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 0);
    //   HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 0);
    //   for (uint8_t i = 0; i < 50; i++)
    //   {
    //     Motor_PWM(1, i);
    //     Motor_PWM(2, i);
    //     HAL_Delay(10);
    //   }
    //   Motor_PWM(1, 50);
    //   Motor_PWM(2, 50);
    //   HAL_Delay(Get_Red_Delay);
    //   for (uint8_t i = 50; i > 0; i--)
    //   {
    //     Motor_PWM(1, i);
    //     Motor_PWM(2, i);
    //     HAL_Delay(10);
    //   }
    //   Motor_PWM(1, 0);
    //   Motor_PWM(2, 0);
    //   Buzzer_Beep(3, 40);
    //   Get_Red_Flag = 0;
    //   Field_Control_Check = 1;
    // }

    // if (Get_Green_Flag)
    // {
    //   Buzzer_Beep(3, 60);
    //   Swerve_Module_Send(70, 1);
    //   Swerve_Module_Send(70, 2);
    //   HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 0);
    //   HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 0);
    //   for (uint8_t i = 0; i < 50; i++)
    //   {
    //     Motor_PWM(1, i);
    //     Motor_PWM(2, i);
    //     HAL_Delay(10);
    //   }
    //   Motor_PWM(1, 50);
    //   Motor_PWM(2, 50);
    //   HAL_Delay(Get_Green_Delay);
    //   for (uint8_t i = 50; i > 0; i--)
    //   {
    //     Motor_PWM(1, i);
    //     Motor_PWM(2, i);
    //     HAL_Delay(10);
    //   }
    //   Motor_PWM(1, 0);
    //   Motor_PWM(2, 0);
    //   Buzzer_Beep(3, 40);
    //   Get_Green_Flag = 0;
    //   Field_Control_Check = 1;
    // }

    while (Rotate_90_Flag)
    {
      Swerve_Module_Send(90, 1);
      Swerve_Module_Send(-90, 2);
      while (One_Time_Flag)
      {
        HAL_UART_Transmit(&huart4, Reset_Angle_Byte, sizeof(Reset_Angle_Byte), 10);
        HAL_UART_Transmit(&huart4, Reset_Angle_Byte, sizeof(Reset_Angle_Byte), 10);
        Prev_Angle = 0;
        Angle_Raw = 0;
        Angle_Driff = 0;
        theta = 0;
        TempSetpoint = -80;
        MX_TIM8_Init();
        One_Time_Flag = 0;
      }
      Angle_Input = (double)theta;
      PID_Compute(&RPID);
      if (PIDOut > 0)
      {
        HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 0);
        HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 0);
      }
      else
      {
        HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 1);
        HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 1);
        PIDOut = -PIDOut;
      }
      Motor_PWM(1, (uint8_t)(PIDOut));
      Motor_PWM(2, (uint8_t)(PIDOut));
      if (fabs(TempSetpoint - theta) < 1)
      {
        Prev_Angle = 0;
        Angle_Raw = 0;
        Angle_Driff = 0;
        theta = 0;
        HAL_UART_Transmit(&huart4, Reset_Angle_Byte, sizeof(Reset_Angle_Byte), 10);
        HAL_UART_Transmit(&huart4, Reset_Angle_Byte, sizeof(Reset_Angle_Byte), 10);
        MX_TIM8_Init();
        Motor_PWM(1, 0);
        Motor_PWM(2, 0);
        Rotate_90_Flag = 0;
        break;
      }
    }

    while (Rotate_n90_Flag)
    {
      Swerve_Module_Send(90, 1);
      Swerve_Module_Send(-90, 2);
      while (One_Time_Flag)
      {
        HAL_UART_Transmit(&huart4, Reset_Angle_Byte, sizeof(Reset_Angle_Byte), 10);
        HAL_UART_Transmit(&huart4, Reset_Angle_Byte, sizeof(Reset_Angle_Byte), 10);
        Prev_Angle = 0;
        Angle_Raw = 0;
        Angle_Driff = 0;
        theta = 0;
        TempSetpoint = 80;
        MX_TIM8_Init();
        One_Time_Flag = 0;
      }
      Angle_Input = (double)theta;
      PID_Compute(&RPID);
      if (PIDOut > 0)
      {
        HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 0);
        HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 0);
      }
      else
      {
        HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 1);
        HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 1);
        PIDOut = -PIDOut;
      }
      Motor_PWM(1, (uint8_t)(PIDOut));
      Motor_PWM(2, (uint8_t)(PIDOut));
      if (fabs(TempSetpoint - theta) < 1)
      {
        Prev_Angle = 0;
        Angle_Raw = 0;
        Angle_Driff = 0;
        theta = 0;
        HAL_UART_Transmit(&huart4, Reset_Angle_Byte, sizeof(Reset_Angle_Byte), 10);
        HAL_UART_Transmit(&huart4, Reset_Angle_Byte, sizeof(Reset_Angle_Byte), 10);
        MX_TIM8_Init();
        Motor_PWM(1, 0);
        Motor_PWM(2, 0);
        Rotate_n90_Flag = 0;
        break;
      }
    }

    while (Rotate_180_Flag)
    {
      Swerve_Module_Send(90, 1);
      Swerve_Module_Send(-90, 2);
      while (One_Time_Flag)
      {
        HAL_UART_Transmit(&huart4, Reset_Angle_Byte, sizeof(Reset_Angle_Byte), 10);
        HAL_UART_Transmit(&huart4, Reset_Angle_Byte, sizeof(Reset_Angle_Byte), 10);
        Prev_Angle = 0;
        Angle_Raw = 0;
        Angle_Driff = 0;
        theta = 0;
        TempSetpoint = 155;
        MX_TIM8_Init();
        One_Time_Flag = 0;
      }
      Angle_Input = (double)theta;
      PID_Compute(&RPID);
      if (PIDOut > 0)
      {
        HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 0);
        HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 0);
      }
      else
      {
        HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 1);
        HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 1);
        PIDOut = -PIDOut;
      }
      Motor_PWM(1, (uint8_t)(PIDOut));
      Motor_PWM(2, (uint8_t)(PIDOut));
      if (fabs(TempSetpoint - theta) < 1)
      {
        Prev_Angle = 0;
        Angle_Raw = 0;
        Angle_Driff = 0;
        theta = 0;
        HAL_UART_Transmit(&huart4, Reset_Angle_Byte, sizeof(Reset_Angle_Byte), 10);
        HAL_UART_Transmit(&huart4, Reset_Angle_Byte, sizeof(Reset_Angle_Byte), 10);
        MX_TIM8_Init();
        Motor_PWM(1, 0);
        Motor_PWM(2, 0);
        Rotate_180_Flag = 0;
        break;
      }
    }

    if (PS2.R2)
    {
      Speed = map(PS2.LY, 0, 255, -95, 95);
    }

    if (Speed > 0)
    {
      HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 1);
      HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 1);
    }
    else
    {
      HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 0);
      HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 0);
      Speed = -Speed;
    }

    if (Field_Control_Check)
    {
      gain = map(temp_RX, -100, 100, -90, 90);
      gain = gain + 90;
      Swerve_Module_Send(gain, 1);
      Swerve_Module_Send(gain, 2);
      Speed = map(PS2.LY, 0, 255, -30, 30);
      // if (gain > 0)
      // {
      //   Motor_PWM(2, (uint8_t)(Speed));
      //   Motor_PWM(1, (uint8_t)(Speed));
      // }
      // else if (gain < 0)
      // {
      //   Motor_PWM(1, (uint8_t)(Speed));
      //   Motor_PWM(2, (uint8_t)(Speed));
      // }
    }
    else
    {
      Swerve_Module_Send((int16_t)roundf(map(temp_RX, -100, 100, 90.0, -90.0)), 1);
      Swerve_Module_Send((int16_t)roundf(map(temp_RX, -100, 100, -90.0, 90.0)), 2);
    }
    Motor_PWM(1, (uint8_t)fabs(Speed));
    Motor_PWM(2, (uint8_t)fabs(Speed));
    One_Time_Flag = 1;
    HAL_Delay(7);
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
