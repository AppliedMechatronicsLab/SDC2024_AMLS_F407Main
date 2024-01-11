/*
 Swerve Driver lib for HAL

 Created on 2020/11/20.
    Author: Trung Ngyen
    Email: trungvd0508@gmail.com
################## BY AML ##################
*/

#include "SwerveDriverAML.h"
#include "main.h"

I2C_HandleTypeDef *Swerve_Driver_IIC;

uint8_t Data_2_Driver[3] = {0x55, 0x00, 0x00};

// sub-function

void U16_2_8(int16_t data, uint8_t *data_array)
{
    *(data_array + 1) = (uint8_t)((data & 0xFF00) >> 8);
    *(data_array + 2) = (uint8_t)(data & 0x00FF);
}

// main fucntion

SD_Status Swerve_IIC_Init(I2C_HandleTypeDef *hi2c)
{
    Swerve_Driver_IIC = hi2c;
}

SD_Status Swerve_Module_Rotate(uint8_t Driver_Addr, int16_t Angle)
{
    U16_2_8(Angle, Data_2_Driver);
    HAL_I2C_Master_Transmit(Swerve_Driver_IIC, Driver_Addr << 1, Data_2_Driver, 3, 15);
}

SD_Status Swerve_Module_Scan_Addr(uint8_t *result_arr)
{
    for (uint8_t i = 0; i < 10; i++)
    {
        if (HAL_I2C_IsDeviceReady(Swerve_Driver_IIC, (uint16_t)(i << 1), 3, 100) == HAL_OK)
        {
            *(result_arr + i) = 1;
        }
        else
        {
            *(result_arr + i) = 0;
        }
        i++;
    }
}

SD_Status Swerve_Module_Check_Connection(uint8_t Driver_Addr)
{
    if (HAL_I2C_IsDeviceReady(Swerve_Driver_IIC, Driver_Addr << 1, 3, 100) == HAL_OK)
    {
        return SD_OK;
    }
    else
    {
        return SD_ERROR;
    }
}
