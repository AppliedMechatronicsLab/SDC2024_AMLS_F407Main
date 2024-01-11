/*
 Swerve Driver lib for HAL

 Created on 2020/11/20.
    Author: Trung Ngyen
    Email: trungvd0508@gmail.com
################## BY AML ##################
*/

#ifndef _SDAML_H_
#define _SDAML_H_

#define SD_1 0x00
#define SD_2 0x01
#define SD_3 0x02
#define SD_4 0x03
#define SD_5 0x04
#define SD_6 0x05
#define SD_7 0x06
#define SD_8 0x07
#define SD_9 0x08

typedef enum
{
    SD_OK = 1,
    SD_ERROR = 0
} SD_Status;

SD_Status Swerve_IIC_Init(I2C_HandleTypeDef *hi2c);
SD_Status Swerve_Module_Rotate(uint8_t Driver_Addr, int16_t Angle);
SD_Status Swerve_Module_Scan_Addr(uint8_t *result_arr);
SD_Status Swerve_Module_Check_Connection(uint8_t Driver_Addr);

#endif // _SDAML_H_