/**
  ******************************************************************************
  * @file    EEPROM_Emulation/inc/eeprom.h 
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    07/27/2009
  * @brief   This file contains all the functions prototypes for the EEPROM 
  *          emulation firmware library.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//Flash虚拟地址宏定义
#define OFFSET_AX_ADDR 0x0000
#define OFFSET_AY_ADDR 0x0001
#define OFFSET_AZ_ADDR 0x0002
#define OFFSET_GX_ADDR 0x0003
#define OFFSET_GY_ADDR 0x0004
#define OFFSET_GZ_ADDR 0x0005

#define HMC5883L_OFFSET_X_ADDR 0x0006
#define HMC5883L_OFFSET_Y_ADDR 0x0007
#define HMC5883L_OFFSET_Z_ADDR 0x0008

#define ROLL_GYRO_KP_ADDR 0x0009
#define ROLL_GYRO_KI_ADDR 0x000a
#define ROLL_GYRO_KD_ADDR 0x000b
#define ROLL_ANGLE_KP_ADDR	0x000c
#define ROLL_ANGLE_KI_ADDR	0x000d
#define ROLL_ANGLE_KD_ADDR	0x000e

#define PITCH_GYRO_KP_ADDR 0x000f
#define PITCH_GYRO_KI_ADDR 0x0010
#define PITCH_GYRO_KD_ADDR 0x0011
#define PITCH_ANGLE_KP_ADDR	0x0012
#define PITCH_ANGLE_KI_ADDR	0x0013
#define PITCH_ANGLE_KD_ADDR	0x0014

#define YAW_GYRO_KP_ADDR 0x0015
#define YAW_GYRO_KI_ADDR 0x0016
#define YAW_GYRO_KD_ADDR 0x0017

/* Variables' number */
#define NumbOfVar               ((uint8_t)0x18)




/* Exported constants --------------------------------------------------------*/
/* Define the STM32F10Xxx Flash page size depending on the used STM32 device */
#if defined (STM32F10X_LD) || defined (STM32F10X_MD)
  #define PAGE_SIZE  (uint16_t)0x400  /* Page size = 1KByte */
#elif defined (STM32F10X_HD) || defined (STM32F10X_CL)
  #define PAGE_SIZE  (uint16_t)0x800  /* Page size = 2KByte */
#endif

/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS    ((uint32_t)0x08010000) /* EEPROM emulation start address:
                                                  after 64KByte of used Flash memory */

/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS      ((uint32_t)(EEPROM_START_ADDRESS + 0x000))
#define PAGE0_END_ADDRESS       ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))

#define PAGE1_BASE_ADDRESS      ((uint32_t)(EEPROM_START_ADDRESS + PAGE_SIZE))
#define PAGE1_END_ADDRESS       ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))

/* Used Flash pages for EEPROM emulation */
#define PAGE0                   ((uint16_t)0x0000)
#define PAGE1                   ((uint16_t)0x0001)

/* No valid page define */
#define NO_VALID_PAGE           ((uint16_t)0x00AB)

/* Page status definitions */
#define ERASED                  ((uint16_t)0xFFFF)     /* PAGE is empty */
#define RECEIVE_DATA            ((uint16_t)0xEEEE)     /* PAGE is marked to receive data */
#define VALID_PAGE              ((uint16_t)0x0000)     /* PAGE containing valid data */

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE    ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE     ((uint8_t)0x01)

/* Page full define */
#define PAGE_FULL               ((uint8_t)0x80)



/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint16_t EE_Init(void);												 //初始化配置
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data);		 //从指定地址读取数据
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);		 //向指定地址写入数据

#endif /* __EEPROM_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
