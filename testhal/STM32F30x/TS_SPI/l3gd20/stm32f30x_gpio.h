/**
  ******************************************************************************
  * @file    stm32f30x_gpio.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    04-September-2012
  * @brief   This file contains all the functions prototypes for the GPIO 
  *          firmware library. 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F30x_GPIO_H
#define __STM32F30x_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "pal_lld.h"
/** @addtogroup STM32F30x_StdPeriph_Driver
  * @{
  */

/** @addtogroup GPIO
  * @{
  */

/* Exported types ------------------------------------------------------------*/
 
#define IS_GPIO_ALL_PERIPH(PERIPH) (((PERIPH) == GPIOA) || \
                                    ((PERIPH) == GPIOB) || \
                                    ((PERIPH) == GPIOC) || \
                                    ((PERIPH) == GPIOD) || \
                                    ((PERIPH) == GPIOE) || \
                                    ((PERIPH) == GPIOF))  
                                    
#define IS_GPIO_LIST_PERIPH(PERIPH) (((PERIPH) == GPIOA) || \
                                     ((PERIPH) == GPIOB) || \
                                     ((PERIPH) == GPIOD))  
/** @defgroup Configuration_Mode_enumeration 
  * @{
  */ 
typedef enum
{ 
  GPIO_Mode_IN   = 0x00, /*!< GPIO Input Mode */
  GPIO_Mode_OUT  = 0x01, /*!< GPIO Output Mode */
  GPIO_Mode_AF   = 0x02, /*!< GPIO Alternate function Mode */
  GPIO_Mode_AN   = 0x03  /*!< GPIO Analog Mode */
}GPIOMode_TypeDef;

#define IS_GPIO_MODE(MODE) (((MODE) == GPIO_Mode_IN)|| ((MODE) == GPIO_Mode_OUT) || \
                            ((MODE) == GPIO_Mode_AF)|| ((MODE) == GPIO_Mode_AN))
/**
  * @}
  */
  
/** @defgroup Output_type_enumeration
  * @{
  */ 
typedef enum
{ 
  GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
}GPIOOType_TypeDef;

#define IS_GPIO_OTYPE(OTYPE) (((OTYPE) == GPIO_OType_PP) || ((OTYPE) == GPIO_OType_OD))

/**
  * @}
  */

/** @defgroup Output_Maximum_frequency_enumeration 
  * @{
  */ 
typedef enum
{ 
  GPIO_Speed_2MHz   = 0x01, /*!< Medium Speed */
  GPIO_Speed_10MHz  = 0x02, /*!< Fast Speed   */
  GPIO_Speed_50MHz  = 0x03  /*!< High Speed   */
}GPIOSpeed_TypeDef;

#define IS_GPIO_SPEED(SPEED) (((SPEED) == GPIO_Speed_2MHz) || \
                              ((SPEED) == GPIO_Speed_10MHz)||  ((SPEED) == GPIO_Speed_50MHz))
/**
  * @}
  */  

/** @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */ 
typedef enum
{
  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_TypeDef;

#define IS_GPIO_PUPD(PUPD) (((PUPD) == GPIO_PuPd_NOPULL) || ((PUPD) == GPIO_PuPd_UP) || \
                            ((PUPD) == GPIO_PuPd_DOWN))
/**
  * @}
  */

/** @defgroup Bit_SET_and_Bit_RESET_enumeration
  * @{
  */
typedef enum
{ 
  Bit_RESET = 0,
  Bit_SET
}BitAction;

#define IS_GPIO_BIT_ACTION(ACTION) (((ACTION) == Bit_RESET) || ((ACTION) == Bit_SET))
/**
  * @}
  */

/** 
  * @brief  GPIO Init structure definition  
  */ 
typedef struct
{
  uint32_t GPIO_Pin;              /*!< Specifies the GPIO pins to be configured.
                                       This parameter can be any value of @ref GPIO_pins_define */
                                       
  GPIOMode_TypeDef GPIO_Mode;     /*!< Specifies the operating mode for the selected pins.
                                       This parameter can be a value of @ref GPIOMode_TypeDef   */

  GPIOSpeed_TypeDef GPIO_Speed;   /*!< Specifies the speed for the selected pins.
                                       This parameter can be a value of @ref GPIOSpeed_TypeDef  */

  GPIOOType_TypeDef GPIO_OType;   /*!< Specifies the operating output type for the selected pins.
                                       This parameter can be a value of @ref GPIOOType_TypeDef  */

  GPIOPuPd_TypeDef GPIO_PuPd;     /*!< Specifies the operating Pull-up/Pull down for the selected pins.
                                       This parameter can be a value of @ref GPIOPuPd_TypeDef   */
}GPIO_InitTypeDef;

/* Exported constants --------------------------------------------------------*/


/**
  * @}
  */

/** @defgroup GPIO_Pin_sources 
  * @{
  */ 
#define GPIO_PinSource0            ((uint8_t)0x00)
#define GPIO_PinSource1            ((uint8_t)0x01)
#define GPIO_PinSource2            ((uint8_t)0x02)
#define GPIO_PinSource3            ((uint8_t)0x03)
#define GPIO_PinSource4            ((uint8_t)0x04)
#define GPIO_PinSource5            ((uint8_t)0x05)
#define GPIO_PinSource6            ((uint8_t)0x06)
#define GPIO_PinSource7            ((uint8_t)0x07)
#define GPIO_PinSource8            ((uint8_t)0x08)
#define GPIO_PinSource9            ((uint8_t)0x09)
#define GPIO_PinSource10           ((uint8_t)0x0A)
#define GPIO_PinSource11           ((uint8_t)0x0B)
#define GPIO_PinSource12           ((uint8_t)0x0C)
#define GPIO_PinSource13           ((uint8_t)0x0D)
#define GPIO_PinSource14           ((uint8_t)0x0E)
#define GPIO_PinSource15           ((uint8_t)0x0F)

#define IS_GPIO_PIN_SOURCE(PINSOURCE) (((PINSOURCE) == GPIO_PinSource0) || \
                                       ((PINSOURCE) == GPIO_PinSource1) || \
                                       ((PINSOURCE) == GPIO_PinSource2) || \
                                       ((PINSOURCE) == GPIO_PinSource3) || \
                                       ((PINSOURCE) == GPIO_PinSource4) || \
                                       ((PINSOURCE) == GPIO_PinSource5) || \
                                       ((PINSOURCE) == GPIO_PinSource6) || \
                                       ((PINSOURCE) == GPIO_PinSource7) || \
                                       ((PINSOURCE) == GPIO_PinSource8) || \
                                       ((PINSOURCE) == GPIO_PinSource9) || \
                                       ((PINSOURCE) == GPIO_PinSource10) || \
                                       ((PINSOURCE) == GPIO_PinSource11) || \
                                       ((PINSOURCE) == GPIO_PinSource12) || \
                                       ((PINSOURCE) == GPIO_PinSource13) || \
                                       ((PINSOURCE) == GPIO_PinSource14) || \
                                       ((PINSOURCE) == GPIO_PinSource15))
/**
  * @}
  */

/** @defgroup GPIO_Alternate_function_selection_define 
  * @{
  */

/** 
  * @brief  AF 0 selection
  */ 
#define GPIO_AF_0            ((uint8_t)0x00) /* JTCK-SWCLK, JTDI, JTDO/TRACESW0, JTMS-SWDAT,  
                                                MCO, NJTRST, TRACED, TRACECK */
/** 
  * @brief  AF 1 selection
  */ 
#define GPIO_AF_1            ((uint8_t)0x01) /*  OUT, TIM2, TIM15, TIM16, TIM17 */

/** 
  * @brief  AF 2 selection
  */ 
#define GPIO_AF_2            ((uint8_t)0x02) /* COMP1_OUT, TIM1, TIM2, TIM3, TIM4, TIM8, TIM15 */

/** 
  * @brief  AF 3 selection
  */ 
#define GPIO_AF_3            ((uint8_t)0x03) /* COMP7_OUT, TIM8, TIM15, Touch */

/** 
  * @brief  AF 4 selection
  */ 
#define GPIO_AF_4            ((uint8_t)0x04) /* I2C1, I2C2, TIM1, TIM8, TIM16, TIM17 */

/** 
  * @brief  AF 5 selection
  */ 
#define GPIO_AF_5            ((uint8_t)0x05) /* IR_OUT, I2S2, I2S3, SPI1, SPI2, TIM8, USART4, USART5 */

/** 
  * @brief  AF 6 selection
  */ 
#define GPIO_AF_6            ((uint8_t)0x06) /*  IR_OUT, I2S2, I2S3, SPI2, SPI3, TIM1, TIM8 */

/** 
  * @brief  AF 7 selection
  */ 
#define GPIO_AF_7            ((uint8_t)0x07) /* AOP2_OUT, CAN, COMP3_OUT, COMP5_OUT, COMP6_OUT, 
                                                USART1, USART2, USART3 */

/** 
  * @brief  AF 8 selection
  */ 
#define GPIO_AF_8            ((uint8_t)0x08) /* COMP1_OUT, COMP2_OUT, COMP3_OUT, COMP4_OUT, 
                                                COMP5_OUT, COMP6_OUT */

/** 
  * @brief  AF 9 selection
  */ 
#define GPIO_AF_9            ((uint8_t)0x09) /* AOP4_OUT, CAN, TIM1, TIM8, TIM15 */

/** 
  * @brief  AF 10 selection
  */ 
#define GPIO_AF_10            ((uint8_t)0x0A) /* AOP1_OUT, AOP3_OUT, TIM2, TIM3, TIM4, TIM8, TIM17 */

/** 
  * @brief  AF 11 selection
  */ 
#define GPIO_AF_11            ((uint8_t)0x0B) /* TIM1, TIM8 */

/** 
   * @brief  AF 12 selection
   */ 
#define GPIO_AF_12            ((uint8_t)0x0E) /* TIM1 */

/** 
  * @brief  AF 14 selection
  */ 
#define GPIO_AF_14            ((uint8_t)0x0E) /* USBDM, USBDP */

/** 
  * @brief  AF 15 selection
  */ 
#define GPIO_AF_15            ((uint8_t)0x0F) /* OUT */

#define IS_GPIO_AF(AF)   (((AF) == GPIO_AF_0)||((AF) == GPIO_AF_1)||\
                          ((AF) == GPIO_AF_2)||((AF) == GPIO_AF_3)||\
                          ((AF) == GPIO_AF_4)||((AF) == GPIO_AF_5)||\
                          ((AF) == GPIO_AF_6)||((AF) == GPIO_AF_7)||\
                          ((AF) == GPIO_AF_8)||((AF) == GPIO_AF_9)||\
                          ((AF) == GPIO_AF_10)||((AF) == GPIO_AF_11)||\
                          ((AF) == GPIO_AF_14)||((AF) == GPIO_AF_15))

/**
  * @}
  */

/**
 * @brief   STM32 GPIO registers block.
 */
/* 
typedef struct {

  volatile uint32_t     MODER;
  volatile uint32_t     OTYPER;
  volatile uint32_t     OSPEEDR;
  volatile uint32_t     PUPDR;
  volatile uint32_t     IDR;
  volatile uint32_t     ODR;
  volatile union {
    uint32_t            W;
    struct {
      uint16_t          set;
      uint16_t          clear;
    } H;
  } BSRR;
  volatile uint32_t     LCKR;
  volatile uint32_t     AFRL;
  volatile uint32_t     AFRH;
} GPIO_TypeDef;
*/


/**
  * @}
  */ 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/* Function used to set the GPIO configuration to the default reset state *****/
void GPIO_DeInit(GPIO_TypeDef* GPIOx);

/* Initialization and Configuration functions *********************************/
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/* GPIO Read and Write functions **********************************************/
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);

/* GPIO Alternate functions configuration functions ***************************/
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F30x_GPIO_H */
/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
