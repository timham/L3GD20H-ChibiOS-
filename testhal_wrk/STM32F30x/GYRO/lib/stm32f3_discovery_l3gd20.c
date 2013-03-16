/**
  ******************************************************************************
  * @file    stm32f3_discovery_l3gd20.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   This file provides a set of functions needed to manage the l3gd20
  *          	MEMS accelerometer available on STM32F3-Discovery Kit.
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f3_discovery_l3gd20.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_spi.h"

/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup STM32F3_DISCOVERY
  * @{
  */ 

/** @addtogroup STM32F3_DISCOVERY_L3GD20
  * @{
  */


/** @defgroup STM32F3_DISCOVERY_L3GD20_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_L3GD20_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_L3GD20_Private_Macros
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup STM32F3_DISCOVERY_L3GD20_Private_Variables
  * @{
  */ 
__IO uint32_t  L3GD20Timeout = L3GD20_FLAG_TIMEOUT;  
/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_L3GD20_Private_FunctionPrototypes
  * @{
  */
//static uint8_t L3GD20_SendByte(uint8_t byte);
static void L3GD20_LowLevel_Init(void);
/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_L3GD20_Private_Functions
  * @{
  */

/**
  * @brief  Set L3GD20 Initialization.
  * @param  L3GD20_InitStruct: pointer to a L3GD20_InitTypeDef structure 
  *         that contains the configuration setting for the L3GD20.
  * @retval None
  */

#define LOW_LEVEL_STEP

#define LOW_LEVEL_STEP1
#undef  LOW_LEVEL_STEP

static void L3GD20_LowLevel_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Enable the SPI periph */
  RCC_APB2PeriphClockCmd(L3GD20_SPI_CLK, ENABLE);

  /* Enable SCK, MOSI and MISO GPIO clocks */
  RCC_AHBPeriphClockCmd(L3GD20_SPI_SCK_GPIO_CLK | L3GD20_SPI_MISO_GPIO_CLK | L3GD20_SPI_MOSI_GPIO_CLK, ENABLE);

  /* Enable CS  GPIO clock */
  RCC_AHBPeriphClockCmd(L3GD20_SPI_CS_GPIO_CLK, ENABLE);
  
  /* Enable INT1 GPIO clock */
  RCC_AHBPeriphClockCmd(L3GD20_SPI_INT1_GPIO_CLK, ENABLE);
  
  /* Enable INT2 GPIO clock */
  RCC_AHBPeriphClockCmd(L3GD20_SPI_INT2_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(L3GD20_SPI_SCK_GPIO_PORT, L3GD20_SPI_SCK_SOURCE, L3GD20_SPI_SCK_AF);
  GPIO_PinAFConfig(L3GD20_SPI_MISO_GPIO_PORT, L3GD20_SPI_MISO_SOURCE, L3GD20_SPI_MISO_AF);
  GPIO_PinAFConfig(L3GD20_SPI_MOSI_GPIO_PORT, L3GD20_SPI_MOSI_SOURCE, L3GD20_SPI_MOSI_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_SCK_PIN;
  GPIO_Init(L3GD20_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  L3GD20_SPI_MOSI_PIN;
  GPIO_Init(L3GD20_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_MISO_PIN;
  GPIO_Init(L3GD20_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(L3GD20_SPI);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(L3GD20_SPI, &SPI_InitStructure);

  /* Configure the RX FIFO Threshold */
  SPI_RxFIFOThresholdConfig(L3GD20_SPI, SPI_RxFIFOThreshold_QF);
  /* Enable SPI1  */
  SPI_Cmd(L3GD20_SPI, ENABLE);

  /* Configure GPIO PIN for Lis Chip select */
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(L3GD20_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  GPIO_SetBits(L3GD20_SPI_CS_GPIO_PORT, L3GD20_SPI_CS_PIN);
  
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_INT1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(L3GD20_SPI_INT1_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_INT2_PIN;
  GPIO_Init(L3GD20_SPI_INT2_GPIO_PORT, &GPIO_InitStructure);
}  


/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */ 

/**
  * @}
  */ 
  

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/     
