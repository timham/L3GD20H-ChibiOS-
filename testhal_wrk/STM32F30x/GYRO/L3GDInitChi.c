/******************************************************************
Author 		: Tim Taisik Ham

Objective 	: 
	- In order to write L3GD20 driver, it was rewritten according to follow sequence
	  ST driver sequence.
		
******************************************************************/
#include  "ch.h"
#include  "hal.h"
#include  "board.h"
#include  "stm32f30x.h"
#include  "stm32_rcc.h"

//#include  "stm32f3_discovery_l3gd20.h"

#define SPI_MODE_MASTER              ((uint16_t)0x0104)
#define SPI_MODE_SLAVE               ((uint16_t)0x0000)

#define CR1_CLEAR_MASK       	     ((uint16_t)0x3040)

#define SPI_I2S_FLAG_RXNE            ((uint16_t)0x0001)
#define SPI_I2S_FLAG_TXE             ((uint16_t)0x0002)

#define  ALTERNATE_SHIFT   		7
#define  PUDR_SHIFT 			5
#define	 OSPEED_SHIFT			3
#define  OTYPE_SHIFT			2
#define	 MODE_SHIFT			0

#define  READWRITE_CMD		     ((uint8_t)0x80)
#define  MULTIPLEBYTE_CMD	     ((uint8_t)0x40)
#define  DUMMY_BYTE		     ((uint8_t)0x00)

#define  L3GD20_CS_LOW()	     pal_lld_clearport(GPIOE, 0x08) 
#define  L3GD20_CS_HIGH()	     pal_lld_setport(GPIOE, 0x08) 

#define  L3GD20_FLAG_TIMEOUT	     ((uint32_t)0x1000)

/*****************************************************************************
  TypeDef   
*****************************************************************************/

typedef struct
{
  uint16_t 	SPI_Direction;           
  uint16_t 	SPI_Mode;
  uint16_t 	SPI_DataSize;
  uint16_t 	SPI_CPOL; 
  uint16_t 	SPI_CPHA;    
  uint16_t 	SPI_NSS; 
  uint16_t 	SPI_BaudRatePrescaler;
  uint16_t 	SPI_FirstBit;            
  uint16_t 	SPI_CRCPolynomial;
}SPI_InitTypeDef_A;

typedef enum
{ 
  GPIO_Mode_IN   = 0x00, /*!< GPIO Input Mode */
  GPIO_Mode_OUT  = 0x01, /*!< GPIO Output Mode */
  GPIO_Mode_AF   = 0x02, /*!< GPIO Alternate function Mode */
  GPIO_Mode_AN   = 0x03  /*!< GPIO Analog Mode */
}GPIOMode_TypeDef;

typedef enum
{ 
  GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
}GPIOOType_TypeDef;

typedef enum
{
  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_TypeDef;

typedef enum
{ 
  GPIO_Speed_2MHz   = 0x01, /*!< Medium Speed */
  GPIO_Speed_10MHz  = 0x02, /*!< Fast Speed   */
  GPIO_Speed_50MHz  = 0x03  /*!< High Speed   */
}GPIOSpeed_TypeDef;


typedef struct
{
  uint32_t 		GPIO_Pin; 
  GPIOMode_TypeDef 	GPIO_Mode;
  GPIOSpeed_TypeDef 	GPIO_Speed;
  GPIOOType_TypeDef 	GPIO_OType;
  GPIOPuPd_TypeDef 	GPIO_PuPd;
}GPIO_InitTypeDef;

///////////////////////////////////////////////////////////////////////////////
//
// Global Variables
//
///////////////////////////////////////////////////////////////////////////////

SPI_TypeDef  gSPI_TypeDef;

SPI_InitTypeDef_A       gSPI_Init = {
		  	  0x0000,  // 2Lines_FullDuplex;
			  0x0700,  // SPI_DataSize_8b
			  0x0000,  // SPI_CPOL_LOW
			  0x0000,  // SPI_CPHA_1Edge
			  0x0200,  // SPI_NSS_Soft
			  0x0010,  // SPI_BaudRatePrescaler_8
			  0x0000,  // SPI_FirstBit_MSB
			  7,       // Polynomial
			  0x0104   // SPI_Mode_Master
			};

/*****************************************************************************
  static Function Declaration :				
*****************************************************************************/
static uint8_t L3GD20_SendByte(uint8_t byte);

/*****************************************************************************
  Function Declaration :				
*****************************************************************************/

void  	L3GD20_LowLevelInit(SPIDriver *drvspi, SPIConfig *spi_cfg);
void    L3GD20_Read(uint8_t* pBuffer, uint8_t *ReadAddr, uint16_t NumByteToRead);

///////////////////////////////////////////////////////////////////////////////
//
//  External global Variables :
//
///////////////////////////////////////////////////////////////////////////////

extern 	SPIDriver	gSpiDriver;  
extern  SPIConfig       gSpiConfig;


#define   VAR_OR(x)   (x.SPI_Direction | x.SPI_Mode | x.SPI_CPOL | x.SPI_CPHA | x.SPI_NSS | x.SPI_BaudRatePrescaler | x.SPI_FirstBit) 


void SPI_Init(SPI_TypeDef *SPIx, SPIConfig *scfg)
{
  uint16_t   reg = 0;

  reg = (gSPI_Init.SPI_Mode == SPI_MODE_MASTER)   
       ? SPIx->CR1 
       : SPIx->CR2;

  reg &= (gSPI_Init.SPI_Mode == SPI_MODE_MASTER)  
       ? CR1_CLEAR_MASK 
       : ~SPI_CR2_DS;

  reg |= (gSPI_Init.SPI_Mode == SPI_MODE_MASTER)  
       ? VAR_OR(gSPI_Init) 
       : (gSPI_Init.SPI_DataSize);

  (gSPI_Init.SPI_Mode == SPI_MODE_MASTER) 
       ? (SPIx->CR1 = scfg->cr1 = reg) 
       : (SPIx->CR2 = scfg->cr2 = reg);

  if(gSPI_Init.SPI_Mode == SPI_MODE_MASTER){
	reg = SPIx->CR2;
	reg &= (uint16_t)~SPI_CR2_DS;

	// Configure SPIx : Data Size */
	reg |= (uint16_t)(gSPI_Init.SPI_DataSize);
	SPIx->CR2 = scfg->cr2 = reg;
  } else {
	reg = SPIx->CR1;
	reg &= CR1_CLEAR_MASK;
	reg |= VAR_OR(gSPI_Init);

	SPIx->CR1 = scfg->cr1 = reg;
  }
 
  /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
  SPIx->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);

  /* Write to SPIx CRCPOLY */
  SPIx->CRCPR = gSPI_Init.SPI_CRCPolynomial;
}


  /**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */

uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
  while (1)
  { 

  }
}

 /**
  * @brief  Checks whether the specified SPI flag is set or not.
  * @param  SPIx: To select the SPIx/I2Sx peripheral, where x can be: 1, 2 or 3 
  *      in SPI mode or 2 or 3 in I2S mode or I2Sxext for I2S full duplex mode.  
  * @param  SPI_I2S_FLAG: specifies the SPI flag to check. 
  *   This parameter can be one of the following values:
  *     @arg SPI_I2S_FLAG_TXE: Transmit buffer empty flag.
  *     @arg SPI_I2S_FLAG_RXNE: Receive buffer not empty flag.
  *     @arg SPI_I2S_FLAG_BSY: Busy flag.
  *     @arg SPI_I2S_FLAG_OVR: Overrun flag.
  *     @arg SPI_I2S_FLAG_MODF: Mode Fault flag.
  *     @arg SPI_I2S_FLAG_CRCERR: CRC Error flag.
  *     @arg SPI_I2S_FLAG_FRE: TI frame format error flag.
  *     @arg I2S_FLAG_UDR: Underrun Error flag.
  *     @arg I2S_FLAG_CHSIDE: Channel Side flag.   
  * @retval The new state of SPI_I2S_FLAG (SET or RESET).
  */

FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, 
				 uint16_t SPI_I2S_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the status of the specified SPI flag */
  bitstatus = ((SPIx->SR & SPI_I2S_FLAG) != (uint16_t)RESET) ? SET : RESET;
  
  /* Return the SPI_I2S_FLAG status */
  return  bitstatus;
}

/**
  * @brief  Configures the FIFO reception threshold for the selected SPI.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral. 
  * @param  SPI_RxFIFOThreshold: specifies the FIFO reception threshold.
  *   This parameter can be one of the following values:
  *     @arg SPI_RxFIFOThreshold_HF: RXNE event is generated if the FIFO 
  *          level is greater or equal to 1/2. 
  *     @arg SPI_RxFIFOThreshold_QF: RXNE event is generated if the FIFO 
  *          level is greater or equal to 1/4. 
  * @retval None
  */

#define SPI_THRESH_HOLD    	  (gSPI_TypeDef.CR2 &=(uint16_t)~((uint16_t)SPI_CR2_FRXTH))
#define SPI_RXFIFO_CONFIG(value)  (gSPI_TypeDef.CR2 |= gSpiConfig.cr2 = (value | SPI_THRESH_HOLD))


static void SPI_RxFIFOThresholdConfig(  SPI_TypeDef* SPIx, 
				  	uint16_t SPI_RxFIFOThreshold, 
				  	SPIConfig *scfg)
{
  /* Clear FRXTH bit */
  SPIx->CR2 &= (uint16_t)~((uint16_t)SPI_CR2_FRXTH);
  
  /* Set new FRXTH bit value */
  SPIx->CR2 |= SPI_RxFIFOThreshold;
  scfg->cr2 = SPIx->CR2;
}

#define  SPI_ENAB(state)  ((state== ENABLE) ? (gSPI_TypeDef.CR1 |= gSpiConfig.cr1 |= SPI_CR1_SPE) : gSpiConfig.cr1 &= gSPI_TypeDef.CR1 &=((uint16_t)~((uint16_t)SPI_CR1_SPE)));

static void SPI_ENABLE(SPI_TypeDef* SPIx, FunctionalState NewState, SPIConfig *scfg)
{
  /* Check the parameters */

  (NewState != DISABLE) ? (SPIx->CR1 |= scfg->cr1 |= SPI_CR1_SPE) 
   			 : (SPIx->CR1 &= scfg->cr1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE));
}


/** @defgroup SPI_FIFO_reception_threshold 
  * @{
  */

#define SPI_RxFIFOThreshold_HF               ((uint16_t)0x0000)
#define SPI_RxFIFOThreshold_QF               ((uint16_t)0x1000)
#define IS_SPI_RX_FIFO_THRESHOLD(THRESHOLD)  (((THRESHOLD) == SPI_RxFIFOThreshold_HF) || \
                                             ((THRESHOLD) == SPI_RxFIFOThreshold_QF))

static void  L3GD_SPI_Init(SPI_TypeDef *spi, SPIConfig *spiconfig)
{
  SPI_Init(spi, spiconfig);

  /* Configure the RX FIFO Threshold */
  //SPI_RXFIFO_CONFIG(SPI_RxFIFOThreshold_QF);
  
  SPI_RxFIFOThresholdConfig(spi, SPI_RxFIFOThreshold_QF, spiconfig);

  /* Enable SPI1	*/
  //SPI_ENABLE(ENABLE);
  SPI_ENABLE(spi, ENABLE, spiconfig);
}

/**
  * @brief  Reads a block of data from the L3GD20.
  * @param  pBuffer : pointer to the buffer that receives the data read from the L3GD20.
  * @param  ReadAddr : L3GD20's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the L3GD20.
  * @retval None
  */

void L3GD20_Read(uint8_t* pBuffer, uint8_t *ReadAddr, uint16_t NumByteToRead)
{  
  if(NumByteToRead > 0x01)
  {
    *ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    *ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  L3GD20_CS_LOW();

 
  /* Send the Address of the indexed register */
  L3GD20_SendByte(*ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to L3GD20 (Slave device) */
    	*pBuffer = L3GD20_SendByte(DUMMY_BYTE);
    	NumByteToRead--;
    	pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  L3GD20_CS_HIGH();
}  

/**
  * @brief  Writes one byte to the L3GD20.
  * @param  pBuffer : pointer to the buffer  containing the data to be written to the L3GD20.
  * @param  WriteAddr : L3GD20's internal address to write to.
  * @param  NumByteToWrite: Number of bytes to write.
  * @retval None
  */
void L3GD20_Write(uint8_t* pBuffer, uint8_t *WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit: 
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
  */

  if(NumByteToWrite > 0x01)
  {
    *WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }

  /* Set chip select Low at the start of the transmission */
  L3GD20_CS_LOW();
  
  /* Send the Address of the indexed register */
  L3GD20_SendByte(*WriteAddr);
  
  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    L3GD20_SendByte(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }
 
  /* Set chip select High at the end of the transmission */ 
  L3GD20_CS_HIGH();
}

__IO uint32_t  L3GD20Timeout = L3GD20_FLAG_TIMEOUT; 

  /**
  *
  * @brief  Sends a Byte through the SPI interface and return the Byte received 
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval The received byte value
  *
  */
static uint8_t L3GD20_SendByte(uint8_t byte)
{
  uint8_t  rxbuf;

  /* Loop while DR register in not empty */
  L3GD20Timeout = L3GD20_FLAG_TIMEOUT;  

  while (SPI_I2S_GetFlagStatus(gSpiDriver.spi, SPI_I2S_FLAG_TXE) == RESET)
  {
    if((L3GD20Timeout--) == 0) 
	return L3GD20_TIMEOUT_UserCallback();
  }  

  spiExchange(&SPID1,1, &byte, &rxbuf); // It works throughly 

  return rxbuf;
}

/******************************************************************************************
*
* @brief  Set L3GD20 Initialization.
* @param  L3GD20_InitStruct: pointer to a L3GD20_InitTypeDef structure 
*         that contains the configuration setting for the L3GD20.
*
* @retval None
*
*
*****************************************************************************************/
  
void L3GD20_Init(SPIDriver *lspidrive, SPIConfig *lspi_cfg)
{	
    L3GD20_LowLevelInit(lspidrive, lspi_cfg);
}	

/**
  * @brief  Writes data to the specified GPIO data port.
  * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
  * @param  GPIO_PinSource: specifies the pin for the Alternate function.
  *   This parameter can be GPIO_PinSourcex where x can be (0..15).
  * @param  GPIO_AF: selects the pin to be used as Alternate function.  
  *   This parameter can be one of the following value:
  *     @arg GPIO_AF_0:  JTCK-SWCLK, JTDI, JTDO/TRACESW0, JTMS-SWDAT, MCO, NJTRST, 
  *                      TRACED, TRACECK.
  *     @arg GPIO_AF_1:  OUT, TIM2, TIM15, TIM16, TIM17.
  *     @arg GPIO_AF_2:  COMP1_OUT, TIM1, TIM2, TIM3, TIM4, TIM8, TIM15.
  *     @arg GPIO_AF_3:  COMP7_OUT, TIM8, TIM15, Touch.
  *     @arg GPIO_AF_4:  I2C1, I2C2, TIM1, TIM8, TIM16, TIM17.
  *     @arg GPIO_AF_5:  IR_OUT, I2S2, I2S3, SPI1, SPI2, TIM8, USART4, USART5
  *     @arg GPIO_AF_6:  IR_OUT, I2S2, I2S3, SPI2, SPI3, TIM1, TIM8
  *     @arg GPIO_AF_7:  AOP2_OUT, CAN, COMP3_OUT, COMP5_OUT, COMP6_OUT, USART1, 
  *                      USART2, USART3.
  *     @arg GPIO_AF_8:  COMP1_OUT, COMP2_OUT, COMP3_OUT, COMP4_OUT, COMP5_OUT, 
  *                      COMP6_OUT.
  *     @arg GPIO_AF_9:  AOP4_OUT, CAN, TIM1, TIM8, TIM15.
  *     @arg GPIO_AF_10: AOP1_OUT, AOP3_OUT, TIM2, TIM3, TIM4, TIM8, TIM17. 
  *     @arg GPIO_AF_11: TIM1, TIM8.
  *     @arg GPIO_AF_12: TIM1.
  *     @arg GPIO_AF_14: USBDM, USBDP.
  *     @arg GPIO_AF_15: OUT.             
  * @note  The pin should already been configured in Alternate Function mode(AF)
  *        using GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
  * @note  Refer to the Alternate function mapping table in the device datasheet 
  *        for the detailed mapping of the system and peripherals alternate 
  *        function I/O pins.
  * @retval None
  */
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF)
{
  uint32_t temp = 0x00;
  uint32_t temp_2 = 0x00;
  
  /* Check the parameters */
#if 0  // tsham blocked 
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN_SOURCE(GPIO_PinSource));
  assert_param(IS_GPIO_AF(GPIO_AF));
#endif

  temp = ((uint32_t)(GPIO_AF) << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
#if 0 // tsham blocked
  GPIOx->AFR[GPIO_PinSource >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
#else
	if (GPIO_PinSource < 0x08)
		GPIOx->AFRL &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
	else
		GPIOx->AFRH &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
#endif

#if 0 // tsham
  temp_2 = GPIOx->AFR[GPIO_PinSource >> 0x03] | temp;
  GPIOx->AFR[GPIO_PinSource >> 0x03] = temp_2;
#else  // ########## It could be checked  ############  tsham
	if(GPIO_PinSource < 0x08) {
		temp_2 = GPIOx->AFRL | temp;
		GPIOx->AFRL = temp_2;
	}
	else {
		temp_2 = GPIOx->AFRH | temp;
		GPIOx->AFRH = temp_2;		
	}
#endif
}


/********************************************************************************
  Function Name : 
   void  L3GD20_LowLevelInit(SPIDriver *drvspi, SPIConfig *spi_cfg)

  Parameters    :
   SPIDriver   *drvspi  :

   SPIConfig   *spi_cfg :

*********************************************************************************/

#define  TEST1
#undef   TEST2

// 210313,tsham : 
//
// In order to test between ChibiOS driver and ST own function, _pal_lld_setgroupmode 
// : ALTERNATE_MASK has problem, so it was changed first of all 
// (/os/hal/platforms/STM32/pal_lld.c) _pal_lld_setgroupmode got error in terms of ST
//  driver. It has different result. it was checked by task operation
//  Also, MODE, OTYPE, PUDR, OSPEED make not operated.
//  _pal_lld_setgroupmode function should be examined. it is strange that I cannot find 
//  out the usage from any other files.
//

void  L3GD20_LowLevelInit(SPIDriver *drvspi, SPIConfig *spi_cfg)
{

  rccEnableAPB2(RCC_APB2RSTR_SPI1RST, TRUE);	
  rccEnableAHB(GPIOA_SPI1_SCK | GPIOA_SPI1_MOSI | GPIOA_SPI1_MISO, TRUE);
  rccEnableAHB(RCC_AHBENR_GPIOEEN, ENABLE);   // GPIOE Clock Enable
  rccEnableAHB(RCC_AHBENR_GPIOEEN, ENABLE);   // GPIOE Clock Enable
  rccEnableAHB(RCC_AHBENR_GPIOEEN, ENABLE);   // GPIOE Clock Enable

#if 0// defined(TEST1)
  _pal_lld_setgroupmode(GPIOA,PAL_STM32_ALTERNATE_MASK, GPIOA_SPI1_SCK << ALTERNATE_SHIFT);
  _pal_lld_setgroupmode(GPIOA,PAL_STM32_ALTERNATE_MASK, GPIOA_SPI1_MISO << ALTERNATE_SHIFT);
  _pal_lld_setgroupmode(GPIOA,PAL_STM32_ALTERNATE_MASK, GPIOA_SPI1_MOSI << ALTERNATE_SHIFT);
#else
  GPIO_PinAFConfig(GPIOA, 0x05, 0x05);
  GPIO_PinAFConfig(GPIOA, 0x06, 0x05);
  GPIO_PinAFConfig(GPIOA, 0x07, 0x05);
#endif

  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////
#if defined(TEST1)
  _pal_lld_setgroupmode(GPIOA, PAL_STM32_MODE_MASK, 0x04 << MODE_SHIFT);   // GPIO_Mode_AF 
  _pal_lld_setgroupmode(GPIOA, PAL_STM32_OTYPE_MASK, 0x00 << OTYPE_SHIFT); // GPIO_OType_PP
  _pal_lld_setgroupmode(GPIOA, PAL_STM32_OTYPE_PUSHPULL, 0x00 << PUDR_SHIFT); //GPIO_PuPd_NOPUU
  _pal_lld_setgroupmode(GPIOA, PAL_STM32_OSPEED_HIGHEST, 0x03 << OSPEED_SHIFT); // highest 50Hz\

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
  pal_lld_writepad(GPIOA,5, GPIO_BSRR_BS_5);  //SCK
  pal_lld_writepad(GPIOA,7, GPIO_BSRR_BS_7);  //MOSI
  pal_lld_writepad(GPIOA,6, GPIO_BSRR_BS_6);  //MISO
  
#else
  GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_SCK_PIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  L3GD20_SPI_MOSI_PIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_MISO_PIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

  /***  SPI Configuration -----------------------------------------------------*/
  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  rccEnableAPB2(RCC_APB2RSTR_SPI1RST, ENABLE);  // Enable SPI_De_Init
  rccEnableAPB2(RCC_APB2RSTR_SPI1RST, DISABLE); // Disable SPI_De_Init
  spi_cfg->ssport = GPIOE;         // spi configuration ssport allocation 
  spi_cfg->sspad  = GPIOE_SPI1_CS; // spi sspad allocation from SPIOE_SPI1_CS

	
  ///////////////////////////////////////////////////////////////////////////////
  //
  L3GD_SPI_Init(drvspi->spi, spi_cfg); // drvspi->spi initialize, and spi_cfg initialize
  //
  ///////////////////////////////////////////////////////////////////////////////
	
  _pal_lld_setgroupmode(GPIOE, PAL_STM32_MODE_MASK, 0x02 << MODE_SHIFT);         // GPIO_Mode_OUT
  _pal_lld_setgroupmode(GPIOE, PAL_STM32_OTYPE_MASK, 0x00 << OTYPE_SHIFT);       // 3: GPIO_OType_PP
  _pal_lld_setgroupmode(GPIOE, PAL_STM32_OSPEED_HIGHEST,  0x03 << OSPEED_SHIFT); // 0x03:50MHz

  pal_lld_writepad(GPIOE,3, GPIO_BSRR_BS_3);
  pal_lld_setport(GPIOE, GPIO_BSRR_BS_3);   // Deselect : Chip Select high

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Configure GPIO PINS to detect Interrupts
  //
  ///////////////////////////////////////////////////////////////////////////////

  _pal_lld_setgroupmode(GPIOE, PAL_STM32_MODE_MASK,0x01 << MODE_SHIFT);  	// GPIO_Mode_IN
  _pal_lld_setgroupmode(GPIOE, PAL_STM32_OTYPE_MASK, 0x00 << OTYPE_SHIFT);
  _pal_lld_setgroupmode(GPIOE, PAL_STM32_OSPEED_HIGHEST, 0x03 << OSPEED_SHIFT);
  _pal_lld_setgroupmode(GPIOE, PAL_STM32_OTYPE_PUSHPULL, 0x00 << PUDR_SHIFT);
	
  pal_lld_writepad(GPIOE,0, GPIO_BSRR_BS_0);    		// INT1 pin	
  pal_lld_writepad(GPIOE,1, GPIO_BSRR_BS_1);    		// INT2 pin
    
}
