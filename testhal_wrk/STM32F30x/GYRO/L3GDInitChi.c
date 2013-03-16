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

#include  "stm32f3_discovery_l3gd20.h"

#define SPI_Mode_Master              ((uint16_t)0x0104)
#define SPI_Mode_Slave               ((uint16_t)0x0000)

#define CR1_CLEAR_MASK       	     ((uint16_t)0x3040)

#define SPI_I2S_FLAG_RXNE            ((uint16_t)0x0001)
#define SPI_I2S_FLAG_TXE             ((uint16_t)0x0002)

#define  ALTERNATE_SHIFT   				7
#define  PUDR_SHIFT 					5
#define	 OSPEED_SHIFT					3
#define  OTYPE_SHIFT					2
#define	 MODE_SHIFT						0


/*****************************************************************************
  TypeDef 				
*****************************************************************************/

typedef struct
{
  uint16_t 		SPI_Direction;           
  uint16_t 		SPI_Mode;
  uint16_t 		SPI_DataSize;
  uint16_t 		SPI_CPOL; 
  uint16_t 		SPI_CPHA;    
  uint16_t 		SPI_NSS; 
  uint16_t 		SPI_BaudRatePrescaler;
  uint16_t 		SPI_FirstBit;            
  uint16_t 		SPI_CRCPolynomial;
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
  uint32_t 				GPIO_Pin; 
  GPIOMode_TypeDef 		GPIO_Mode;
  GPIOSpeed_TypeDef 	GPIO_Speed;
  GPIOOType_TypeDef 	GPIO_OType;
  GPIOPuPd_TypeDef 		GPIO_PuPd;
}GPIO_InitTypeDef;

/*****************************************************************************
  Function Declaration :				
*****************************************************************************/
static uint8_t L3GD20_SendByte(uint8_t byte);
void  L3GD20_LowLevelInit(SPIDriver *drvspi, SPIConfig *spi_cfg);


/*****************************************************************************
  External global Variables :
*****************************************************************************/
extern SPIDriver	gSpiDriver, gSpiConfig;  


/**
  * @brief  Initializes the SPIx peripheral according to the specified 
  *         parameters in the SPI_InitStruct.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  SPI_InitStruct: pointer to a SPI_InitTypeDef structure that
  *         contains the configuration information for the specified SPI peripheral.
  * @retval None
  */
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef_A* SPI_InitStruct, SPIConfig *scfg)
{
  uint16_t tmpreg = 0;

  /* Configuring the SPI in master mode */
  if(SPI_InitStruct->SPI_Mode == SPI_Mode_Master)
  {
    tmpreg = SPIx->CR1;
	
    tmpreg &= CR1_CLEAR_MASK;

    tmpreg |= (uint16_t)((uint16_t)(SPI_InitStruct->SPI_Direction | SPI_InitStruct->SPI_Mode) |
                  (uint16_t)((uint16_t)(SPI_InitStruct->SPI_CPOL | SPI_InitStruct->SPI_CPHA) |
                  (uint16_t)((uint16_t)(SPI_InitStruct->SPI_NSS | SPI_InitStruct->SPI_BaudRatePrescaler) | 
                  SPI_InitStruct->SPI_FirstBit)));
    /* Write to SPIx CR1 */
    SPIx->CR1 = tmpreg;
	scfg->cr1 = tmpreg;
	
    /*-------------------------Data Size Configuration -----------------------*/
    /* Get the SPIx CR2 value */
    tmpreg = SPIx->CR2;
    /* Clear DS[3:0] bits */
    tmpreg &= (uint16_t)~SPI_CR2_DS;
    /* Configure SPIx: Data Size */
    tmpreg |= (uint16_t)(SPI_InitStruct->SPI_DataSize);
    /* Write to SPIx CR2 */
    SPIx->CR2 = tmpreg;
	scfg->cr2 = tmpreg;
  }
  /* Configuring the SPI in slave mode */
  else
  {
/*---------------------------- Data size Configuration -----------------------*/
    /* Get the SPIx CR2 value */
    tmpreg = SPIx->CR2;
    /* Clear DS[3:0] bits */
    tmpreg &= (uint16_t)~SPI_CR2_DS;
    /* Configure SPIx: Data Size */
    tmpreg |= (uint16_t)(SPI_InitStruct->SPI_DataSize);
    /* Write to SPIx CR2 */
    SPIx->CR2 = tmpreg;
	scfg->cr2 = tmpreg;
/*---------------------------- SPIx CR1 Configuration ------------------------*/
    /* Get the SPIx CR1 value */
    tmpreg = SPIx->CR1;
    /* Clear BIDIMode, BIDIOE, RxONLY, SSM, SSI, LSBFirst, BR, MSTR, CPOL and CPHA bits */
    tmpreg &= CR1_CLEAR_MASK;
    /* Configure SPIx: direction, NSS management, first transmitted bit, BaudRate prescaler
       master/salve mode, CPOL and CPHA */
    /* Set BIDImode, BIDIOE and RxONLY bits according to SPI_Direction value */
    /* Set SSM, SSI and MSTR bits according to SPI_Mode and SPI_NSS values */
    /* Set LSBFirst bit according to SPI_FirstBit value */
    /* Set BR bits according to SPI_BaudRatePrescaler value */
    /* Set CPOL bit according to SPI_CPOL value */
    /* Set CPHA bit according to SPI_CPHA value */
    tmpreg |= (uint16_t)((uint16_t)(SPI_InitStruct->SPI_Direction | SPI_InitStruct->SPI_Mode) | 
                         (uint16_t)((uint16_t)(SPI_InitStruct->SPI_CPOL | SPI_InitStruct->SPI_CPHA) | 
                         (uint16_t)((uint16_t)(SPI_InitStruct->SPI_NSS | SPI_InitStruct->SPI_BaudRatePrescaler) | 
                         SPI_InitStruct->SPI_FirstBit)));

    /* Write to SPIx CR1 */
    SPIx->CR1 = tmpreg;
	scfg->cr1 = tmpreg;
  }

  /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
  SPIx->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);

/*---------------------------- SPIx CRCPOLY Configuration --------------------*/
  /* Write to SPIx CRCPOLY */
  SPIx->CRCPR = SPI_InitStruct->SPI_CRCPolynomial;
}

/**
  * @brief  Transmits a Data through the SPIx peripheral.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  Data: Data to be transmitted.
  * @retval None
  */
void SPI_SendData8(SPI_TypeDef* SPIx, uint8_t Data)
{
  uint32_t spixbase = 0x00;

  /* Check the parameters */
#if 0  
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
#endif

  spixbase = (uint32_t)SPIx; 
  spixbase += 0x0C;
  
  *(__IO uint8_t *) spixbase = Data;
}

/**
  * @brief  Returns the most recent received data by the SPIx peripheral. 
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @retval The value of the received data.
  */
uint8_t SPI_ReceiveData8(SPI_TypeDef* SPIx)
{
  uint32_t spixbase = 0x00;
  
  /* Check the parameters */
#if 0  
  assert_param(IS_SPI_ALL_PERIPH_EXT(SPIx));
#endif

  spixbase = (uint32_t)SPIx; 
  spixbase += 0x0C;
  
  return *(__IO uint8_t *) spixbase;
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
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */

  /* Check the status of the specified SPI flag */
  if ((SPIx->SR & SPI_I2S_FLAG) != (uint16_t)RESET)
  {
    /* SPI_I2S_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* SPI_I2S_FLAG is reset */
    bitstatus = RESET;
  }
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
static void SPI_RxFIFOThresholdConfig( SPI_TypeDef* SPIx, 
				  uint16_t SPI_RxFIFOThreshold, 
				  SPIConfig *scfg)
{
  /* Clear FRXTH bit */
  SPIx->CR2 &= (uint16_t)~((uint16_t)SPI_CR2_FRXTH);
  
  /* Set new FRXTH bit value */
  SPIx->CR2 |= SPI_RxFIFOThreshold;

  scfg->cr2 = SPIx->CR2;
}


static void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState, SPIConfig *scfg)
{
  /* Check the parameters */

  if (NewState != DISABLE)
  {
    /* Enable the selected SPI peripheral */
    SPIx->CR1 |= SPI_CR1_SPE;
  }
  else
  {
    /* Disable the selected SPI peripheral */
    SPIx->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);
  }

  scfg->cr1 = SPIx->CR1; 
}


/** @defgroup SPI_FIFO_reception_threshold 
  * @{
  */

#define SPI_RxFIFOThreshold_HF          	 ((uint16_t)0x0000)
#define SPI_RxFIFOThreshold_QF          	 ((uint16_t)0x1000)
#define IS_SPI_RX_FIFO_THRESHOLD(THRESHOLD)  (((THRESHOLD) == SPI_RxFIFOThreshold_HF) || \
                                             ((THRESHOLD) == SPI_RxFIFOThreshold_QF))

static void  L3GD_SPI_Init(SPI_TypeDef *spi, SPIConfig *spiconfig)
{
	SPI_InitTypeDef_A  SPI_InitStructure;

	SPI_InitStructure.SPI_Direction = ((uint16_t)0x0000); //  2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = ((uint16_t)0x0700);   //SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = ((uint16_t)0x0000);		  //SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA =  ((uint16_t)0x0000); 	//SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = ((uint16_t)0x0200);      // SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = ((uint16_t)0x0010); //SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = ((uint16_t)0x0000);	// SPI_FirstBit_MSB
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = ((uint16_t)0x0104); //SPI_Mode_Master;
	
	SPI_Init(spi, &SPI_InitStructure, spiconfig);

	/* Configure the RX FIFO Threshold */
	SPI_RxFIFOThresholdConfig(spi, SPI_RxFIFOThreshold_QF, spiconfig);
	/* Enable SPI1	*/
	SPI_Cmd(spi, ENABLE, spiconfig);
	
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
  #if 1
  L3GD20_SendByte(*ReadAddr);
  #else
  spiExchange(&gSpiDriver,NumByteToRead,ReadAddr,pBuffer);
  #endif
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to L3GD20 (Slave device) */
	#if 1
    *pBuffer = L3GD20_SendByte(DUMMY_BYTE);
	#else
	spiSend(&gSpiDriver,NumByteToRead,*pBuffer);
	#endif
    NumByteToRead--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  L3GD20_CS_HIGH();
}  

//spiExchange(SPIDriver * spip,size_t n,const void * txbuf,void * rxbuf)

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

#if 1 
  L3GD20_SendByte(*WriteAddr);
#else
   spiExchange(&gSpiDriver,NumByteToWrite,WriteAddr,pBuffer);
#endif
  
  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
  	#if 1
    L3GD20_SendByte(*pBuffer);
	#else
	spiSend(&gSpiDriver,NumByteToWrite,*pBuffer);
	#endif
    NumByteToWrite--;
    pBuffer++;
  }
 
  /* Set chip select High at the end of the transmission */ 
  L3GD20_CS_HIGH();
}



__IO uint32_t  L3GD20Timeout = L3GD20_FLAG_TIMEOUT; 


/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received 
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval The received byte value
  */
static uint8_t L3GD20_SendByte(uint8_t byte)
{
 uint8_t  rxbuf;

  /* Loop while DR register in not empty */
  L3GD20Timeout = L3GD20_FLAG_TIMEOUT;  
  return;  // =============> tsham blocked for testing 1

  while (SPI_I2S_GetFlagStatus(gSpiDriver.spi, SPI_I2S_FLAG_TXE) == RESET)
  {
    if((L3GD20Timeout--) == 0) 
		return L3GD20_TIMEOUT_UserCallback();
  }  

  spiExchange(&SPID1,1, &byte, &rxbuf); // to need checkig.
  return rxbuf;
  
  /* Send a Byte through the SPI peripheral */
#if 0  // tsham changed
  SPI_SendData8(gSpiDriver, byte);
#else
  spiStartSend(&gSpiDriver, 1, &byte);
#endif
  
  /* Wait to receive a Byte */
  L3GD20Timeout = L3GD20_FLAG_TIMEOUT;   // default value is 0x1000

  while (SPI_I2S_GetFlagStatus(gSpiDriver.spi, SPI_I2S_FLAG_RXNE) == RESET)
  {
    if((L3GD20Timeout--) == 0) 
		return L3GD20_TIMEOUT_UserCallback();
  }
 
  /* Return the Byte read from the SPI bus */
#if 0  
  return (uint8_t)SPI_ReceiveData8(gSpiDriver);
#else
  spiReceive(&gSpiDriver,1, &rxbuf);	
  return  rxbuf;
#endif
}

/**
  * @brief  Set L3GD20 Initialization.
  * @param  L3GD20_InitStruct: pointer to a L3GD20_InitTypeDef structure 
  *         that contains the configuration setting for the L3GD20.
  * @retval None
 */
  
void L3GD20_Init(SPIDriver *lspidrive, SPIConfig *lspi_cfg)
{	
	//lspidrive->spi = SPI1;
	L3GD20_LowLevelInit(lspidrive, lspi_cfg);
}	

void  L3GD20_Init_Setting(L3GD20_InitTypeDef *L3GD20_InitStruct)
{
	static uint8_t   	ctrl_reg;
	uint8_t 			ctrl1 = 0x00, ctrl4 = 0x00;


	/* Configure MEMS: data rate, power mode, full scale and axes */
	ctrl1 |= (uint8_t) (L3GD20_InitStruct->Power_Mode | L3GD20_InitStruct->Output_DataRate | \
					  L3GD20_InitStruct->Axes_Enable | L3GD20_InitStruct->Band_Width);

	ctrl4 |= (uint8_t) (L3GD20_InitStruct->BlockData_Update | L3GD20_InitStruct->Endianness | \
					  L3GD20_InitStruct->Full_Scale);

	ctrl_reg = 0x20;
	/* Write value to MEMS CTRL_REG1 regsister */
	L3GD20_Write(&ctrl1, &ctrl_reg, 1);

	ctrl_reg = 0x23;
	/* Write value to MEMS CTRL_REG4 regsister */
	L3GD20_Write(&ctrl4, &ctrl_reg, 1);
}

// Ino order to replace GPIO_Init function
void _pal_lld_l3gdgroupmode(ioportid_t port, GPIO_InitTypeDef *gpio_init)
{
	  uint32_t pinpos = 0x00, pos = 0x00 , currentpin = 0x00;
		
	  for (pinpos = 0x00; pinpos < 0x10; pinpos++)
	  {
		pos = ((uint32_t)0x01) << pinpos;
	
		/* Get the port pins position */
		currentpin = (gpio_init->GPIO_Pin) & pos;
	
		if (currentpin == pos)
		{
		  if (  (gpio_init->GPIO_Mode == GPIO_Mode_OUT) 
		  	    || (gpio_init->GPIO_Mode == GPIO_Mode_AF))
		  {	
			/* Speed mode configuration */
			port->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));
			port->OSPEEDR |= ((uint32_t)(gpio_init->GPIO_Speed) << (pinpos * 2));
		
			/* Output mode configuration */
			port->OTYPER &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)pinpos));
			port->OTYPER |= (uint16_t)(((uint16_t)gpio_init->GPIO_OType) << ((uint16_t)pinpos));
		  }
		  
		  port->MODER	&= ~(GPIO_MODER_MODER0 << (pinpos * 2));	
		  port->MODER |= (((uint32_t)gpio_init->GPIO_Mode) << (pinpos * 2));
	
		  /* Pull-up Pull down resistor configuration */
		  port->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)pinpos * 2));
		  port->PUPDR |= (((uint32_t)gpio_init->GPIO_PuPd) << (pinpos * 2));
		}
	  }

}

void  L3GD20_LowLevelInit(SPIDriver *drvspi, SPIConfig *spi_cfg)
{
	GPIO_InitTypeDef   	s_gpio_init;

	//if(drvspi->spi == SPI1)
	rccEnableAPB2(RCC_APB2RSTR_SPI1RST, TRUE);	
	rccEnableAHB(GPIOA_SPI1_SCK | GPIOA_SPI1_MOSI | GPIOA_SPI1_MISO, TRUE);
	rccEnableAHB(RCC_AHBENR_GPIOEEN, ENABLE);   // GPIOE Clock Enable
#if 1	// ??? According to the st platform driver, this performed three times.
	rccEnableAHB(RCC_AHBENR_GPIOEEN, ENABLE);
	rccEnableAHB(RCC_AHBENR_GPIOEEN, ENABLE);
#endif
	_pal_lld_setgroupmode(GPIOA,PAL_STM32_ALTERNATE_MASK,GPIOA_SPI1_SCK << ALTERNATE_SHIFT);
	//_pal_lld_setgroupmode(GPIOA,PAL_STM32_ALTERNATE_MASK, GPIOA_SPI1_SCK);
	_pal_lld_setgroupmode(GPIOA,PAL_STM32_ALTERNATE_MASK, GPIOA_SPI1_MISO << ALTERNATE_SHIFT);
	_pal_lld_setgroupmode(GPIOA,PAL_STM32_ALTERNATE_MASK, GPIOA_SPI1_MOSI << ALTERNATE_SHIFT);

	#if 1
	_pal_lld_setgroupmode(GPIOA, PAL_STM32_MODE_MASK, 0x02 << MODE_SHIFT);    
	_pal_lld_setgroupmode(GPIOA, PAL_STM32_OTYPE_MASK, 0x00 << OTYPE_SHIFT);
	_pal_lld_setgroupmode(GPIOA, PAL_STM32_OTYPE_PUSHPULL, 0x00 << PUDR_SHIFT);
	_pal_lld_setgroupmode(GPIOA, PAL_STM32_OSPEED_HIGHEST, 0x03 << OSPEED_SHIFT);
	#else
	s_gpio_init.GPIO_Mode = GPIO_Mode_AF;
	s_gpio_init.GPIO_OType = GPIO_OType_PP;
	s_gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	s_gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	s_gpio_init.GPIO_Pin = L3GD20_SPI_SCK_PIN;

	_pal_lld_l3gdgroupmode(GPIOA, &s_gpio_init);
	
	s_gpio_init.GPIO_Pin = L3GD20_SPI_MOSI_PIN;
	_pal_lld_l3gdgroupmode(GPIOA, &s_gpio_init);

	s_gpio_init.GPIO_Pin = L3GD20_SPI_MISO_PIN;
	_pal_lld_l3gdgroupmode(GPIOA, &s_gpio_init);
	#endif

	pal_lld_writepad(GPIOA,5, GPIO_BSRR_BS_5);
	pal_lld_writepad(GPIOA,7, GPIO_BSRR_BS_7);

	/***  SPI Configuration -----------------------------------------------------*/
	rccEnableAPB2(RCC_APB2RSTR_SPI1RST, ENABLE);  // Disable SPI_De_Init
	rccEnableAPB2(RCC_APB2RSTR_SPI1RST, DISABLE);  // Disable SPI_De_Init
	spi_cfg->ssport = GPIOE;
	spi_cfg->sspad = GPIOE_SPI1_CS;
	L3GD_SPI_Init(drvspi->spi, spi_cfg);
	
	//////////////////////////////////////////////////////////////////////////
	
	_pal_lld_setgroupmode(GPIOE, PAL_STM32_MODE_MASK, 0x02 << MODE_SHIFT);  // GPIO_Mode_OUT
	_pal_lld_setgroupmode(GPIOE, PAL_STM32_OTYPE_MASK, 0x00 << OTYPE_SHIFT); // 3: GPIO_OType_PP
	_pal_lld_setgroupmode(GPIOE, PAL_STM32_OSPEED_HIGHEST,  0x03 << OSPEED_SHIFT);

	pal_lld_writepad(GPIOE,3, GPIO_BSRR_BS_3);
	pal_lld_setport(GPIOE, GPIO_BSRR_BS_3);   // Deselect : Chip Select high

	// Configure GPIO PINS to detect Interrupts
	_pal_lld_setgroupmode(GPIOE, PAL_STM32_MODE_MASK,0x00 << MODE_SHIFT);   // GPIO_Mode_IN
	_pal_lld_setgroupmode(GPIOE, PAL_STM32_OTYPE_MASK, 0x00 << OTYPE_SHIFT);
	_pal_lld_setgroupmode(GPIOE, PAL_STM32_OSPEED_HIGHEST, 0x03 << OSPEED_SHIFT);
	_pal_lld_setgroupmode(GPIOE, PAL_STM32_OTYPE_PUSHPULL, 0x00 << PUDR_SHIFT);
	
	pal_lld_writepad(GPIOE,0, GPIO_BSRR_BS_0);    // INT1 pin	
	pal_lld_writepad(GPIOE,1, GPIO_BSRR_BS_1);    // INT2 pin
	//drvspi->state = SPI_READY;
	
}



