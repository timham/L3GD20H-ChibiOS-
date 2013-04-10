
#include  "stm32f30x.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//
//  Type Declaration 				
//

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


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//
//  Definition 				
//

#define SPI_MODE_MASTER             ((uint16_t)0x0104)
#define SPI_MODE_SLAVE              ((uint16_t)0x0000)

#define CR1_CLEAR_MASK              ((uint16_t)0x3040)

#define SPI_I2S_FLAG_RXNE           ((uint16_t)0x0001)
#define SPI_I2S_FLAG_TXE            ((uint16_t)0x0002)

#define SPI_DIR_FULLDUPLEX_2LINE    ((uint16_t)0x0000)
#define SPI_DIR_RXONLY_2LINE        ((uint16_t)0x0400)
#define SPI_DIR_RX_1LINE            ((uint16_t)0x8000)
#define SPI_DIR_TX_1LINE            ((uint16_t)0xC000)


#define SPI_DATASIZE_4BIT           ((uint16_t)0x0300)
#define SPI_DATASIZE_5BIT           ((uint16_t)0x0400)
#define SPI_DATASIZE_6BIT           ((uint16_t)0x0500)
#define SPI_DATASIZE_7BIT           ((uint16_t)0x0600)
#define SPI_DATASIZE_8BIT           ((uint16_t)0x0700)
#define SPI_DATASIZE_9BIT           ((uint16_t)0x0800)
#define SPI_DATASIZE_10BIT          ((uint16_t)0x0900)
#define SPI_DATASIZE_11BIT          ((uint16_t)0x0A00)
#define SPI_DATASIZE_12BIT          ((uint16_t)0x0B00)
#define SPI_DATASIZE_13BIT          ((uint16_t)0x0C00)
#define SPI_DATASIZE_14BIT          ((uint16_t)0x0D00)
#define SPI_DATASIZE_15BIT          ((uint16_t)0x0E00)
#define SPI_DATASIZE_16BIT          ((uint16_t)0x0F00)

// SPI Clock Polarity

#define  SPI_CPOL_LOW               ((uint16_t)0x0000)
#define  SPI_CPOL_HIGH              ((uint16_t)0x0002)
#define  IS_SPI_CPOL(CPOL)          (((CPOL) == SPI_CPOL_LOW) || ((CPOL) == SPI_CPOL_HIGH))

// SPI Clock Phase

#define  SPI_CPHA_1EDGE             ((uint16_t)0x0000)
#define  SPI_CPHA_2EDGE             ((uint16_t)0x0001)
#define  IS_SPI_CPHA(CPHA)          (((CPHA) == SPI_CPHA_1EDGE) || ((CPHA) == SPI_CPHA_2EDGE))

// SPI Slave Select Management

#define  SPI_NSS_SOFT               ((uint16_t)0x0200)
#define  SPI_NSS_HARD               ((uint16_t)0x0000)


// SPI BaudRate Prescaler

#define  SPI_BAUDRATE_PRESCALER_2    ((uint16_t)0x0000)
#define  SPI_BAUDRATE_PRESCALER_4    ((uint16_t)0x0008)
#define  SPI_BAUDRATE_PRESCALER_8    ((uint16_t)0x0010)
#define  SPI_BAUDRATE_PRESCALER_16   ((uint16_t)0x0018)
#define  SPI_BAUDRATE_PRESCALER_32   ((uint16_t)0x0020)
#define  SPI_BAUDRATE_PRESCALER_64   ((uint16_t)0x0028)
#define  SPI_BAUDRATE_PRESCALER_128  ((uint16_t)0x0030)
#define  SPI_BAUDRATE_PRESCALER_256  ((uint16_t)0x0038)

// SPI MSB LSB Transmission
#define  SPI_FIRSTBIT_MSB            ((uint16_t)0x0000)
#define  SPI_FIRSTBIT_LSB            ((uint16_t)0x0080)


// SPI_CRC_Polynomial
#define IS_SPI_CRC_POLYNOMIAL(POLYNOMIAL)   ((POLYNOMIAL) >= 0x01)

#define  ALTERNATE_SHIFT    7
#define  PUDR_SHIFT         5
#define  OSPEED_SHIFT       3
#define  OTYPE_SHIFT        2
#define  MODE_SHIFT         0

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// 
//  Globale Variables
//
//

SPI_TypeDef                 gSPI_TypeDef;

SPI_InitTypeDef_A        gSPI_Init = 
      {
           SPI_DIR_FULLDUPLEX_2LINE,   // 2Lines_FullDuplex
	   SPI_DATASIZE_8BIT,                // SPI_DataSize_8b
	   SPI_CPOL_LOW,                        // SPI_CPOL_LOW
	   SPI_CPHA_1EDGE,                    // SPI_CPHA_1Edge
	   SPI_NSS_SOFT,                        // SPI_NSS_Soft
	   SPI_BAUDRATE_PRESCALER_8,  // SPI_BaudRatePrescaler_8
	   SPI_FIRSTBIT_MSB,                   // SPI_FirstBit_MSB
	   7,                                             // Polynomial
	   SPI_MODE_MASTER                   // SPI_Mode_Master
      };

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// 
//  Global Procedures :
//


void SPI_Init(SPI_TypeDef *SPIx, SPIConfig *scfg);



//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// 
//  External global Variables :
//
  
extern SPIDriver	gSpiDriver; 
extern SPIConfig   gSpiConfig;  


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// 
//  External Functions
//

extern void RCC_APB2PeriphClockCmd(u32 RCC_APB2Periph, FunctionalState NewState);


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// 
//  Static Functions
//




