/**
 *  Copyright 2008 Silicon Laboratories, Inc.
 *  http://www.silabs.com
 *
 *  @file CommIF.c
 *  
 *  C File Description:
 *  @brief Communication interface functions for the EzRadio2 Loadboard.
 *
 *  Project Name: dev_EzR2_Loadboard 
 * 
 * 
 *  @author Sz. Papp
 *
 *  @date 2012.03.02.
 *
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 *  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 *  DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *  This software must be used in accordance with the End User License Agreement.
 */

/*****************************************************************************
 *  Includes
 *****************************************************************************/
#include "compiler_defs.h"
#include "C8051F930_defs.h"
#include "hardware_defs.h"

#include "CommIF.h"

/*****************************************************************************
 *  Global Variable Declarations
 *****************************************************************************/
/*! Global flag that holds the nSEL pin value */
SEGMENT_VARIABLE(fSelectState,        U8, SEG_DATA);
SEGMENT_VARIABLE(fSPIBusy,            U8, SEG_DATA);
SEGMENT_VARIABLE(fSPIDisabled,        U8, SEG_DATA);
SEGMENT_VARIABLE(fSMBusTransaction,   U8, SEG_DATA);

/*****************************************************************************
 *  Local variable Declarations
 *****************************************************************************/
SEGMENT_VARIABLE(wDelay,               U16, SEG_XDATA);
volatile SEGMENT_VARIABLE(lUartInternal,  tUartData, SEG_XDATA);

/*****************************************************************************
 *  Local Functions Declaration
 *****************************************************************************/
INTERRUPT_PROTO(UART_ISR,  INTERRUPT_UART0);

/*****************************************************************************
 *  Local Functions Definition
 *****************************************************************************/

/**
 *  Simple UART receive function.
 *
 * @param[out] byte
 *
 * @return
 *
 ******************************************************************************/
U8 Comm_IF_RecvUART(U8 * byte)
{
  if (lUartInternal.RXReadPosition != lUartInternal.RXWritePosition)
  {
    *byte = lUartInternal.RXBuffer[lUartInternal.RXReadPosition++];

    if (lUartInternal.RXReadPosition >= COMM_IF_UART_RX_BUFFER)
    {
      lUartInternal.RXReadPosition = 0u;
    }

    return TRUE;
  }

  return FALSE;
}

/**
 *  Simple UART send function.
 *
 * @param byte
 *
 * @return
 *
 ******************************************************************************/
U8 Comm_IF_SendUART(U8 byte)
{
  /* Check if buffer is full */
  if ((lUartInternal.TXReadPosition == lUartInternal.TXWritePosition) && (FALSE == lUartInternal.TXBufferEmpty))
  {
    return FALSE;
  }

  /* Write input data byte into buffer */
  lUartInternal.TXBuffer[lUartInternal.TXWritePosition++] = byte;

  /* Increment pointer */
  if (lUartInternal.TXWritePosition >= COMM_IF_UART_TX_BUFFER)
  {
    lUartInternal.TXWritePosition = 0u;
  }

  if (TRUE == lUartInternal.TXBufferEmpty)
  {
    SBUF0 = byte;
  }

  lUartInternal.TXBufferEmpty = FALSE;

  return TRUE;
}

/**
 *  Enable and set the UART0 peripheral.
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 ******************************************************************************/
void Comm_IF_EnableUART()
{
  /* Enable UART0 in Crossbar */
  XBR0 |= 0x01;

  /* Timer1 Init for UART baud rate */
  /* the default baud rate is 115200bps */
  Set115200bps_24MHZ5;
  TR1   = 1;                /* START Timer1 */
  TI0   = 1;                /* Transciever ready */

  /* UART init */
  SCON0 = 0x10;

  /* Enable UART0 IT */
  ES0   = TRUE;

  lUartInternal.TXBufferEmpty = TRUE;
}

/**
 *  Read/Write one byte to/from the SPI1 peripheral.
 *
 *  @param[in]  DataIn The data byte needs to be sent.
 *
 *  @return     Byte received from SPI1.
 *
 *  @note
 *
 ******************************************************************************/
U8 Comm_IF_Spi1ReadWrite(U8 DataIn)
{
   SEGMENT_VARIABLE(lTmp, U8 , SEG_DATA);

   /* clear SPIF */
   SPIF1 = 0;

   /* write register address */
   SPI1DAT = DataIn;

   /* wait on TXBMT */
   while(!TXBMT1);

   /* wait on SPIBSY */
   while((SPI1CFG & 0x80) == 0x80);

   /* read value */
   lTmp = SPI1DAT;

   SPIF1 = 0;

   return lTmp;
}

/**
 *  Writes one byte to the SPI1 peripheral and reads one byte at the same time.
 *
 *  @param[in] DataIn Data to be written.
 *
 *  @return    Data read from SPI1.
 *
 *  @note
 ******************************************************************************/
U8 Comm_IF_Spi1ReadWrite_Byte(U8 DataIn)
{
  SEGMENT_VARIABLE(ReadValue, U8, SEG_DATA);

  fSPIBusy = TRUE;

  /* select device */
  ReadValue = Comm_IF_Spi1ReadWrite(DataIn);

  fSPIBusy = FALSE;

  return ReadValue;
}

/**
 *  Read/Write 16 bits to the selected device.
 *
 *  @param[in] Address Address of the register.
 *
 *  @param[in] DataIn Data to be written.
 *
 *  @return    Data read from SPI1.
 *
 *  @note
 ******************************************************************************/
U16 Comm_IF_Spi1ReadWrite_Word(U8 Address, U8 DataIn)
{
  SEGMENT_VARIABLE(ReadValue, UU16, SEG_DATA);

  fSPIBusy = TRUE;

  /* select device */
  ReadValue.U8[MSB] = Comm_IF_Spi1ReadWrite(Address);
  ReadValue.U8[LSB] = Comm_IF_Spi1ReadWrite(DataIn);

  fSPIBusy = FALSE;

  return ReadValue.U16;
}

/**
 *  Read/Write several number of bytes into the selected device.
 *
 *  @param[in]  Address Starting address; MSB defines whether it is read or write!
 *  @param[in]  Length Number of registers have to be written.
 *  @param[in]  pIn Pointer to the source of the registers.
 *  @param[out] pOut Bytes read during transaction.
 *
 *  @note       Read or write bit has to be set!
 ******************************************************************************/
void Comm_IF_Spi1ReadWrite_Burst(U8 Address, U8 Length, U8 *pIn, U8 *pOut)
{
  SEGMENT_VARIABLE(ii, U8, SEG_DATA);

  fSPIBusy = TRUE;

  /* send the register address */
  Comm_IF_Spi1ReadWrite(Address);
  /* send register content */
  for (ii = 0u; ii < Length; ++ii) {
    *pOut++ = Comm_IF_Spi1ReadWrite(*pIn++);
  }

  fSPIBusy = FALSE;

}

/**
 *  Read/Write one byte to/from the SPI0 peripheral.
 *
 *  @param[in] data_in The data byte needs to be sent.
 *
 *  @return Byte received from SPI0.
 *
 *  @note
 *
 ******************************************************************************/
U8 Comm_IF_Spi0ReadWrite(U8 data_in)
{
  SEGMENT_VARIABLE(lTmp, U8 , SEG_DATA);

  /* clear SPIF */
  SPIF0 = 0;

  /* write register address */
  SPI0DAT = data_in;

  /* wait on TXBMT */
  while(!TXBMT0);

  /* wait on SPIBSY */
  while((SPI0CFG & 0x80) == 0x80);

  /* read value */
  lTmp = SPI0DAT;

  SPIF0 = 0;

  return lTmp;
}

/**
 *  Enable SPI1 and associate to XBAR.
 *
 *  @note
 *
 ******************************************************************************/
void Comm_IF_Spi1Enable(void)
{
  /* Enable SPI1 port */
  SPI1CN |= 0x01;

  /* Associate to the XBAR */
  XBR1 |= 0x40;

  /* set default state for pins */
  MCU_SCK = TRUE;
  MCU_MOSI = TRUE;

  fSPIDisabled = FALSE;
}

/**
 *  Disable SPI1 and disconnect from XBAR
 *
 *  @note
 *
 ******************************************************************************/
void Comm_IF_Spi1Disable(void)
{
  /* Disable the SPI0 port */
  SPI1CN &=0xFE;

  /* Disconnect from XBAR */
  XBR1 &= ~(0x40);

#if 0
  /* Sets the SPI1 pins in skip register */
  P1SKIP = 0xFF;
#endif

  /* set default state for the pins */
  MCU_SCK = FALSE;
  MCU_MOSI = TRUE;

  fSPIDisabled = TRUE;
}

/**
 *  Read one byte from the SPI1 using bit-bang method.
 *
 *  @return Read byte.
 *
 ******************************************************************************/
U8 Comm_IF_Spi1ReadByteBitbang(void)
{
  SEGMENT_VARIABLE(read_byte, U8, SEG_DATA) = 0u;
  SEGMENT_VARIABLE(ii,        U8, SEG_DATA);

  MCU_SCK = TRUE;

  NOP();
  NOP();
  NOP();
  NOP();

  MCU_SCK = FALSE;

  NOP();

  for(ii = 0u; ii < 8u; ii++)
  {
    MCU_SCK = TRUE;
    read_byte <<= 1u;

    /* Sample the line */
    if (TRUE == MCU_MISO)
    {
      read_byte |= 0x01;
    }
    MCU_SCK = FALSE;
  }

  return read_byte;
}

/**
 *  Write one byte to the SPI1 using bit-bang method.
 *
 *  @param[in] data_in Data byte to be sent.
 *  @param[in] nmbr_bit Number of bits to be sent from the data byte.
 *
 *  @note
 *
 ******************************************************************************/
void Comm_IF_Spi1WriteBitsBitbang(U8 data_in, U8 nmbr_bit)
{
  SEGMENT_VARIABLE(ii,     U8, SEG_DATA);
  SEGMENT_VARIABLE(lMask,  U8, SEG_DATA);

  lMask = (1u << (nmbr_bit - 1u));
  for (ii = 0u; ii < nmbr_bit; ii++)
  {
    MCU_SCK = FALSE;
    if ((data_in & lMask) != 0u)
    {
      MCU_MOSI = TRUE;
    }
    else
    {
      MCU_MOSI = FALSE;
    }
    /* clock pulse to sample */
    MCU_SCK = TRUE;
    lMask >>= 1u;
  }

  MCU_SCK = FALSE;
}


/**
 *  Pull down nSEL of the selected device.
 *
 *  @param[in] sel Device select.
 *
 *  @note Input: sel <br> 0 - \b DUT <br> 1 - \b LCD <br>
 *
 ******************************************************************************/
void Comm_IF_SpiClearNsel(eNSEL sel)
{
  switch (sel)
  {
  case RF_NSELECT:
    RF_NSEL = FALSE;
    break;

  case LCD_NSELECT:
    LCD_NSEL = FALSE;
    break;

  default:
    break;
  }
}

/**
 *  Pull-up nSEL of the selected device.
 *
 *  @param[in] sel Device select.
 *
 *  @note Input: sel <br> 0 - \b DUT <br> 1 - \b LCD <br>
 *
 ******************************************************************************/
void Comm_IF_SpiSetNsel(eNSEL sel)
{
  switch (sel)
  {
  case RF_NSELECT:
    RF_NSEL = TRUE;
    break;

  case LCD_NSELECT:
    LCD_NSEL = TRUE;
    break;

  default:
    break;
  }
}

/**
 *  Read one PRO2 debug register.
 *
 *  @param[in]  DebugCommand Debug Command.
 *
 *  @param[in]  AddressMSB  MSB 8bits of the debug reg. address.
 *
 *  @param[in]  AddressLSB LSB 8bits of the debug reg. address.
 *
 *  @return     Value read with bit-bang debug interface.
 *
 *  @note
 *
 ******************************************************************************/
U8 Comm_IF_Spi1DebugRead(U8 DebugCommand, U8 AddressMSB, U8 AddressLSB)
{
  SEGMENT_VARIABLE(lReadByte, U8, SEG_DATA);

  /* disable global IT */
  ENTER_CRITICAL_SECTION;

  /* disable HW SPI port */
  Comm_IF_Spi1Disable();

  /* send LSB address */
  Comm_IF_SpiClearNsel(0u);
  Comm_IF_Spi1WriteBitsBitbang(DebugCommand, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x02, 4u);
  Comm_IF_Spi1WriteBitsBitbang(AddressLSB, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x03, 2u);
  Comm_IF_SpiSetNsel(0u);

  /* send MSB address */
  Comm_IF_SpiClearNsel(0u);
  Comm_IF_Spi1WriteBitsBitbang(DebugCommand, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x03, 4u);
  Comm_IF_Spi1WriteBitsBitbang(AddressMSB, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x03, 2u);
  Comm_IF_SpiSetNsel(0u);

  /* wait cycle */
  Comm_IF_SpiClearNsel(0u);
  Comm_IF_Spi1WriteBitsBitbang(DebugCommand, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x01, 4u);
  Comm_IF_Spi1WriteBitsBitbang(0x00, 8u);     /* 0x01 */
  Comm_IF_Spi1WriteBitsBitbang(0x03, 2u);
  Comm_IF_SpiSetNsel(0u);

  /* send 8 clock before read */
  Comm_IF_SpiClearNsel(0u);
  Comm_IF_Spi1WriteBitsBitbang(DebugCommand, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x08, 4u);
  Comm_IF_Spi1WriteBitsBitbang(0xFF, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x01, 2u);

  /* wait 1us */
  DELAY_1US();
  DELAY_1US();
  DELAY_1US();
  DELAY_1US();
  DELAY_1US();

  lReadByte = Comm_IF_Spi1ReadByteBitbang();
  Comm_IF_SpiSetNsel(0u);

  /* enable HW SPI port */
  Comm_IF_Spi1Enable();

  /* enable global IT */
  EXIT_CRITICAL_SECTION;

  return lReadByte;
}

/**
 *  Read one PRO2 debug register (SFR).
 *
 *  @param[in]  DebugCommand Debug Command.
 *
 *  @param[in]  AddressMSB  MSB 8bits of the debug reg. address.
 *
 *  @param[in]  AddressLSB LSB 8bits of the debug reg. address.
 *
 *  @return     Value read with bit-bang debug interface.
 *
 *  @note
 *
 ******************************************************************************/
U8 Comm_IF_Spi1DebugReadSFR(U8 DebugCommand, U8 AddressMSB, U8 AddressLSB)
{
  SEGMENT_VARIABLE(lReadByte, U8, SEG_DATA);

  /* disable global IT */
  ENTER_CRITICAL_SECTION;

  /* disable HW SPI port */
  Comm_IF_Spi1Disable();

  /* send LSB address */
  Comm_IF_SpiClearNsel(0u);
  Comm_IF_Spi1WriteBitsBitbang(DebugCommand, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x02, 4u);
  Comm_IF_Spi1WriteBitsBitbang(AddressLSB, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x03, 2u);
  Comm_IF_SpiSetNsel(0u);

  /* send MSB address */
  Comm_IF_SpiClearNsel(0u);
  Comm_IF_Spi1WriteBitsBitbang(DebugCommand, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x03, 4u);
  Comm_IF_Spi1WriteBitsBitbang(AddressMSB, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x03, 2u);
  Comm_IF_SpiSetNsel(0u);

  /* wait cycle */
  Comm_IF_SpiClearNsel(0u);
  Comm_IF_Spi1WriteBitsBitbang(DebugCommand, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x01, 4u);
  Comm_IF_Spi1WriteBitsBitbang(0x00, 8u);     /* 0x01 */
  Comm_IF_Spi1WriteBitsBitbang(0x03, 2u);
  Comm_IF_SpiSetNsel(0u);

  /* send 8 clock before read */
  Comm_IF_SpiClearNsel(0u);
  Comm_IF_Spi1WriteBitsBitbang(DebugCommand, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x0A, 4u);
  Comm_IF_Spi1WriteBitsBitbang(0xFF, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x01, 2u);

  /* wait 1us */
  DELAY_1US();
  DELAY_1US();
  DELAY_1US();
  DELAY_1US();
  DELAY_1US();

  lReadByte = Comm_IF_Spi1ReadByteBitbang();
  Comm_IF_SpiSetNsel(0u);

  /* enable HW SPI port */
  Comm_IF_Spi1Enable();

  /* enable global IT */
  EXIT_CRITICAL_SECTION;

  return lReadByte;
}


/**
 *
 *  @param[in]  DebugCommand  Debug commnad.
 *
 *  @param[in]  AddressMSB MSB 8bits of the debug reg. address
 *
 *  @param[in]  AddressLSB LSB 8bits of the debug reg. address
 *
 *  @param[in]  Value The value needs to be written
 *
 *  @note
 *
 ******************************************************************************/
void Comm_IF_Spi1DebugWrite(U8 DebugCommand, U8 AddressMSB, U8 AddressLSB, U8 Value)
{
  /* disable global IT */
  ENTER_CRITICAL_SECTION;

  /* disable HW SPI port */
  Comm_IF_Spi1Disable();

  /* send LSB address */
  Comm_IF_SpiClearNsel(0u);
  Comm_IF_Spi1WriteBitsBitbang(DebugCommand, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x05, 4u);
  Comm_IF_Spi1WriteBitsBitbang(AddressLSB, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x03, 2u);
  Comm_IF_SpiSetNsel(0u);

  /* send MSB address */
  Comm_IF_SpiClearNsel(0u);
  Comm_IF_Spi1WriteBitsBitbang(DebugCommand, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x06, 4u);
  Comm_IF_Spi1WriteBitsBitbang(AddressMSB, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x03, 2u);
  Comm_IF_SpiSetNsel(0u);

  /* send data */
  Comm_IF_SpiClearNsel(0u);
  Comm_IF_Spi1WriteBitsBitbang(DebugCommand, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x08, 4u);
  Comm_IF_Spi1WriteBitsBitbang(Value, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x03, 2u);
  Comm_IF_SpiSetNsel(0u);

  /* enable HW SPI port */
  Comm_IF_Spi1Enable();

  /* enable global IT */
  EXIT_CRITICAL_SECTION;
}

/**
 *  Write one PRO2 debug register (SFR).
 *
 *  @param DebugCommand Debug command.
 *
 *  @param AddressMSB MSB 8bits of the debug reg. address.
 *
 *  @param AddressLSB LSB 8bits of the debug reg. address.
 *
 *  @param Value The value needs to be written.
 *
 ******************************************************************************/
void Comm_IF_Spi1DebugWriteSFR(U8 DebugCommand, U8 AddressMSB, U8 AddressLSB, U8 Value)
{
  /* disable global IT */
  ENTER_CRITICAL_SECTION;

  /* disable HW SPI port */
  Comm_IF_Spi1Disable();

  /* send LSB address */
  Comm_IF_SpiClearNsel(0u);
  Comm_IF_Spi1WriteBitsBitbang(DebugCommand, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x05, 4u);
  Comm_IF_Spi1WriteBitsBitbang(AddressLSB, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x03, 2u);
  Comm_IF_SpiSetNsel(0u);

  /* send MSB address */
  Comm_IF_SpiClearNsel(0u);
  Comm_IF_Spi1WriteBitsBitbang(DebugCommand, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x06, 4u);
  Comm_IF_Spi1WriteBitsBitbang(AddressMSB, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x03, 2u);
  Comm_IF_SpiSetNsel(0u);

  /* send data */
  Comm_IF_SpiClearNsel(0u);
  Comm_IF_Spi1WriteBitsBitbang(DebugCommand, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x0A, 4u);
  Comm_IF_Spi1WriteBitsBitbang(Value, 8u);
  Comm_IF_Spi1WriteBitsBitbang(0x03, 2u);
  Comm_IF_SpiSetNsel(0u);

  /* enable HW SPI port */
  Comm_IF_Spi1Enable();

  /* enable global IT */
  EXIT_CRITICAL_SECTION;
}

/**
 *  Initialize and enable the SMBus interface.
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 ******************************************************************************/
void Comm_IF_InitSMBusInterface()
{
  fSMBusTransaction = TRUE;

  /* SMBus Slave Inhibit, Clock from T1 overflow
   * This way only one timer is used. It gives approx. 77 kHz SMBus speed */
  SMB0CF = 0x41;

  /* set auto ACK bit */
  SMB0ADM |= 0x01;

  /* Enable SMBUS0 in CrossBar */
  XBR0    |= 0x04;

#if 0
  while (MCU_SDA == FALSE)
  {
    /* Provide clock pulses to allow the slave to advance out
     * of its current state. This will allow it to release SDA.       */

    MCU_SCL = FALSE;                       /* Drive the clock low     */
    for(ii = 0u; ii < 255u; ii++)  ;       /* Hold the clock low      */
    MCU_SCL = TRUE;                        /* Release the clock       */
    while (MCU_SCL == FALSE)  ;            /* Wait for open-drain     */
    for(ii = 0u; ii < 10u; ii++)  ;        /* Hold the clock high     */
  }
#endif

  EIE1 &= 0xFE;                            /* Disable SMBus interrupt */

  /* Clear flags */
  STA = FALSE;
  STO = FALSE;
  ACK = FALSE;
  /* clear IT flag */
  SI = FALSE;
  /* Enable SMBus peripheral */
  SMB0CF |= 0x80;

}

/**
 *  Disable the SMBus interface.
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 ******************************************************************************/
void Comm_IF_DisableSMBusInterface()
{
  /* Disable SMBus */
  STA = FALSE;
  STO = FALSE;
  ACK = FALSE;
  SMB0CF &= 0x7F;

  /* Clear IT flag */
  SI = FALSE;

  /* Disconnect SMBUS0 from CrossBar */
  XBR0 &= ~(0x04);

  /* Clear flag */
  fSMBusTransaction = FALSE;
}

/**
 *  Generate start condition on SMBus.
 *
 *  @note
 *
 ******************************************************************************/
void Comm_IF_SMBusStart()
{
  /* Set START bit */
  STA = TRUE;

  /* wait for the start condition */
  while (SI == FALSE);

  /* clear interrupt flag */
  SI = FALSE;
}

/**
 *  Write one byte to the SMBus.
 *
 *  @param[in] Data Data byte to send.
 *
 *  @return   SMBUS_NACK_RECEIVED <br>
 *            SMBUS_ACK_RECEIVED <br>
 *            SMBUS_ARBITRATION_LOST <br>
 *            SMBUS_GENERAL_ERROR <br>
 *
 *  @note
 *
 ******************************************************************************/
eSMBusReturnStates Comm_IF_SMBusWriteByte(U8 Data)
{
  if (TRUE == ARBLOST)
  {
    /* arbitration lost */
    ARBLOST = FALSE;

    return SMBUS_ARBITRATION_LOST;
  }
  else
  {
    /* START condition transmitted */
    if ((SMB0CN & 0xF0) == 0xE0)
    {
      /* load address and read/write bytes */
      SMB0DAT = Data;
      /* delete start bit */
      STA = FALSE;
      SI = FALSE;

      /* wait for send data */
      while (SI == FALSE);

      /* clear interrupt flag */
      SI = FALSE;

      /* check ACK */
      if (TRUE == ACK)
      {
        return SMBUS_ACK_RECEIVED;
      }

      return SMBUS_NACK_RECEIVED;
    }

    /* previous byte was transmitted */
    if ((SMB0CN & 0xF0) == 0xC0)
    {
      /* load data byte */
      SMB0DAT = Data;

      SI = FALSE;
      //wait for send data
      while (SI == FALSE);
      /* clear interrupt flag */
      SI = FALSE;

      /* check ACK */
      if (TRUE == ACK)
      {
        return SMBUS_ACK_RECEIVED;
      }

      return SMBUS_NACK_RECEIVED;
    }

    /* error happened */
    return SMBUS_GENERAL_ERROR;
  }
}

/**
 *  Read one byte from the SMBus. It also handles ACK / NACK and STOP condition.
 *
 *  @param[in] AckReq \b TRUE - If master wants to read more byte. <br>
 *                    \b FALSE - If this is the last byte to be read.
 *
 *  @return   Received byte.
 *
 *  @note
 *
 ******************************************************************************/
U8 Comm_IF_SMBusReadByte(U8 AckReq)
{
  U8 lTmp;

  /* clear IT flag */
  SI = FALSE;

  /* send ACK / NACK */
  ACK = AckReq;

  /* wait for IT */
  while (SI == FALSE) ;

  /* read data */
  lTmp = SMB0DAT;

  /* clear interrupt flag */
  SI = FALSE;

  /* send ACK / NACK */
  if (AckReq == FALSE)
  {
    /* send STOP */
    STO = TRUE;
  }

  return lTmp;
}

/**
 *  Generate stop condition on SMBus.
 *
 *  @note
 *
 ******************************************************************************/
void Comm_IF_SMBusStop(void)
{
  /* Stop SMBus transaction */
  STO = TRUE;
  SI = FALSE;
  while (STO == TRUE);
}

/**
 *  Send a given number of bytes on SMBus to the given address.
 *
 *  @param[in] Address 7bit slave address (7 MSB bit is used).
 *
 *  @param[in] Length Number of bytes to be sent.
 *
 *  @param[in] pData Pointer to data bytes to be sent.
 *
 *  @return    Result of the transaction, refer to CommIF.h
 *
 *  @note      The function blocks the flow until completion.
 *
 ******************************************************************************/
eSMBusReturnStates Comm_IF_SMBusWrite(U8 Address, U8 Length, U8 * pData)
{
  SEGMENT_VARIABLE(lTmp, U8, SEG_XDATA);

  if (fSMBusTransaction == FALSE)
  {
    /* enable SMBus */
    Comm_IF_InitSMBusInterface();

    /* START condition */
    STO = FALSE;
    ACK = FALSE;
    STA = TRUE;

    /* wait for START to be completed */
    if (Comm_IF_SMBusWaitForItWithTimeout() == FALSE)
    {
      /* timeout occurred */
      return SMBUS_TIMEOUT_ERROR;
    }

    /* send address byte */
    lTmp = ((Address << 1u) & 0xFE);
    SMB0DAT = lTmp;

    /* clear IT flag */
    SI = FALSE;
    /* delete start bit */
    STA = FALSE;
    /* wait address byte to be sent */
    if (Comm_IF_SMBusWaitForItWithTimeout() == FALSE)
    {
      /* timeout occured */
      return SMBUS_TIMEOUT_ERROR;
    }

    if (ACK == FALSE)
    {
      /* Slave address is not acknowledged */
      /* disable SMBus */
      Comm_IF_DisableSMBusInterface();

      return SMBUS_WRONG_SLAVE_ADDRESS;
    }

    /* send data bytes */
    for (lTmp = 0u; lTmp < Length; lTmp++)
    {
      /* send next data bytes */
      SMB0DAT = *pData++;

      /* clear IT flag */
      SI = FALSE;
      /* wait data to be sent */
      if (Comm_IF_SMBusWaitForItWithTimeout() == FALSE)
      {
        /* timeout occured */
        return SMBUS_TIMEOUT_ERROR;
      }

      if (ARBLOST == TRUE)
      {
        /* arbritation lost */
        /* disable SMBus */
        Comm_IF_DisableSMBusInterface();

        return SMBUS_ARBITRATION_LOST;
      }
    }

    /* all data byte sent */
    /* Stop SMBus transaction */
    STO = TRUE;
    SI = FALSE;
    while (STO == TRUE) ;

    /* disable SMBus */
    Comm_IF_DisableSMBusInterface();

    return SMBUS_TRANSMISSION_OK;
  }
  else
  {
    /* SMBus is already active */
    return SMBUS_BUSY;
  }
}

/**
 *  Read from the SMBus.
 *
 *  @param[in]  Address 7bit slave address (7 MSB bit is used)
 *
 *  @param[in]  Length Number of bytes to be read.
 *
 *  @param[out] pData Array of data bytes read from the slave.
 *
 *  @return     Result of the transaction, refer to CommIF.h.
 *
 *  @note       The function blocks the code until completion.
 *
 ******************************************************************************/
eSMBusReturnStates Comm_IF_SMBusRead(U8 Address, U8 Length, U8 * pData)
{
  SEGMENT_VARIABLE(lTmp, U8, SEG_XDATA);

  if (fSMBusTransaction == FALSE)
  {
    /* enable SMBus */
    Comm_IF_InitSMBusInterface();

    /* START condition */
    STO = FALSE;
    ACK = FALSE;
    STA = TRUE;

    /* wait for START to be completed */
    if (Comm_IF_SMBusWaitForItWithTimeout() == FALSE)
    {
      /* timeout occured */
      return SMBUS_TIMEOUT_ERROR;
    }

    /* send address byte */
    lTmp = ((Address << 1u) | 0x01);
    SMB0DAT = lTmp;

    /* clear IT flag */
    SI = FALSE;
    /* delete start bit */
    STA = FALSE;

    /* wait address byte to be sent */
    if (Comm_IF_SMBusWaitForItWithTimeout() == FALSE)
    {
      /* timeout occured */
      return SMBUS_TIMEOUT_ERROR;
    }

    if (ACK == FALSE)
    {
      /* Slave address is not acknowledged */
      /* disable SMBus */
      Comm_IF_DisableSMBusInterface();

      return SMBUS_WRONG_SLAVE_ADDRESS;
    }

    /* read data bytes */
    for (lTmp = 0u; lTmp < Length; lTmp++)
    {
      if (lTmp < Length - 1u)
      {
        /* set ACK */
        ACK = TRUE;
      }
      else
      {
        /* set NACK */
        ACK = FALSE;
      }

      /* clear IT flag */
      SI = FALSE;
      /* wait for receiving the next byte */
      if (Comm_IF_SMBusWaitForItWithTimeout() == FALSE)
      {
        /* timeout occured */
        return SMBUS_TIMEOUT_ERROR;
      }

      /* save next data bytes */
      *pData++ = SMB0DAT;
    }

    /* all data byte read */
    /* Stop SMBus transaction */
    STO = TRUE;
    SI = FALSE;
    while (STO == TRUE) ;

    /* disable SMBus */
    Comm_IF_DisableSMBusInterface();

    return SMBUS_RX_FINISHED;
  }
  else
  {
    /* SMBus is active */
    return SMBUS_BUSY;
  }
}

/**
 *  Wait for the SMBus interrupt with timeout (~2.6ms).
 *
 *  @return  \b FALSE - Timeout occurred without IT. <br>
 *           \b TRUE - IT occurred within Timeout.
 *
 *  @note
 *
 ******************************************************************************/
U8 Comm_IF_SMBusWaitForItWithTimeout()
{
  SEGMENT_VARIABLE(SMBusTimeOut, U16, SEG_XDATA);

  /* reset timeout */
  SMBusTimeOut = 0u;

  /* wait for the IT */
  do
  {
    SMBusTimeOut++;
  } while ((SMBusTimeOut < 0xFFFE) && (SI == FALSE))  ;

  if (SI == TRUE)
  {
    return TRUE;
  }
  else
  {
    /* timeout occured, disable SMBus */
    Comm_IF_DisableSMBusInterface();
    /* return with error */
    return FALSE;
  }
}

/**
 *  Interrupt handler for UART0 peripheral.
 *
 *  @author Sz. Papp
 *
 *  @note   Receive Overrun may occur as it not protected against it yet.
 *
 *****************************************************************************/
INTERRUPT(UART_ISR, INTERRUPT_UART0)
{
  if (TI0)
  {
    /* Transmit Interrupt */
    TI0 = FALSE;

    lUartInternal.TXReadPosition++;
    if (lUartInternal.TXReadPosition >= COMM_IF_UART_TX_BUFFER)
    {
      lUartInternal.TXReadPosition = 0u;
    }

    if (lUartInternal.TXReadPosition == lUartInternal.TXWritePosition)
    {
      lUartInternal.TXBufferEmpty = TRUE;
    }
    else
    {
      SBUF0 = lUartInternal.TXBuffer[lUartInternal.TXReadPosition];
    }
  }

  if (RI0)
  {
    /* Receive Interrupt */
    RI0 = FALSE;

    lUartInternal.RXBuffer[lUartInternal.RXWritePosition++] = SBUF0;

    if (lUartInternal.RXWritePosition >= COMM_IF_UART_RX_BUFFER)
    {
      lUartInternal.RXWritePosition = 0u;
    }
  }
}
