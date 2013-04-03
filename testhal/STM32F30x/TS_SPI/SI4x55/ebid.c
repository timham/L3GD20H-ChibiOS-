/**
 *  Copyright 2008 Silicon Laboratories, Inc.
 *  http://www.silabs.com
 *
 *  @file ebid.c
 *  
 *  C File Description:
 *  @brief TODO
 *
 *  Project Name: dev_EzR2LCD_AESDemo 
 * 
 * 
 *  @author Sz. Papp
 *
 *  @date 2012.03.30.
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
/*! Compiler definitions */
#include "compiler_defs.h"

/*! MCU related */
#include "C8051F930_defs.h"

/*! Hardware related */
#include "hardware_defs.h"

/*! Communication interface include */
#include "CommIF.h"

/*! EBID handler functions include */
#include "ebid.h"

/*****************************************************************************
 *  Local Variable Definitions
 *****************************************************************************/
SEGMENT_VARIABLE(EBIDInfo, tEBIDInfo, SEG_XDATA);

/*****************************************************************************
 *  Local Macro Definitions
 *****************************************************************************/
#if (MSB == 1)
/* Little-endian compiler */
#define SMBUS_ADDRESS_ENDIAN(X) (((U16) ((U16) (X) << 8u) & 0xFF00) |\
                                  ((U8) ((U16) (X) >> 8u) & 0xFF))
#else
#define SMBUS_ADDRESS_ENDIAN(X) (X)
#endif

/*****************************************************************************
 *  Global Functions
 *****************************************************************************/

/**
 *  Check if EBID available (RFPico connected)
 *
 *  @return   TRUE - EBID present / FALSE - EBID info not available.
 *
 *  @author   Sz. Papp
 *
 *  @note
 *
 *****************************************************************************/
U8 EBID_CheckAvailability()
{
  U8 lTemp;

  /* Set eeprom pointer to 0 */
  EBIDInfo.EBID_EEPromPointer.U16 = 0u;

  if (Comm_IF_SMBusWrite(EBID_SMBUS_ADDRESS, 2u, (U8 *) &EBIDInfo.EBID_EEPromPointer) != SMBUS_TRANSMISSION_OK)
  {
    return FALSE;
  }

  /* Read first byte */
  if (Comm_IF_SMBusRead(EBID_SMBUS_ADDRESS, 1u, &lTemp) != SMBUS_RX_FINISHED)
  {
    return FALSE;
  }

  if ( (0x01 == lTemp) || /* Board Record */
       (0x02 == lTemp) || /* MCU Record */
       (0x40 == lTemp) )  /* Radio Record */
  {
      return TRUE;
  }

  return FALSE;
}

/**
 *  Check if Radio record present in EBID. If yes, fills the EBIDInfo struct.
 *
 *  @return   NULL - Not Present / Pointer to BoardName string.
 *
 *  @author   Sz. Papp
 *
 *  @note     Affects the EBID_EEPromPointer variable.
 *
 *****************************************************************************/
U8 EBID_SearchRadioRecord()
{
  /*! Local variable to get rid of endianness problem */
  U16 lPointer;

  U8 lTemp = 0u;
  U16 lLength;

  /* Set eeprom pointer to 0 */
  EBIDInfo.EBID_EEPromPointer.U16 = 0u;

  /* Iterate further through eeprom */
  while (0xFF != lTemp)
  {
    lPointer = SMBUS_ADDRESS_ENDIAN(EBIDInfo.EBID_EEPromPointer.U16);
    if (Comm_IF_SMBusWrite(EBID_SMBUS_ADDRESS, 2u, (U8 *) &lPointer) != SMBUS_TRANSMISSION_OK) return FALSE;

    /* Read record first byte */
    if (Comm_IF_SMBusRead(EBID_SMBUS_ADDRESS, 1u, &lTemp) != SMBUS_RX_FINISHED) return FALSE;

    if (0x40 == lTemp)
    {
      /* Read out BoardName string */
      EBIDInfo.EBID_EEPromPointer.U16 += 0x1B;
      lPointer = SMBUS_ADDRESS_ENDIAN(EBIDInfo.EBID_EEPromPointer.U16);
      if (Comm_IF_SMBusWrite(EBID_SMBUS_ADDRESS, 2u, (U8 *) &lPointer) != SMBUS_TRANSMISSION_OK) return FALSE;

      if (Comm_IF_SMBusRead(EBID_SMBUS_ADDRESS, EBID_BOARDNAME_LENGTH, EBIDInfo.EBID_BoardName) != SMBUS_RX_FINISHED) return FALSE;

      /* Read out Frequency Band word */
      EBIDInfo.EBID_EEPromPointer.U16 += 0x28;
      lPointer = SMBUS_ADDRESS_ENDIAN(EBIDInfo.EBID_EEPromPointer.U16);
      if (Comm_IF_SMBusWrite(EBID_SMBUS_ADDRESS, 2u, (U8 *) &lPointer) != SMBUS_TRANSMISSION_OK) return FALSE;

      if (Comm_IF_SMBusRead(EBID_SMBUS_ADDRESS, 2u, (U8 *) &EBIDInfo.EBID_Freqband) != SMBUS_RX_FINISHED) return FALSE;
      EBIDInfo.EBID_Freqband = SMBUS_ADDRESS_ENDIAN(EBIDInfo.EBID_Freqband);

      return TRUE;
    }

    /* Not MCU and Board record? */
    if ((0x01 != lTemp) && (0x02 != lTemp))
    {
      return FALSE;
    }

    /* Get length of this record */
    EBIDInfo.EBID_EEPromPointer.U16 += 0x0E;
    lPointer = SMBUS_ADDRESS_ENDIAN(EBIDInfo.EBID_EEPromPointer.U16);
    if (Comm_IF_SMBusWrite(EBID_SMBUS_ADDRESS, 2u, (U8 *) &lPointer) != SMBUS_TRANSMISSION_OK) return FALSE;

    if (Comm_IF_SMBusRead(EBID_SMBUS_ADDRESS, 2u, (U8 *) &lLength) != SMBUS_RX_FINISHED) return FALSE;
    lLength = SMBUS_ADDRESS_ENDIAN(lLength);

    EBIDInfo.EBID_EEPromPointer.U16 += lLength + 2u;
  }

  return FALSE;
}
