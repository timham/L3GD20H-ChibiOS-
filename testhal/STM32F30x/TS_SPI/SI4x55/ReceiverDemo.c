/**
 *  Copyright 2008 Silicon Laboratories, Inc.
 *  http://www.silabs.com
 *
 *  @file ReceiverDemo.c
 *  
 *  C File Description:
 *  @brief TODO
 *
 *  Project Name: dev_EzR2_ReceiverDemo
 * 
 * 
 *  @author Sz. Papp
 *
 *  @date 2012.04.02.
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
#include <STDIO.H>

/*! Compiler definitions */
#include "compiler_defs.h"

/*! MCU related */
#include "C8051F930_defs.h"

/*! Hardware related */
#include "hardware_defs.h"

/*! AES Demo header */
#include "ReceiverDemo.h"

/*! Software Timer module include */
#include "SoftTimer.h"

/*! EBID handler functions include */
#include "ebid.h"

/*! Radio functions */
#include "Radio.h"

/*! Radio driver includes */
#include "Radio/Si4455/si4455_api_lib.h"
#include "Radio/Si4455/si4455_defs.h"
#include "Radio/radio_comm.h"
#include "Radio/radio_hal.h"

/*****************************************************************************
 *  Local macros & Defines
 *****************************************************************************/
/*! Error hook */
#define ERROR_HOOK                while (TRUE) \
                                  { \
                                    for (RadioPacket.RollingCounter = 0u; \
                                         RadioPacket.RollingCounter < 0xFFFF; \
                                         RadioPacket.RollingCounter++)  ; \
                                         LED4 = !LED4; \
                                  }


/*****************************************************************************
 *  Global Variables
 *****************************************************************************/
SEGMENT_VARIABLE(ReceiverDemo_InternalData, tReceiverDemo, SEG_XDATA);
SEGMENT_VARIABLE(RadioPacket,                tRadioPacket, SEG_XDATA);

/*****************************************************************************
 *  Local Variables
 *****************************************************************************/

/*****************************************************************************
 *  Local Functions
 *****************************************************************************/

/*****************************************************************************
 *  Function definitions
 *****************************************************************************/

/**
 *  Receiver Demo application poll handler.
 *
 *  @author Sz. Papp
 *
 *  @note   Must be called periodically.
 *
 *****************************************************************************/
void ReceiverDemo_Pollhandler()
{
  /* Receiver Demo state machine */
  switch (ReceiverDemo_InternalData.ReceiverDemo_State)
  {

  case SM_START:
    /* Check RF Pico board EBID */

    if (EBID_CheckAvailability() == TRUE)
    {
      if (EBID_SearchRadioRecord() != FALSE)
      {
        /* EBIDInfo filled */

        if (EBIDInfo.EBID_Freqband & (1u << 9u))
        {
          /* 917.00 MHz */
          pEzConfigArray = pEzConfigArray_917M00_FSK;
        }
        else if (EBIDInfo.EBID_Freqband & (1u << 8u))
        {
          /* 868.30 MHz */
          pEzConfigArray = pEzConfigArray_868M30_FSK;
        }
        else if (EBIDInfo.EBID_Freqband & (1u << 4u))
        {
          /* 433.92 MHz */
          pEzConfigArray = pEzConfigArray_433M92_FSK;
        }
        else if (EBIDInfo.EBID_Freqband & (1u << 3u))
        {
          /* 316.66 MHz */
          pEzConfigArray = pEzConfigArray_316M66_FSK;
        }
        else
        {
          /* Waiting for Reset */
          ERROR_HOOK;
        }

        ReceiverDemo_InternalData.ReceiverDemo_State = SM_STEP1;
        break;
      }
    }

    /* Waiting for Reset */
    ERROR_HOOK;
    break;

  case SM_STEP1:
    /* RF Pico Board detected */

    /* Set GPIOs HiZ & Load PartInfo */
    vRadio_SetChip();

    /* Check the Radio Part number */
    if ((Si4455Cmd.PART_INFO.PART.U16 != 0x4455) && (Si4455Cmd.PART_INFO.PART.U16 != 0x4355))
    {
      /* Waiting for Reset */
      ERROR_HOOK;
    }

    ReceiverDemo_InternalData.ReceiverDemo_State = SM_STEP2;
    break;

  case SM_STEP2:
    /* Radio armed */

    Radio_StartRX(0u);
    /* Reset RX FIFO */
    si4455_fifo_info(0x02);

    ReceiverDemo_InternalData.ReceiverDemo_State = SM_STEP3;
    break;

  case SM_STEP3:
    /* Wait for incoming RF frame */

    /* Incoming RF Packet */
    if (gRadio_CheckReceived() == TRUE)
    {
      /* RF Packet received */

      if (RadioPacket.Flags&(1u << 4u))
      {
        LED1 = ILLUMINATE;
        LED3 = ILLUMINATE;
        /* Start LED timers */
        SoftTimer_Add(10u, SOFTTIMER_CH0);
        SoftTimer_Add(10u, SOFTTIMER_CH2);
      }
      if (RadioPacket.Flags&(1u << 3u))
      {
        LED1 = ILLUMINATE;
        LED2 = ILLUMINATE;
        LED3 = ILLUMINATE;
        /* Start LED timer */
        SoftTimer_Add(10u, SOFTTIMER_CH0);
        SoftTimer_Add(10u, SOFTTIMER_CH1);
        SoftTimer_Add(10u, SOFTTIMER_CH2);
      }
      if (RadioPacket.Flags&(1u << 2u))
      {
        LED1 = ILLUMINATE;
        /* Start LED timer */
        SoftTimer_Add(10u, SOFTTIMER_CH0);
      }
      if (RadioPacket.Flags&(1u << 1u))
      {
        LED2 = ILLUMINATE;
        /* Start LED timers */
        SoftTimer_Add(10u, SOFTTIMER_CH1);
      }
      if (RadioPacket.Flags&(1u << 0u))
      {
        LED3 = ILLUMINATE;
        /* Start LED timer */
        SoftTimer_Add(10u, SOFTTIMER_CH2);
      }

      ReceiverDemo_InternalData.ReceiverDemo_State = SM_STEP2;
    }

    if ((LED1 == ILLUMINATE) && (SoftTimer_Elapsed(SOFTTIMER_CH0) == TRUE))
    {
      LED1 = EXTINGUISH;
    }
    if ((LED2 == ILLUMINATE) && (SoftTimer_Elapsed(SOFTTIMER_CH1) == TRUE))
    {
      LED2 = EXTINGUISH;
    }
    if ((LED3 == ILLUMINATE) && (SoftTimer_Elapsed(SOFTTIMER_CH2) == TRUE))
    {
      LED3 = EXTINGUISH;
    }
    break;

  default:
    /* Unknown state */

    /* ReInit */
    ReceiverDemo_Init();

    ReceiverDemo_InternalData.ReceiverDemo_State = SM_START;
    break;
  }
}

/**
 *  Initializes AESDemo internal variables
 *
 *  @note
 *
 *****************************************************************************/
void ReceiverDemo_Init()
{
  ReceiverDemo_InternalData.ReceiverDemo_State = SM_START;
}
