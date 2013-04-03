/**
 *  Copyright 2008 Silicon Laboratories, Inc.
 *  http://www.silabs.com
 *
 *  @file SoftTimer.c
 *  
 *  C File Description:
 *  @brief TODO
 *
 *  Project Name: dev_EzR2_Loadboard 
 * 
 * 
 *  @author Sz. Papp
 *
 *  @date 2012.03.07.
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

/*! AES Demo header */
#include "ReceiverDemo.h"

#include "SoftTimer.h"

/*****************************************************************************
 *  Global Variable Definitions
 *****************************************************************************/
SEGMENT_VARIABLE(SoftTimer_Channels[SOFTTIMER_NUMOF_CHANNELS], tSoftTimer, SEG_XDATA);


/*****************************************************************************
 *  Local Functions
 *****************************************************************************/
void StartTimer2(void);
void StopTimer2(void);

/**
 *  Adds a software timer channel into array.
 *
 *  @param Ticks Amount of time.
 *
 *  @param Channel Channel number.
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 *****************************************************************************/
void SoftTimer_Add(U16 Ticks, eSoftTimerChannels Channel)
{
  if ((Channel < SOFTTIMER_NUMOF_CHANNELS) && (Ticks > 0u))
  {
    SoftTimer_Channels[Channel].Elapsed = FALSE;
    SoftTimer_Channels[Channel].Ticks   = ++Ticks;

    if (FALSE == TR2)
    {
      StartTimer2();
    }
  }
}

/**
 *  Removes a software timer channel from array.
 *
 *  @param Channel
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 *****************************************************************************/
void SoftTimer_Remove(eSoftTimerChannels Channel)
{
  U8 ii;
  BIT lActive = FALSE;

  if (Channel < SOFTTIMER_NUMOF_CHANNELS)
  {
    SoftTimer_Channels[Channel].Elapsed = FALSE;
    SoftTimer_Channels[Channel].Ticks   = 0u;
  }

  /* Check for running Channels */
  if (TRUE == TR2)
  {
    for (ii = 0u; ii < SOFTTIMER_NUMOF_CHANNELS; ii++)
    {
      if (SoftTimer_Channels[ii].Ticks > 0u)
      {
        lActive = TRUE;
      }
    }

    if (FALSE == lActive)
    {
      StopTimer2();
    }
  }
}

/**
 *  Checks if a software timer elapsed.
 *
 *  @param Channel
 *
 *  @return True/False
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 *****************************************************************************/
U8 SoftTimer_Elapsed(eSoftTimerChannels Channel)
{
  U8 ii;
  BIT lActive = FALSE;

  /* Check for running Channels */
  if (TRUE == TR2)
  {
    for (ii = 0u; ii < SOFTTIMER_NUMOF_CHANNELS; ii++)
    {
      if (SoftTimer_Channels[ii].Ticks > 0u)
      {
        lActive = TRUE;
      }
    }

    if (FALSE == lActive)
    {
      StopTimer2();
    }
  }

  if (Channel < SOFTTIMER_NUMOF_CHANNELS)
  {
    if (TRUE == SoftTimer_Channels[Channel].Elapsed)
    {
      SoftTimer_Channels[Channel].Elapsed = FALSE;

      return TRUE;
    }

    return FALSE;
  }

  return TRUE;
}

/**
 *  Interrupt handler for Timer2 overflow.
 *
 *  @author Sz. Papp
 *
 *  @note   Keep the Channel count low!
 *
 *****************************************************************************/
INTERRUPT(Timer2ISR, INTERRUPT_TIMER2)
{
  U8 ii;

  /* Reset IT Flag */
  TF2H = FALSE;

  for (ii = 0u; ii < SOFTTIMER_NUMOF_CHANNELS; ii++)
  {
    if (SoftTimer_Channels[ii].Ticks > 1u)
    {
      SoftTimer_Channels[ii].Ticks--;
    }
    else
    {
      SoftTimer_Channels[ii].Elapsed = TRUE;
      SoftTimer_Channels[ii].Ticks = 0u;
    }
  }
}

/**
 *  Starts Timer2 HW peripheral & sets timeout to 10ms.
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 *****************************************************************************/
void StartTimer2()
{
  /* Set T2 to run from SYSCLK/12 */
  CKCON &= ~(1u << 4u);

  /* Reset Timer2 flags, Stop mode, Clock is SYSCLK/12 */
  TMR2CN = 0u;

  /* Set Reload value */
  TMR2RL = (0xFFFF - (10u * (SYS_CLK_KHZ / 12u)));

  /* Enable Timer2 IT */
  ET2 = TRUE;

  /* Start Timer2 */
  TR2 = TRUE;
}

/**
 *  Stops Timer3 HW peripheral.
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 *****************************************************************************/
void StopTimer2()
{
  /* Disable Timer2 IT */
  ET2 = FALSE;

  /* Stop Timer2 */
  TR2 = FALSE;

  /* Reset Timer2 flags, Stop mode, Clock is SYSCLK/12 */
  TMR2CN = 0u;
}

/**
 *  Starts Timer3 HW peripheral
 *
 *  @param Reload Timer Reload Value
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 *****************************************************************************/
void StartTimer3(U16 Reload)
{
  /* Set T3 to run from SYSCLK */
  CKCON &= 0x3F;
  CKCON |= 0x40;

  /* Reset Timer3 flags, Stop mode, AutoReload */
  TMR3CN = 0u;

  /* Set Reload value */
  TMR3RL = Reload;

  /* Enable Timer3 IT */
  EIE1 |= 0x80;

  /* Start Timer3 */
  TMR3CN |= 0x04;
}

/**
 *  Stops Timer3 HW peripheral.
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 *****************************************************************************/
void StopTimer3()
{
  /* Disable Timer2 IT */
  EIE1 &= ~0x80;

  /* Stop Timer3 */
  TMR3CN &= ~0x04;

  /* Reset Timer3 flags, Stop mode, AutoReload */
  TMR3CN = 0u;
}

/**
 *  Interrupt handler for Timer3 overflow.
 *
 *  @author Sz. Papp
 *
 *  @note   Used to generate buzzer variable frequency signal.
 *
 *****************************************************************************/
INTERRUPT(Timer3ISR, INTERRUPT_TIMER3)
{
  /* Clear Timer3 IT Flags */
  TMR3CN &= ~0xC0;

  /* Change Buzzer Output pin state */
  BUZZ = !BUZZ;

  if (0u == --ReceiverDemo_InternalData.ReceiverDemo_BeepLength)
  {
    EIE1 &= ~0x80;

    /* Stop Timer3 */
    TMR3CN &= ~0x04;

    /* Reset Timer3 flags, Stop mode, AutoReload */
    TMR3CN = 0u;
  }
}
