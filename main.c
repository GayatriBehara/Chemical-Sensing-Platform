
/******************************************************************************
 * @file
 * @brief Backup power domain and backup real time counter application note
 *****************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_rmu.h"
#include "em_burtc.h"
#include "clock.h"
#include "clockApp_dk.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "em_adc.h"
#include "rtcdrv.h"
#include "retargetserial.h"
#include "em_gpio.h"
#include "glib.h"
#include "retargetserialconfig.h"
#include "em_pcnt.h"
#include "em_system.h"
#include "em_adc.h"
#include "em_timer.h"
#include "em_letimer.h"
#include <stdlib.h>
#include <string.h>
/* Standard C header files */
#include <stdint.h>
/* EM types */
#include "em_types.h"
/* GLIB header files */
#include "glib.h"
#include "glib_font.h"
#include "glib_color.h"
/** Interrupt pin used to detect joystick activity */
#define GPIO_INT_PIN    0
/** Interrupt pin used to detect DVK button/joystick activity */
#define GPIO_INT_PORT    gpioPortE
/* TFT defines */
#define LINE_HEIGHT    8 /* Line height (in pixels) */
#define CHAR_WIDTH    8 /* Character width (in pixels) */
/* Setting TOP value to 14 so that the frequency is around 50khz
  F = HFPERCLK / ( 2^(PRESC + 1) x (TOP + 1))
  HFPERCLK=48Mhz AND PRESC=5  */
#define TOP 14


/* Declare variables */
volatile uint32_t  pulseCount=0;
uint32_t  diffPulseCnt=0;
uint32_t  prevPulseCnt=0;
uint32_t  pulseCountOverflow=0;
uint32_t  gTemperature;
uint32_t   extemp;
uint32_t   voltage;
uint32_t sample;
uint32_t   voltage1;
uint32_t sample1;
extern bool displayUpdate = true;/* Declare variables for TFT output*/
/** Flag used to indicate if displaying in Celsius or Fahrenheit */
static int showFahrenheit;
/* Declare variables */
static uint32_t resetcause = 0;
/* Declare BURTC variables */
static uint32_t burtcCountAtWakeup = 0;
/* Calendar struct for initial date setting */
struct tm initialCalendar;
/* Local Prototypes */
void temperatureIRQInit(void);
void gpioSetup(void);
void ExtTempSetup(void);
void ADCConfig(void);
float convertToCelsius(uint32_t adcSample);
float convertToFahrenheit(uint32_t adcSample);
float convertToCelsius1(uint32_t adcSample);
float convertToFahrenheit1(uint32_t adcSample);
/* Function prototypes */
void budSetup( void );
void burtcSetup( void );
void ExtBattSensor(void);
float convertToCelsius(uint32_t adcSample);
float convertToFahrenheit(uint32_t adcSample);
float convertToCelsius1(uint32_t adcSample);
float convertToFahrenheit1(uint32_t adcSample);

/**************************************************************************//**
 * @brief PCNT1_IRQHandler
 * Interrupt Service Routine for PCNT1 Interrupt Line
 *****************************************************************************/
void PCNT1_IRQHandler(void)
{

  /* Clear PCNT1 overflow interrupt flag */
  PCNT_IntClear(PCNT1, PCNT_IF_OF);

  /* Update the number of pulses on LCD
     The value of TOP is written instead of
     count because the overflow occurs when
     CNT goes from TOP to 0 */
  pulseCountOverflow++;

}

/**************************************************************************//**
 * @brief GPIO Interrupt handler
 * This interrupt handler is not an example of good design, as it will do
 * a lot of operations inside the interrupt handler.
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  uint16_t joystick;

  /* Clear interrupt */
  BSP_InterruptFlagsClear(BC_INTEN_JOYSTICK);
  GPIO_IntClear(1 << GPIO_INT_PIN);

  /* LEDs on to indicate joystick used */
  BSP_LedsSet(0xffff);

  /* Read and store joystick activity - wait for key release */
  joystick = BSP_JoystickGet();
  while (BSP_JoystickGet()) ;

  /* LEDs off to indicate joystick release */
  BSP_LedsSet(0x0000);

  /* Push toggles celsius/fahrenheit */
  if (joystick & BC_UIF_JOYSTICK_CENTER)
  {
    showFahrenheit ^= 1;
  }
}

/**************************************************************************//**
 * @brief Initialize GPIO interrupt for joystick (ie FPGA signal)
 *****************************************************************************/
void temperatureIRQInit(void)
{
  /* Configure interrupt pin as input with pull-up */
  GPIO_PinModeSet(gpioPortE, GPIO_INT_PIN, gpioModeInputPull, 1);

  /* Set falling edge interrupt and clear/enable it */
  GPIO_IntConfig(gpioPortE, GPIO_INT_PIN, false, true, true);

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

/**************************************************************************//**
 * @brief ADC0 interrupt handler. Simply clears interrupt flag.
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  ADC_IntClear(ADC0, ADC_IF_SINGLE);
}


/**************************************************************************//**
 * @brief Initialize ADC for temperature sensor readings in single point
 *****************************************************************************/
void ExtTempSetup(void)
{
  /* Base the ADC configuration on the default setup. */
  ADC_Init_TypeDef       init  = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef sInit = ADC_INITSINGLE_DEFAULT;

  /* Initialize timebases */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(400000, 0);

  ADC_Init(ADC0, &init);

  /* Set input to temperature sensor. Reference must be 1.25V */
  sInit.reference = adcRef1V25;
  sInit.input     = adcSingleInpCh6;

  ADC_InitSingle(ADC0, &sInit);

  /* Setup interrupt generation on completed conversion. */
  ADC_IntEnable(ADC0, ADC_IF_SINGLE);
  NVIC_EnableIRQ(ADC0_IRQn);
}


/**************************************************************************//**
 * @brief Convert ADC sample values to celsius.
 * @note See section 2.3.4 in the reference manual for details on this
 *       calculatoin
 * @param adcSample Raw value from ADC to be converted to celsius
 * @return The temperature in degrees Celsius.
 *****************************************************************************/
float convertToCelsius(uint32_t adcSample)
{
  float temp;
  /* Factory calibration temperature from device information page. */
  float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
                             >> _DEVINFO_CAL_TEMP_SHIFT);

  float cal_value_0 = (float)((DEVINFO->ADC0CAL2
                               & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)
                              >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

  /* Temperature gradient (from datasheet) */
  float t_grad = -6.27;

  temp = cal_temp_0 - ((cal_value_0 - adcSample) / t_grad);

  return temp;
}

/**************************************************************************//**
 * @brief Convert ADC sample values to fahrenheit
 * @param adcSample Raw value from ADC to be converted to fahrenheit
 * @return The temperature in degrees Fahrenheit
 *****************************************************************************/
float convertToFahrenheit(uint32_t adcSample)
{
  float celsius;
  float fahrenheit;
  celsius = convertToCelsius(adcSample);

  fahrenheit = (celsius * (9.0 / 5.0)) + 32.0;

  return fahrenheit;
}

/***************************************************************************//**
* @brief
*   Configure ADC usage for measuring external temparature sensor with oversampling.
*******************************************************************************/
 void ExtBattSensor(void)
{
  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef sInit = ADC_INITSINGLE_DEFAULT;

  /* Init common settings for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  /* Might as well finish conversion as quickly as possibly since polling */
  /* for completion. */
  /* Set ADC clock to 7 MHz, use default HFPERCLK */
  init.prescale = ADC_PrescaleCalc(4000000, 0);


  /* WARMUPMODE must be set to Normal according to ref manual before */
  /* entering EM2. In this example, the warmup time is not a big problem */
  /* due to relatively infrequent polling. Leave at default NORMAL, */

  ADC_Init(ADC0, &init);

  /* Init for single conversion use, measure VDD/3 with 1.25 reference. */
  sInit.reference = adcRef1V25;
  sInit.input     = adcSingleInpCh7;



  ADC_InitSingle(ADC0, &sInit);
}

/******************************************************************************
 * @brief  Main function
 *
 *****************************************************************************/


int main( void )
{
  /* Initialize chip - handle erratas */
  CHIP_Init();
  uint8_t prod_rev;
  uint32_t temp_offset;

  /* Initialize DK board register access */
  BSP_Init(BSP_INIT_DEFAULT);



  /* Read and clear RMU->RSTCAUSE as early as possible */
  /*   RMU_ResetCauseGet() cannot yet (as of emlib version 2.4.1) be used as it masks out RMU_RSTCAUSE_BUMODERESET.
       This will be fixed in an upcoming release of emlib */
  resetcause = RMU_ResetCauseGet();
  resetcause = RMU->RSTCAUSE;
  RMU_ResetCauseClear();

  /* Enable clock to low energy modules */
  CMU_ClockEnable(cmuClock_CORELE,true);
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_ADC0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_PCNT1, true);      /* Enable clock for PCNT module */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
  CMU_ClockEnable(cmuClock_TIMER0, true);   /* Enable clock for TIMER0 module */


  /* Set CC0 location 3 pin (PD1) as output */
  GPIO_PinModeSet(gpioPortD, 1, gpioModePushPull, 0);

  /* Configure PC5 as pushpull with output high
     This pin is located next to the input pin for
     the PCNT, so the user can make contact between
     the two pins to generate pulses */
  GPIO_PinModeSet(gpioPortC, 5, gpioModePushPull, 1);

  /* Configure PC4 as input to drive pulse counter */
  GPIO_PinModeSet(gpioPortC, 4, gpioModeInputPull, 0);

  /* Set configuration for pulse counter */







  PCNT_Init_TypeDef
    pcntInit =
  {
    .mode       = pcntModeExtSingle,  /* clocked by LFACLK */
    .counter    = 0,                  /* Set initial value to 0 */
    .top        = 100,                 /* Set top to max value */
    .negEdge    = false,              /* positive edges */
    .countDown  = false,              /* up count */
    .filter     = true,               /* filter enabled */
  };

  /* Initialize Pulse Counter */
  PCNT_Init(PCNT1, &pcntInit);

  /* Enable PCNT overflow interrupt */
  PCNT_IntEnable(PCNT1, PCNT_IF_OF);

  /* Enable PCNT1 interrupt vector in NVIC */
  NVIC_EnableIRQ(PCNT1_IRQn);

  /* Route PCNT1 input to location 0 -> PCNT1_S0IN on PC4 */
  PCNT1->ROUTE = PCNT_ROUTE_LOCATION_LOC0;

  /* Select CC channel parameters */
  TIMER_InitCC_TypeDef
    timerCCInit =
  {
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionToggle,
    .cmoa       = timerOutputActionNone,
    .mode       = timerCCModeCompare,
    .filter     = true,
    .prsInput   = false,
    .coist      = false,
    .outInvert  = false,
  };

  /* Configure CC channel 0 */
  TIMER_InitCC(TIMER0, 0, &timerCCInit);

  /* Route CC0 to location 3 (PD1) and enable pin */
  TIMER0->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC3);

  /* Set Top Value */
  TIMER_TopSet(TIMER0, TOP);

  /* Select timer parameters */
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = true,
    .debugRun   = true,
    .prescale   = timerPrescale32,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeUp,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };

  /* Configure timer */
  TIMER_Init(TIMER0, &timerInit);


  /* Read Backup Real Time Counter value */
  burtcCountAtWakeup = BURTC_CounterGet();

  /* Configure Backup Domain */
  budSetup();

  /* Setting up a structure to initialize the calendar
     for January 1 2012 12:00:00
     The struct tm is declared in time.h
     More information for time.h library in http://en.wikipedia.org/wiki/Time.h */
  initialCalendar.tm_sec    = 0;    /* 0 seconds (0-60, 60 = leap second)*/
  initialCalendar.tm_min    = 22;//0;   /* 0 minutes (0-59) */
  initialCalendar.tm_hour   = 12;//12;   /* 12 hours (0-23) */
  initialCalendar.tm_mday   = 11;//1;   /* 1st day of the month (1 - 31) */
  initialCalendar.tm_mon    = 2;//0;   /* January (0 - 11, 0 = January) */
  initialCalendar.tm_year   = 114;//112;  /* Year 2012 (year since 1900) */
  initialCalendar.tm_wday   = 0;    /* Sunday (0 - 6, 0 = Sunday) */
  initialCalendar.tm_yday   = 0;  /* 1st day of the year (0-365) */
  initialCalendar.tm_isdst  = -1;    /* Daylight saving time; enabled (>0), disabled (=0) or unknown (<0) */

  /* Set the calendar */
  clockInit(&initialCalendar);

  /* Display reset cause */
  clockAppPrintResetCause(resetcause);

   /* Initialize display ++ */
  clockAppInit();

  /* Display reset cause */
  clockAppPrintResetCause(resetcause);

  /* If waking from backup mode, restore time from retention registers */
  if ( resetcause & RMU_RSTCAUSE_BUMODERST )
  {
    /* Check if retention registers were being written to when backup mode was entered */
    if ( (BURTC_Status() & BURTC_STATUS_RAMWERR) >> _BURTC_STATUS_RAMWERR_SHIFT )
    {
      clockAppPrintRamWErr();
    }

    /* Check if timestamp is written */
    if (! ((BURTC_Status() & BURTC_STATUS_BUMODETS) >> _BURTC_STATUS_BUMODETS_SHIFT) )
    {
      clockAppPrintNoTimestamp();
    }

    /* Restore time from backup RTC + retention memory and print backup info*/
    clockAppRestore( burtcCountAtWakeup );

    /* Reset timestamp */
    BURTC_StatusClear();
  }

  /* If normal startup, initialize BURTC */
  else
  {
    /* Start LFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

    /* Setup BURTC */
    burtcSetup();

    /* Backup initial calendar (also to initialize retention registers) */
    clockAppBackup();

    /* Update display if necessary */
   clockAppDisplay();
  }

  /* This is a work around for Chip Rev.D Errata, Revision 0.6. */
  /* Check for product revision 16 and 17 and set the offset */
  /* for ADC0_TEMP_0_READ_1V25. */
  prod_rev = (DEVINFO->PART & _DEVINFO_PART_PROD_REV_MASK) >> _DEVINFO_PART_PROD_REV_SHIFT;
  if( (prod_rev == 16) || (prod_rev == 17) )
  {
    temp_offset = 112;
  }
  else
  {
    temp_offset = 0;
  }
    /* Setup ADC for sampling internal temperature sensor. */


  /* Enable BURTC interrupts */
  NVIC_ClearPendingIRQ( BURTC_IRQn );
  NVIC_EnableIRQ( BURTC_IRQn );
  /* Enable board control interrupts */
  BSP_InterruptDisable(0xffff);
  BSP_InterruptFlagsClear(0xffff);

  /* ---------- Eternal while loop ---------- */
  while (1)

  {
    /* Sleep while waiting for interrupt */
    /* Use EM1 to drain VMCU capacitors quickly */ /* Go to EM1, while TIMER tuns compare output */
    EMU_EnterEM1();

    if(displayUpdate) {


    ExtBattSensor();         //Configure ADC to sample CH7: Battery sense
     ADC_Start(ADC0, adcStartSingle); //Get the sample from ADC
      /* Get ADC result */
      sample1 = ADC_DataSingleGet(ADC0);
      /* Calculate supply voltage, result is now 16 bit, divide by 65536 */
      voltage1 = (sample * 1250 ) / 4096;

      /* Sample external temperature sensor */
      //exttempsensor();
      /* Start one ADC sample */
      //ADC_Start(ADC0, adcStartSingle);

      /* Get ADC result */
      //extemp = ADC_DataSingleGet(ADC0);
      // showFahrenheit = 0;
      /* Setup ADC for sampling internal temperature sensor. */
      ExtTempSetup();

      /* Start one ADC sample */
      ADC_Start(ADC0, adcStartSingle);

      /* Read sensor value */
      /* According to rev. D errata ADC0_TEMP_0_READ_1V25 should be decreased */
      /* by the offset  but it is the same if ADC reading is increased - */
      /* reference manual 28.3.4.2. */
      //gTemperature = ADC_DataSingleGet(ADC0) + temp_offset;
      //showFahrenheit = 0;

      /* Get ADC result */
      sample = ADC_DataSingleGet(ADC0);
      /* Calculate supply voltage, result is now 16 bit, divide by 65536 */
      voltage = (sample * 1250 ) / 4096;
      }
    /* Update display if necessary */
    clockAppDisplay();
 }

}

/***************************************************************************//**
 * @brief Set up backup domain.
 ******************************************************************************/
void budSetup(void)
{
  /* Assign default TypeDefs */
  EMU_EM4Init_TypeDef em4Init = EMU_EM4INIT_DEFAULT;
  EMU_BUPDInit_TypeDef bupdInit = EMU_BUPDINIT_DEFAULT;

  /*Setup EM4 configuration structure */
  em4Init.lockConfig = true;
  em4Init.osc = emuEM4Osc_LFXO;
  em4Init.buRtcWakeup = true;
  em4Init.vreg = true;

  /* Setup Backup Power Domain configuration structure */
  bupdInit.probe = emuProbe_Disable;
  bupdInit.bodCal = false;
  bupdInit.statusPinEnable = true;
  bupdInit.resistor = emuRes_Res0;
  bupdInit.voutStrong = false;
  bupdInit.voutMed = false;
  bupdInit.voutWeak = false;
  bupdInit.inactivePower = emuPower_MainBU;
  bupdInit.activePower = emuPower_None;
  bupdInit.enable = true;

  /* Unlock configuration */
  EMU_EM4Lock( false );

  /* Initialize EM4 and Backup Power Domain with init structs */
  EMU_BUPDInit( &bupdInit );
  EMU_EM4Init( &em4Init );

  /* Release reset for backup domain */
  RMU_ResetControl( rmuResetBU, false );

  /* Lock configuration */
  EMU_EM4Lock( true );
}



/******************************************************************************
 * @brief   Configure backup RTC
 *****************************************************************************/
void burtcSetup(void)
{

  /* Create burtcInit struct and fill with default values */
  BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;

  /* Set burtcInit to proper values for this application */
  /* To make this example easier to read, all fields are listed,
     even those which are equal to their default value */
  burtcInit.enable = true;
  burtcInit.mode = burtcModeEM4;
  burtcInit.debugRun = false;
  burtcInit.clkSel = burtcClkSelLFXO;
  burtcInit.clkDiv = burtcClkDiv_128;
  burtcInit.timeStamp = true;
  burtcInit.compare0Top = false;
  burtcInit.lowPowerMode = burtcLPDisable;

  /* Initialize BURTC with burtcInit struct */
  BURTC_Init( &burtcInit );

  /* Enable BURTC interrupt on compare match and counter overflow */
  BURTC_IntEnable( BURTC_IF_COMP0 | BURTC_IF_OF );
}
