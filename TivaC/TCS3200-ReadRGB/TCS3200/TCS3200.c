#include <stddef.h>
#include <assert.h>
#include "TM4C123GH6PM.h"
#include "Utils.h"
#include "TCS3200.h"

static void Timer0Capture_init(TCS3200_t* const sensor);
static int Timer0A_periodCapture(void);
static void Timer0Capture_Start(void);
static void Timer0Capture_Stop(void);

void TCS3200_Init(TCS3200_t* const sensor, uint8_t doCalib, const PinConfig_t *pin_config, size_t pin_config_len, TCS3200_FreqScaler_t scale_factor)
{

    assert((doCalib >= 0) && (doCalib <= 1));

    // Pins array must be of length 5
    assert(pin_config_len == 5);

    sensor->doCalibration = doCalib;
    sensor->pins = pin_config;

    /* Initialise the pins */
    for(int i = 0; i < 5; ++i){

        SYSCTL->RCGCGPIO  |= (sensor->pins[i]).port; /* enable Run mode for GPIO Port */
        while( !(SYSCTL->RCGCGPIO & (sensor->pins[i]).port)){}

        SYSCTL->GPIOHBCTL |= sensor->pins[i].port; /* enable AHB for GPIO Port */
        while( !(SYSCTL->GPIOHBCTL & sensor->pins[i].port)){}
    }
    
    for(int i = 0; i < 4; ++i){
        /* GPIOx pins as outputs mapped to sensor pins s0-s3 */
        sensor->pins[i].port_base->DIR |= (sensor->pins[i].pin);
        sensor->pins[i].port_base->DEN |= (sensor->pins[i].pin);
    }

    sensor->scale_factor = scale_factor;

    Timer0Capture_init(sensor);

}

int TCS3200_GetRedPulseWidth(TCS3200_t* const sensor){

    // Set Sensor to read Red only
    sensor->pins[S2].port_base->DATA_Bits[sensor->pins[S2].pin] = 0;
    sensor->pins[S3].port_base->DATA_Bits[sensor->pins[S3].pin] = 0;

    Timer0Capture_Start();
    
    int period = Timer0A_periodCapture();
    
    Timer0Capture_Stop();         /* disable timer0A */

    sensor->redValue = (uint8_t)map(period, TCS3200_RED_MIN, TCS3200_RED_MAX, 255, 0);

    return period;
}

int TCS3200_GetGreenPulseWidth(TCS3200_t* const sensor){

    // Set Sensor to read green only
    sensor->pins[S2].port_base->DATA_Bits[sensor->pins[S2].pin] = sensor->pins[S2].pin;
    sensor->pins[S3].port_base->DATA_Bits[sensor->pins[S3].pin] = sensor->pins[S3].pin;

    Timer0Capture_Start(); /* enable timer0A */
    
    int period = Timer0A_periodCapture();
    
    Timer0Capture_Stop();          /* disable timer0A */

    sensor->greenValue = (uint8_t)map(period, TCS3200_GREEN_MIN, TCS3200_GREEN_MAX, 255, 0);

    return period;
}

int TCS3200_GetBluePulseWidth(TCS3200_t* const sensor){
    
    // Set Sensor to read blue only
    sensor->pins[S2].port_base->DATA_Bits[sensor->pins[S2].pin] = 0;
    sensor->pins[S3].port_base->DATA_Bits[sensor->pins[S3].pin] = sensor->pins[S3].pin;

    Timer0Capture_Start(); /* enable timer0A */
    
    int period = Timer0A_periodCapture();
    
    Timer0Capture_Stop();          /* disable timer0A */

    sensor->blueValue = (uint8_t)map(period, TCS3200_BLUE_MIN, TCS3200_BLUE_MAX, 255, 0);

    return period;
}


void TCS3200_SetFreqScaling(TCS3200_t* const sensor, TCS3200_FreqScaler_t scale_factor){
    switch(scale_factor){

        case TCS3200_SCALE_2_PCT:
            sensor->pins[S0].port_base->DATA_Bits[sensor->pins[S0].pin] = 0;
            sensor->pins[S1].port_base->DATA_Bits[sensor->pins[S1].pin] = sensor->pins[S1].pin;
            break;

        case TCS3200_SCALE_20_PCT:
            sensor->pins[S0].port_base->DATA_Bits[sensor->pins[S0].pin] = sensor->pins[S0].pin;
            sensor->pins[S1].port_base->DATA_Bits[sensor->pins[S1].pin] = 0;
            break;

        case TCS3200_SCALE_100_PCT:
            sensor->pins[S0].port_base->DATA_Bits[sensor->pins[S0].pin] = sensor->pins[S0].pin;
            sensor->pins[S1].port_base->DATA_Bits[sensor->pins[S1].pin] = sensor->pins[S1].pin;
            break;

        case TCS3200_POWER_DOWN:
            sensor->pins[S0].port_base->DATA_Bits[sensor->pins[S0].pin] = 0;
            sensor->pins[S1].port_base->DATA_Bits[sensor->pins[S1].pin] = 0;
            break;

        default:break;
    }
}

static void Timer0Capture_init(TCS3200_t* const sensor)
{
    SYSCTL->RCGCTIMER |= 1;     /* enable clock to Timer Block 0 */
    SYSCTL->RCGCGPIO  |= (sensor->pins[FREQ_OUT]).port; /* enable Run mode for GPIO Port */
    while( !(SYSCTL->RCGCGPIO & (sensor->pins[FREQ_OUT]).port)){}

    sensor->pins[FREQ_OUT].port_base->DIR &= ~sensor->pins[FREQ_OUT].pin;        /* make freq out pin an input pin */
    sensor->pins[FREQ_OUT].port_base->DEN |= sensor->pins[FREQ_OUT].pin;         /* make freq out pin as digital pin */
    sensor->pins[FREQ_OUT].port_base->AFSEL |= sensor->pins[FREQ_OUT].pin;       /* use freq out pin alternate function */
    sensor->pins[FREQ_OUT].port_base->PCTL &= ~0x0F000000;  /* configure freq out pin for T0CCP0 */
    sensor->pins[FREQ_OUT].port_base->PCTL |= 0x07000000;
    
    TIMER0->CTL &= ~1;          /* disable timer0A during setup */
    TIMER0->CFG = 4;            /* 16-bit timer mode */
    TIMER0->TAMR = 0x17;        /* up-count, edge-time, capture mode */
    TIMER0->CTL &= ~0x0C;        /* capture the rising edge */
}

static void Timer0Capture_Start(void){

    TIMER0->CTL |= 1;
}

static void Timer0Capture_Stop(void){

    TIMER0->CTL &= ~1;
}

/* This function captures two consecutive rising edges of a periodic signal from Timer Block 0 Timer A and returns the time difference (the period of the signal). */
static int Timer0A_periodCapture(void)
{
    int lastEdge, thisEdge;
    
    /* capture the first rising edge */
    TIMER0->ICR = 4;            /* clear timer0A capture flag */
    while((TIMER0->RIS & 4) == 0) ;    /* wait till captured */
    lastEdge = TIMER0->TAR;     /* save the timestamp */

    /* capture the second rising edge */
    TIMER0->ICR = 4;            /* clear timer0A capture flag */
    while((TIMER0->RIS & 4) == 0) ;    /* wait till captured */
    thisEdge = TIMER0->TAR;     /* save the timestamp */
    
    return (thisEdge - lastEdge) & 0x00FFFFFF; /* return the time difference */
}