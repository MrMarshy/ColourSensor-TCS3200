#include <MKL25Z4.h>
#include <stdint.h>
#include <string.h>
#include "common.h"
#include "uart.h"
#include "TCS320.h"
#include "pulse_measure.h"



// TCS320 Colour Sensor initialization
static TCS320_t colourSensor = {0};

// Prototypes of the TCS320_t function pointers
static int getRedPulseWidth(void);
static int getGreenPulseWidth(void);
static int getBluePulseWidth(void);
static int setFrequencyScaling(TCS320_FreqScaler_t);

static void RGB_LED(uint32_t r, uint32_t g, uint32_t b){
	TPM2_C0V = r;
	TPM2_C1V = g;
	TPM0_C1V = b;
}

static void init_led_io(void){
// Turn on clock gating to PortB module (red and green LEDs) and 
    // PortD module (blue LED)  
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;

    SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM2_MASK;
    SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1);

    PORTB_PCR18 = PORT_PCR_MUX(3);  // TPM2_CH0 enable on PTB18 (red)
    PORTB_PCR19 = PORT_PCR_MUX(3);  // TPM2_CH1 enable on PTB19 (green)
    PORTD_PCR1  = PORT_PCR_MUX(4);  // TPM0_CH1 enable on PTD1  (blue)

    RGB_LED(0,0,0);                 // Off
    
    TPM0_MOD  = 99;
    TPM0_C1SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
    TPM2_MOD  = 99;
    TPM2_C0SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
    TPM2_C1SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;

    TPM2_SC   = TPM_SC_CMOD(1) | TPM_SC_PS(0);     /* Edge Aligned PWM running from BUSCLK / 1 */
    TPM0_SC   = TPM_SC_CMOD(1) | TPM_SC_PS(0);     /* Edge Aligned PWM running from BUSCLK / 1 */
}

static int InitGPIO(void){
	/* S0 Pin is PTC9 */
	SIM->SCGC5 |= 0x800;      /* Enable Clock to PORT C */
	PORTC->PCR[9] = 0x100;    /* Make PTC9 pin as GPIO */
	PTC->PDDR |= 0x200;       /* Make PTC9 pin as an output pin */
	
	/* S1 Pin is PTC8 */
	PORTC->PCR[8] = 0x100;    /* Make PTC8 pin as GPIO */
	PTC->PDDR |= 0x100;       /* Make PTC8 pin as an output pin */
	
	/* S2 Pin is PTA5 */
	SIM->SCGC5 |= 0x400;      /* Enable Clock to PORTA */
	PORTA->PCR[5] = 0x100;    /* Make PTA5 pin as GPIO */
	PTA->PDDR |= 0x20;        /* Make PTA5 pin as an output pin */
	
	/* S3 Pin is PTA4 */
	PORTA->PCR[4] = 0x100;    /* Make PTA4 pin as GPIO */
	PTA->PDDR |= 0x10;        /* Make PTA4 pin as an output pin */
	
	return 0;
}

int main(void){
	
	SystemCoreClockUpdate();
	
	uart_init(115200);
	
	accel_init();
	
	init_led_io();
	
	/* Initialize Colour Sensor S0, S1, S2 and S3 pins as outputs */
	InitGPIO();
	
	RGB_LED(0, 255, 0);

  // Assign function pointers to read RGB Pulse widths and set frequency scaling
  colourSensor.getRedPulseWidth = getRedPulseWidth;
  colourSensor.getGreenPulseWidth = getGreenPulseWidth;
  colourSensor.getBluePulseWidth = getBluePulseWidth;
	colourSensor.setFrequencyScaling = setFrequencyScaling;
	
	
	printf("Colour Sensor Demo Program\r\n");
	printf("Built: %s %s\r\n\r\n", __DATE__, __TIME__);
	
	// Set Colour Sensors frequency scaling to 20%
	colourSensor.setFrequencyScaling(TCS320_SCALE_20_PCT);
	
	colourSensor.doCalibration = 0;
	
	uint32_t delayTime = 0;
	if(colourSensor.doCalibration){
		delayTime = 200;
	}
	else{
		delayTime = 2000;
	}

	for(;;){
		
		int32_t r = colourSensor.getRedPulseWidth();
		delay(200);
		
		int32_t g = colourSensor.getGreenPulseWidth();
		delay(200);
		
		int32_t b = colourSensor.getBluePulseWidth();
		delay(200);
		
		RGB_LED(r, g, b);
		
		printf("%d,%d,%d\r\n", r, g, b);
		delay(delayTime);
		
		
	}
}

static int getRedPulseWidth(void){
	
	// Set Sensor to read Red Only
	PTA->PDOR &= ~0x20u;        /* Make PTA5 pin (S2 Pin) low */
	PTA->PDOR &= ~0x10u;        /* Make PTA4 pin (S3 Pin) low */
	
	const int averagePoints = 25;
	int avg = 0;

	for(int i = 0; i < averagePoints; ++i){

		// Get Pulse Width Measurement
		
		colourSensor.redPulseWidth = PulseIn();

		colourSensor.redValue = (uint8_t)map(colourSensor.redPulseWidth, TCS320_RED_MIN,
				TCS320_RED_MAX, 255, 0);

		if(colourSensor.doCalibration){
			printf("rw: %d, rv: %d\r\n", colourSensor.redPulseWidth, colourSensor.redValue);
		}
		avg += colourSensor.redValue;

	}
	return (int)(avg/averagePoints);
}

static int getGreenPulseWidth(void){
	// Set Sensor to read Green Only
	PTA->PDOR |= 0x20u;        /* Make PTA5 pin (S2 Pin) high */
	PTA->PDOR |= 0x10u;        /* Make PTA4 pin (S3 Pin) high */
	
	const int averagePoints = 25;
	int avg = 0;

	for(int i = 0; i < averagePoints; ++i){

		// Get Pulse Width Measurement
		
		colourSensor.greenPulseWidth = PulseIn();

		colourSensor.greenValue = (uint8_t)map(colourSensor.greenPulseWidth, TCS320_GREEN_MIN,
				TCS320_GREEN_MAX, 255, 0);

		if(colourSensor.doCalibration){
			printf("gw: %d, gv: %d\r\n", colourSensor.greenPulseWidth, colourSensor.greenValue);
		}
		avg += colourSensor.greenValue;

	}
	return (int)(avg/averagePoints);
}

static int getBluePulseWidth(void){

	// Set Sensor to read Green Only
	PTA->PDOR &= ~0x20u;        /* Make PTA5 pin (S2 Pin) low */
	PTA->PDOR |= 0x10u;        /* Make PTA4 pin (S3 Pin) high */
	
	const int averagePoints = 25;
	int avg = 0;

	for(int i = 0; i < averagePoints; ++i){

		// Get Pulse Width Measurement
		colourSensor.bluePulseWidth = PulseIn();

		colourSensor.blueValue = (uint8_t)map(colourSensor.bluePulseWidth, TCS320_BLUE_MIN,
				TCS320_BLUE_MAX, 255, 0);

		if(colourSensor.doCalibration){
			printf("bw: %d, bv: %d\r\n", colourSensor.bluePulseWidth, colourSensor.blueValue);
		}
		avg += colourSensor.blueValue;

	}
	return (int)(avg/averagePoints);
}

static int setFrequencyScaling(TCS320_FreqScaler_t scaleFactor){
	
	switch(scaleFactor){
		case TCS320_SCALE_20_PCT:
			// Set S0 Pin, Reset S1 Pin
			PTC->PDOR |= 0x200u; /* Set S0 pin */
			PTC->PDOR &= ~0x100u;  /* Reset S1 pin */
			break;
		
		default:break;
				
	}
	
	return 0;
}

long map(long x, long in_min, long in_max, long out_min, long out_max){

	in_max += 1;
	out_max += 1;

	long ret = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

	if((ret < out_min) && (out_min > out_max)){
		return ret;
	}
	else if((ret < out_min) && (out_min < out_max)){
		return out_min;
	}
	else if((ret > out_max) && (out_min > out_max)){
		return ret;
	}
	else if((ret > out_max) && (out_min < out_max)){
		return out_max;
	}
	else{
		return ret;
	}

}

