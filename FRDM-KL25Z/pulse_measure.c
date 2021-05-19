#include <MKL24Z4.h>
#include <stdint.h>
#include "pulse_measure.h"

// Input-Capture variables
static uint32_t captures[2] = {0};
static volatile uint8_t captureDone = 0;
static uint8_t isRisingEdge = 1;
static uint32_t pulseWidth = 0;

static int CaptureInit(void){
	SIM->SCGC5 |= 0x1000;     					/* Enable clock to Port D */
	PORTD->PCR[2] = 0x0400;   					/* Set PTD2 pin for TPM0CH2 */
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; 	/* Enable clock to TPM0 */
	SIM->SOPT2 |= 0x01000000; 					/* Use MCGFLLCLK as timer counter clock */
	
	TPM0->SC = 0;                  			/* Disable Timer while configuring */
	TPM0->SC = 0x05;               			/* Prescaler 32 */
	TPM0->MOD = 0xFFFF;            			/* Max modulo value */
	TPM0->CONTROLS[2].CnSC = 0x0C; 			/* Rising and Falling Edge Capture */
	TPM0->SC |= 0x08;              			/* Enable Timer */
	
	return 0;
}

int32_t PulseIn(){
	
	CaptureInit(); /* Start Capturing Rising and Falling Edges */
	
	while(!captureDone){
		
		while(!(TPM0->CONTROLS[2].CnSC & TPM_CnSC_CHF_MASK)){} /* Wait for channel event to be set */
		TPM0->CONTROLS[2].CnSC |= TPM_CnSC_CHF_MASK;           /* Clear CHF */
		
		if(isRisingEdge){
			
			captures[0] = TPM0->CONTROLS[2].CnV;
			isRisingEdge = 0;
		}
		else{ /* Falling Edge */
			captures[1] = TPM0->CONTROLS[2].CnV;
			isRisingEdge = 1;
			
			TPM0->CNT = 0;     /* Reset Counter */
			
			if(captures[1] >= captures[0]){
				pulseWidth = captures[1] - captures[0];
			}
			else{
				pulseWidth = (TPM0->MOD - captures[0]) + captures[1];
			}
			captureDone = 1;
		
		}
	}
	
	TPM0->SC = 0;                  			/* Disable Timer */
	captureDone = 0;
	return (int32_t)pulseWidth;
	
}
