#ifndef TCS320_H
#define TCS320_H

#include <stdint.h>

// Calibration constants
#define TCS320_RED_MIN  	75u
#define TCS320_RED_MAX  	1300u
#define TCS320_GREEN_MIN  80u
#define TCS320_GREEN_MAX  1500u
#define TCS320_BLUE_MIN  	65u
#define TCS320_BLUE_MAX  	1110u

typedef enum{
	TCS320_SCALE_20_PCT,
}TCS320_FreqScaler_t;

typedef struct {

	// Colour Pulse Width Measurements
	int32_t redPulseWidth;
	int32_t greenPulseWidth;
	int32_t bluePulseWidth;

	// Variables for final Colour RGB Values
	uint8_t redValue;
	uint8_t greenValue;
	uint8_t blueValue;

	uint8_t doCalibration;

	// Function Pointers
	int (*getRedPulseWidth)(void);
	int (*getGreenPulseWidth)(void);
	int (*getBluePulseWidth)(void);
	int (*setFrequencyScaling)(TCS320_FreqScaler_t scaleFactor);
}TCS320_t;

#endif // TCS320_H
