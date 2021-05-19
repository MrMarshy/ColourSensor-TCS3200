#pragma once

#include <stdint.h>

// Calibration constants
#define TCS320_RED_MIN  	3100u 		/*!< MIN Calibrated with White Colour */
#define TCS320_RED_MAX  	4805u 		/*!< MAX Calibrated with Black, Colour */
#define TCS320_GREEN_MIN    3100u
#define TCS320_GREEN_MAX    4975u
#define TCS320_BLUE_MIN  	3000u
#define TCS320_BLUE_MAX  	4875u

typedef enum{
    TCS320_SCALE_2_PCT,		/*!< Typical Output Freq 12kHz */
	TCS320_SCALE_20_PCT,	/*!< Typical Output Freq 120kHz */
    TCS320_SCALE_100_PCT, 	/*!< Typical Output Freq 600kHz */
    TCS320_POWER_DOWN
}TCS320_FreqScaler_t;

typedef struct {

	// Colour Pulse Width Measurements
	int32_t redPulseWidth;
	int32_t greenPulseWidth;
	int32_t bluePulseWidth;
    int32_t rawPulseWidth;

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