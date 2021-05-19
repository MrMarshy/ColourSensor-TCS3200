#ifndef TCS3200_H
#define TCS3200_H

#include <stdint.h>
#include "bsp.h"

/**
 * | S0 | S1 | Output Freq Scaling
 * ---------------------------------
 * |  L |  L | Power down
 * |  L |  H | 2%
 * |  H |  L | 20%
 * |  H |  H | 100%
*/

/**
 * | S2 | S3 | PHOTODIODE TYPE
 * --------------------------------
 * |  L |  L | RED
 * |  L |  H | BLUE
 * |  H |  L | Clear (no filter)
 * |  H |  H | GREEN
 */


// Calibration constants
#define TCS3200_RED_MIN  	1400u 		/*!< MIN Calibrated with White Colour */
#define TCS3200_RED_MAX  	18005u 		/*!< MAX Calibrated with Black, Colour */
#define TCS3200_GREEN_MIN   1600u
#define TCS3200_GREEN_MAX   20975u
#define TCS3200_BLUE_MIN  	1200u
#define TCS3200_BLUE_MAX  	16485u

#define S0              0U
#define S1              1U
#define S2              2U
#define S3              3U
#define FREQ_OUT        4U

typedef enum{
    TCS3200_SCALE_2_PCT,		/*!< Typical Output Freq 12kHz */
	TCS3200_SCALE_20_PCT,	/*!< Typical Output Freq 120kHz */
    TCS3200_SCALE_100_PCT, 	/*!< Typical Output Freq 600kHz */
    TCS3200_POWER_DOWN
}TCS3200_FreqScaler_t;

typedef struct {
	// Colour Pulse Width Measurements
	int32_t redPulseWidth;
	int32_t greenPulseWidth;
	int32_t bluePulseWidth;

	// Variables for final Colour RGB Values
	uint8_t redValue;
	uint8_t greenValue;
	uint8_t blueValue;

    // Sensor Pins
    const PinConfig_t *pins;

    // Scal Factor
    TCS3200_FreqScaler_t scale_factor;

	uint8_t doCalibration;

}TCS3200_t;

void TCS3200_Init(TCS3200_t* const sensor, uint8_t doCalib, const PinConfig_t *pin_config,
                         size_t pin_config_len, TCS3200_FreqScaler_t scale_factor);
int TCS3200_GetRedPulseWidth(TCS3200_t* const sensor);
int TCS3200_GetGreenPulseWidth(TCS3200_t* const sensor);
int TCS3200_GetBluePulseWidth(TCS3200_t* const sensor);
void TCS3200_SetFreqScaling(TCS3200_t* const sensor, TCS3200_FreqScaler_t scale_factor);

#endif // TCS3200_H
