#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include <stddef.h>
#include "msp.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CS_PORT    P2
#define CS_PIN     (BIT3)

typedef enum{
    /**< SPI_MODE0 & SPI_MODE1 use non-inverted clock polarity (clk is low when ss transitions to low) */
    SPI_MODE0, /**< Data sampled on rising edge of clk and shifted out on falling edge */
    SPI_MODE1, /**< Data sampled on the falling edge of clk and shifted out on the rising edge */
    
    /**< SPI_MODE2 & SPI_MODE3 use inverted clock polarity (clk is high when ss transitions to low) */
    SPI_MODE2, /**< Data sampled on the falling edge of clk and shifted out on the rising edge */
    SPI_MODE3  /**< Data sampled on rising edge of clk and shifted out on falling edge */
}SPI_Mode;

typedef enum{
    SPI_MSBFIRST,
    SPI_LSBFIRST
}SPI_BitOrder;

typedef struct{
    SPI_BitOrder bit_order;
    SPI_Mode mode;

}SPI_settings;

void SPI_chipSelect(void);

void SPI_chipDeselect(void);

void SPI1_init(SPI_settings const * const settings);

void SPI1_deInit(void);

void SPI1_enable(void);

void SPI1_disable(void);

uint8_t SPI_transfer(uint8_t data);

uint16_t SPI_transfer16(uint16_t data);

void SPI_beginTransaction(SPI_settings const* const settings);

void SPI_endTransaction(void);

void SPI_setBitOrder(SPI_BitOrder bit_order);

void SPI_setDataMode(SPI_Mode mode);

void SPI_configure(SPI_settings const * const settings);

#ifdef __cplusplus
}
#endif

#endif /* SPI_H */
