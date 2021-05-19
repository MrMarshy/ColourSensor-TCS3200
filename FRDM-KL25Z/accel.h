#ifndef ACCEL_H
#define ACCEL_H

#include <stdint.h>
#include "MKL25Z4.h"
// ----------------------------------------
// Function Prototypes
//
void i2c_init(I2C_MemMapPtr p);
void i2c_start(I2C_MemMapPtr p);
void i2c_stop(I2C_MemMapPtr p);
void i2c_set_tx(I2C_MemMapPtr p);
void i2c_set_rx(I2C_MemMapPtr p);
void i2c_set_slave(I2C_MemMapPtr p);
void i2c_set_master(I2C_MemMapPtr p);
void i2c_give_nack(I2C_MemMapPtr p);
void i2c_give_ack(I2C_MemMapPtr p);
void i2c_repeated_start(I2C_MemMapPtr p);
uint8_t i2c_read(I2C_MemMapPtr p);
void i2c_wait(I2C_MemMapPtr p);
int i2c_write(I2C_MemMapPtr p, uint8_t data);

void accel_init(void);
int16_t accel_x(void);
int16_t accel_y(void);
int16_t accel_z(void);

uint8_t mma8451_read(uint8_t addr);
void mma8451_write(uint8_t addr, uint8_t data);

#endif // ACCEL_H
