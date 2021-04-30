#ifndef __BME280_H__
#define __BME280_H__

#include <stdint.h>
#include "driver/i2c.h"

enum bme280_status {
    BME280_ERROR = -1,
    BME280_SUCCESS,
};

enum bme280_oversamp_rate {
    BME280_OVERSAMP_RATE_X0 = 0,
    BME280_OVERSAMP_RATE_X1,
    BME280_OVERSAMP_RATE_X2,
    BME280_OVERSAMP_RATE_X4,
    BME280_OVERSAMP_RATE_X8,
    BME280_OVERSAMP_RATE_X16,
};

typedef struct bme280_config {
    uint8_t dev_addr;
    uint8_t oversamp_rate_tem;
    uint8_t oversamp_rate_pre;
    uint8_t oversamp_rate_hum;
} bme280_config_t;

typedef struct bme280_measure_date {
    int32_t raw_tempreture;
    int32_t raw_pressure;
    int32_t raw_humidity;
    int32_t tempreture;
    uint32_t pressure;
    uint32_t humidity;
} bme280_measure_date_t;

int8_t bme280_init(bme280_config_t *bme280_config);
int8_t bme280_exit();
int8_t bme280_measure(bme280_measure_date_t *measure_data);
int8_t bme280_read_reg(uint8_t reg_addr, uint8_t *data, uint8_t size);
int8_t bme280_write_reg(uint8_t reg_addr, uint8_t data);

#endif /* __BME280_H__ */