#ifndef __BME280_H__
#define __BME280_H__

#include <stdint.h>
#include "driver/i2c.h"

#define BME280_DEV_ADDR (0x76)

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

typedef struct bme280_measure_data {
    int32_t raw_tempreture;
    int32_t raw_pressure;
    int32_t raw_humidity;
    int32_t tempreture;
    uint32_t pressure;
    uint32_t humidity;
} bme280_measure_data_t;

/**
 * @brief Initialization of BME280
 *
 * @param bme280_config Pointer to the data structure that stores the configurations
 * @return
 *      - BME280_SUCCESS   Success
 *      - BME280_ERROR   Error
 */
int8_t bme280_init(bme280_config_t *bme280_config);

/**
 * @brief Perfome BME280 exit. At present, nothing is being done.
 * 
 * @return
 *      - BME280_SUCCESS   Success
 */
int8_t bme280_exit();

/**
 * @brief Conducting measurements and acquiring data
 * 
 * @param measure_data Pointer to the measurement data structure
 * @return
 *      - BME280_SUCCESS   Success
 *      - BME280_ERROR   Error
 */
int8_t bme280_measure(bme280_measure_data_t *measure_data);

/**
 * @brief Reading register data
 * 
 * @param reg_addr Register address
 * @param data pointer to the data array which stores register data
 * @param size Number of bytes to read
 * @return
 *      - BME280_SUCCESS   Success
 *      - BME280_ERROR   Error
 */
int8_t bme280_read_reg(uint8_t reg_addr, uint8_t *data, uint8_t size);

/**
 * @brief Writing data to registers
 *
 * @param reg_addr Register address
 * @param data Pointer to the data to write
 * @return
 *      - BME280_SUCCESS   Success
 *      - BME280_ERROR   Error
 */
int8_t bme280_write_reg(uint8_t reg_addr, uint8_t data);

#endif /* __BME280_H__ */