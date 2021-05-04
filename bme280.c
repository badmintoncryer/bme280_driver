#include "bme280.h"

#define ENABLE_ACK_CHECK (true)

#define BME280_CAST_TO_U16(array, upper_pos, lower_pos) (uint16_t)((((uint16_t)((uint8_t)array[upper_pos])) << 8) | array[lower_pos])
#define BME280_CAST_TO_S16(array, upper_pos, lower_pos) (int16_t)((((int16_t)((int8_t)array[upper_pos])) << 8) | array[lower_pos])

typedef struct param_table {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;
} param_table_t;

static int8_t bme280_get_adjust_param(param_table_t *param_table);
static int8_t bme280_compensate_data(bme280_measure_data_t *measure_data);
static int32_t bme280_compensate_tem(int32_t tem);
static uint32_t bme280_compensate_pre(int32_t pre);
static uint32_t bme280_compensate_hum(int32_t hum);

static int32_t t_fine;
static param_table_t param;
static bme280_config_t *config_table;

int8_t bme280_init(bme280_config_t *bme280_config)
{
    if (bme280_config == NULL) {
        return BME280_ERROR;
    }

    config_table = bme280_config;
    return bme280_get_adjust_param(&param);
}

int8_t bme280_exit()
{
    /* Do nothing */
    return BME280_SUCCESS;
}

int8_t bme280_measure(bme280_measure_data_t *measure_data)
{
    if (measure_data == NULL) {
        return BME280_ERROR;
    }

    int8_t status;
    uint8_t write_data;
    uint8_t is_measuring;
    uint8_t temp_measure_data[3];

    write_data = config_table->oversamp_rate_hum;
    status = bme280_write_reg(0xF2, write_data);

    write_data = config_table->oversamp_rate_tem << 5
               | config_table->oversamp_rate_pre << 2
               | 0x01;
    if (status == BME280_SUCCESS) {
        status = bme280_write_reg(0xF4, write_data);
    } else {
        goto ERROR;
    }

    do {
        status = bme280_read_reg(0xF3, &is_measuring, 1);
        is_measuring = is_measuring >> 3;

        if (status != BME280_SUCCESS) {
            goto ERROR;
        }
    } while (is_measuring != 0);

    status = bme280_read_reg(0xF7, temp_measure_data, 3);
    measure_data->raw_pressure = (int32_t)((int32_t)temp_measure_data[0] << 12
                                         | (int32_t)temp_measure_data[1] << 4
                                         | temp_measure_data[2] >> 4);

    if (status == BME280_SUCCESS) {
        status = bme280_read_reg(0xFA, temp_measure_data, 3);
        measure_data->raw_tempreture = (int32_t)((int32_t)temp_measure_data[0] << 12
                                               | (int32_t)temp_measure_data[1] << 4
                                               | temp_measure_data[2] >> 4);
    } else {
        goto ERROR;
    }

    if (status == BME280_SUCCESS) {
        status = bme280_read_reg(0xFD, temp_measure_data, 2);
        measure_data->raw_humidity = (int32_t)((int32_t)temp_measure_data[0] << 8
                                             | temp_measure_data[1]);
    } else {
        goto ERROR;
    }

    if (status == BME280_SUCCESS) {
        status = bme280_compensate_data(measure_data);
    } else {
        goto ERROR;
    }

    return status;

ERROR:
    return BME280_ERROR;
}

/* 指定したレジスタに1byteデータを書き込む。
   BME280への複数バイト書き込みは自動インクリメントが行われないため、1byteのみ書き込む仕様とした */
int8_t bme280_write_reg(uint8_t reg_addr, uint8_t data)
{
    int8_t status = BME280_SUCCESS;

    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    status = i2c_master_start(cmd_handle);

    /* I2Cキューにコマンドを全て送り、i2c_master_cmd_begin()でトランザクションを実行する */
    /* 書き込みを行うデバイスアドレスの送信 */
    if (status == BME280_SUCCESS) {
        status = i2c_master_write_byte(cmd_handle,
                                       (BME280_DEV_ADDR << 1) | I2C_MASTER_WRITE,
                                       ENABLE_ACK_CHECK);
    } else {
        goto ERROR;
    }

    /* 書き込みを行うレジスタアドレスの送信 */
    if (status == BME280_SUCCESS) {
        status = i2c_master_write_byte(cmd_handle, reg_addr, ENABLE_ACK_CHECK);
    } else {
        goto ERROR;
    }

    /* 書き込むデータの送信 */
    if (status == BME280_SUCCESS) {
        status = i2c_master_write_byte(cmd_handle, data, ENABLE_ACK_CHECK);
    } else {
        goto ERROR;
    }

    /* キューへのコマンド送信の終了 */
    if (status == BME280_SUCCESS) {
        status = i2c_master_stop(cmd_handle);
    } else {
        goto ERROR;
    }

    /* キューにあるコマンドのトランザクションの実施 */
    if (status == BME280_SUCCESS) {
        status = i2c_master_cmd_begin(I2C_NUM_0,
                                      cmd_handle,
                                      10 / portTICK_PERIOD_MS);
    } else {
        goto ERROR;
    }

    if (status == BME280_SUCCESS) {
        i2c_cmd_link_delete(cmd_handle);
    } else {
        goto ERROR;
    }

    return status;

ERROR:
    return BME280_ERROR;
}

int8_t bme280_read_reg(uint8_t reg_addr, uint8_t *data, uint8_t size)
{
    if (data == NULL) {
        return BME280_ERROR;
    }

    int32_t status;

    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    status = i2c_master_start(cmd_handle);
    if (status == BME280_SUCCESS) {
        status = i2c_master_write_byte(cmd_handle,
                                       (BME280_DEV_ADDR << 1) | I2C_MASTER_WRITE,
                                       ENABLE_ACK_CHECK);
    } else {
        goto ERROR;
    }

    if (status == BME280_SUCCESS) {
        status = i2c_master_write_byte(cmd_handle, reg_addr, ENABLE_ACK_CHECK);
    } else {
        goto ERROR;
    }

    if (status == BME280_SUCCESS) {
        status = i2c_master_start(cmd_handle);
    } else {
        goto ERROR;
    }

    if (status == BME280_SUCCESS) {
        status = i2c_master_write_byte(cmd_handle,
                                       (BME280_DEV_ADDR << 1) | I2C_MASTER_READ,
                                       ENABLE_ACK_CHECK);
    } else {
        goto ERROR;
    }

    if (status == BME280_SUCCESS) {
        if (size > 1) {
            status = i2c_master_read(cmd_handle, data, size-1, I2C_MASTER_ACK);
        }
    } else {
        goto ERROR;
    }

    if (status == BME280_SUCCESS) {
        status = i2c_master_read_byte(cmd_handle, data+size-1, I2C_MASTER_NACK);
    } else {
        goto ERROR;
    }

    if (status == BME280_SUCCESS) {
        status = i2c_master_stop(cmd_handle);
    } else {
        goto ERROR;
    }

    if (status == BME280_SUCCESS) {
        status = i2c_master_cmd_begin(I2C_NUM_0,
                                      cmd_handle,
                                      10 / portTICK_PERIOD_MS);
    } else {
        goto ERROR;
    }

    if (status == BME280_SUCCESS) {
        i2c_cmd_link_delete(cmd_handle);
    } else {
        goto ERROR;
    }

    return status;

ERROR:
    return BME280_ERROR;
}

static int8_t bme280_get_adjust_param(param_table_t *param_table)
{
    if (param_table == NULL) {
        return BME280_ERROR;
    }

    int8_t status;
    uint8_t read_data[32];

    status = bme280_read_reg(0x88, read_data, 1);
    if (status == BME280_SUCCESS) {
        status = bme280_read_reg(0xA1, &read_data[24], 1);
    } else {
        return status;
    }
    if (status == BME280_SUCCESS) {
        status = bme280_read_reg(0xE1, &read_data[25], 7);
    } else {
        return status;
    }

    if (status == BME280_SUCCESS) {
        param_table->dig_t1 = BME280_CAST_TO_U16(read_data, 1, 0);
        param_table->dig_t2 = BME280_CAST_TO_S16(read_data, 3, 2);
        param_table->dig_t3 = BME280_CAST_TO_S16(read_data, 5, 4);
        param_table->dig_p1 = BME280_CAST_TO_U16(read_data, 7, 6);
        param_table->dig_p2 = BME280_CAST_TO_S16(read_data, 9, 8);
        param_table->dig_p3 = BME280_CAST_TO_S16(read_data, 11, 10);
        param_table->dig_p4 = BME280_CAST_TO_S16(read_data, 13, 12);
        param_table->dig_p5 = BME280_CAST_TO_S16(read_data, 15, 14);
        param_table->dig_p6 = BME280_CAST_TO_S16(read_data, 17, 16);
        param_table->dig_p7 = BME280_CAST_TO_S16(read_data, 19, 18);
        param_table->dig_p8 = BME280_CAST_TO_S16(read_data, 21, 20);
        param_table->dig_p9 = BME280_CAST_TO_S16(read_data, 23, 22);
        param_table->dig_h1 = read_data[24];
        param_table->dig_h2 = BME280_CAST_TO_S16(read_data, 26, 25);
        param_table->dig_h3 = read_data[27];
        param_table->dig_h4 = (int16_t)((int16_t)read_data[28] << 4 | (read_data[29] & 0x0F));
        param_table->dig_h5 = (int16_t)((int16_t)read_data[30] << 4 | (read_data[29] & 0xF0));
        param_table->dig_h6 = (int8_t)read_data[31];
    }
    return status;
}

static int8_t bme280_compensate_data(bme280_measure_data_t *measure_data)
{
    if (measure_data == NULL) {
        return BME280_ERROR;
    }

    measure_data->tempreture = bme280_compensate_tem(measure_data->raw_tempreture);
    measure_data->pressure = bme280_compensate_pre(measure_data->raw_pressure);
    measure_data->humidity = bme280_compensate_hum(measure_data->raw_humidity);

    return BME280_SUCCESS;
}

static int32_t bme280_compensate_tem(int32_t tem)
{
    int32_t var1, var2, t;
    var1 = ((((tem >> 3) - ((int32_t)param.dig_t1 << 1))) * ((int32_t)param.dig_t2)) >> 11;
    var2 = (((((tem >> 4) - ((int32_t)param.dig_t1)) * ((tem >> 4) - ((int32_t)param.dig_t1))) >> 12)
            * ((int32_t)param.dig_t3)) >> 14;
    t_fine = var1 + var2;
    t = (t_fine * 5 + 128) >> 8;

    return t;
}

static uint32_t bme280_compensate_pre(int32_t pre)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)param.dig_p6;
    var2 = var2 + ((var1 * (int64_t)param.dig_p5) << 17);
    var2 = var2 + (((int64_t)param.dig_p4) << 35);
    var1 = ((var1 * var1 * (int64_t)param.dig_p3) >> 8) + ((var1 * (int64_t)param.dig_p2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)param.dig_p1) >> 33;

    if (var1 == 0) {
        return 0;
    }

    p = 1048576 - pre;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)param.dig_p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)param.dig_p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)param.dig_p7) << 4);

    return (uint32_t)p;
}

static uint32_t bme280_compensate_hum(int32_t hum)
{
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (
        ((((hum << 14) - (((int32_t)param.dig_h4) << 20) - (((int32_t)param.dig_h5) * v_x1_u32r)) + ((int32_t)16384)) >> 15)
        * (((((((v_x1_u32r * ((int32_t)param.dig_h6)) >> 10) * (((v_x1_u32r * ((int32_t)param.dig_h3)) >> 11) + ((int32_t)32768))) >> 10)
        + ((int32_t)2097152)) * ((int32_t)param.dig_h2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)param.dig_h1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r >> 12);
}