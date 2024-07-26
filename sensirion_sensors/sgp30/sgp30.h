/*
 * Copyright (c) 2024 Cédric Toulotte <https://github.com/Pl0uf-tlt>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file sgp30.h
 * @defgroup sgp30 sgp30
 * 
 * ESP-IDF driver for Sensirion sgp30 air quality sensor 
 *
 * Copyright (c) 2024 Cédric Toulotte <https://github.com/Pl0uf-tlt>
 *
 * BSD Licensed as described in the file LICENSE
 */

#ifndef __SGP30_H__
#define __SGP30_H__

#include <stdbool.h>
#include <esp_err.h> 
#include <driver/i2c_master.h>

#define SGP30_I2C_ADDR 0x58

#define SGP30_DATA_SIZE 6
#define SGP30_RAW_DATA_SIZE 6
#define SGP30_IAQ_BASELINE_SIZE 6
#define SGP30_FEATURE_SET_SIZE 3
#define SGP30_TVOC_BASELINE_SIZE 3
#define SGP30_SERIAL_ID_SIZE 9

typedef uint8_t sgp30_data_t[SGP30_DATA_SIZE];
typedef uint8_t sgp30_raw_data_t[SGP30_RAW_DATA_SIZE];
typedef uint8_t sgp30_iaq_baseline_t[SGP30_IAQ_BASELINE_SIZE];
typedef uint8_t sgp30_feature_set_t[SGP30_FEATURE_SET_SIZE];
typedef uint8_t sgp30_tvoc_baseline_t[SGP30_TVOC_BASELINE_SIZE];
typedef uint8_t sgp30_serial_id_t[SGP30_SERIAL_ID_SIZE];

/**
 * Device descriptor
 */
typedef struct
{
    i2c_master_bus_handle_t *bus_mst_handle; //!< master bus handle 
    i2c_master_dev_handle_t *sgp30_dev_handle; //!< sgp's i2c device handle

    sgp30_data_t data;
    sgp30_raw_data_t raw_data;
    sgp30_iaq_baseline_t iaq_baseline;
    sgp30_feature_set_t feature_set;
    sgp30_tvoc_baseline_t tvoc_baseline;
    sgp30_serial_id_t serial_id;
    uint16_t tvoc;
    uint16_t co2;
} sgp30_t;

esp_err_t sgp30_measure(sgp30_t *dev, uint16_t *tvoc, uint16_t *co2);

esp_err_t sgp30_get_raw_data(sgp30_t *dev, sgp30_raw_data_t raw_data);

esp_err_t sgp30_init_desc(sgp30_t *dev, i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *sgp30_handle);

esp_err_t sgp30_init(sgp30_t *dev);

esp_err_t sgp30_compute_values(sgp30_data_t data, uint16_t *tvoc, uint16_t *co2);

/**
 * @brief Gets the IAQ algorithme baseline to stock for future usage
 * 
 * @note Thread safe
 * 
 * @param dev A pointer to the sgp30 device descriptor 
 * @param iaq_bl An array of 16 bits unsigned integers that represents the baseline for IAQ algorithme
 * 
 * @return ESP_OK if success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash
 */
esp_err_t sgp30_get_iaq_baseline(sgp30_t *dev, sgp30_iaq_baseline_t iaq_bl);

/**
 * @brief Sets the IAQ algorithme baseline. The command sgp30_iaq_init(dev) must have been sent before the use of sgp30_set_iaq_baseline(dev, iaq_bl)
 * 
 * @note Thread safe
 * 
 * @param dev A pointer to the sgp30 device descriptor 
 * @param iaq_bl An array of 16 bits unsigned integers that represents the baseline for IAQ algorithme
 * 
 * @return    ESP_OK on success - ESP_ERR_INVALIG_ARG if dev and/or iaq_bl don't exist - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash
 */
esp_err_t sgp30_set_IAQ_baseline(sgp30_t *dev, sgp30_iaq_baseline_t iaq_bl);

/**
 * @brief Sets the absolute humidity for the compensation algorithme
 * 
 * @note Thread safe
 * 
 * @param dev A pointer to the sgp30 device descriptor 
 * @param absolute_humidty A 8.8bits fixed point number representing the absolute humidity in g/m^3
 * 
 * @return ESP_OK on success - ESP_ERR_INVALIG_ARG if dev and/or iaq_bl don't exist - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash
 */
esp_err_t sgp30_set_humidity(sgp30_t *dev, uint32_t absolute_humidity);

esp_err_t sgp30_measure_test(sgp30_t *dev);

esp_err_t sgp30_get_feature_set(sgp30_t *dev, sgp30_feature_set_t feature_set); 

esp_err_t sgp30_measure_raw(sgp30_t *dev, sgp30_raw_data_t raw_data);

esp_err_t sgp30_get_tvoc_inceptive_baseline(sgp30_t *dev, sgp30_tvoc_baseline_t tvoc_baseline);

esp_err_t sgp30_set_tvoc_inceptive_baseline(sgp30_t *dev, sgp30_tvoc_baseline_t tvoc_baseline);

esp_err_t sgp30_get_serial_id(sgp30_t *dev, sgp30_serial_id_t serial_id);

#endif /* __SGP30_H__ */
