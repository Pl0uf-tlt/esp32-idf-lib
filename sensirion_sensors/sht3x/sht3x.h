/*
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file sht3x.h
 * @defgroup sht3x sht3x
 * 
 *
 * ESP-IDF driver for Sensirion SHT3x digital temperature and humidity sensor
 *
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>\n
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2024 Cédric Toulotte <https://github.com/Pl0uf-tlt>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __SHT3X_H__
#define __SHT3X_H__

#include <stdbool.h>
#include <esp_err.h> 
#include <driver/i2c_master.h>

#define SHT3X_I2C_ADDR_GND 0x44
#define SHT3X_I2C_ADDR_VDD 0x45

#define SHT3X_RAW_DATA_SIZE 6
#define SHT3X_STATUS_SIZE 3

typedef uint8_t sht3x_raw_data_t[SHT3X_RAW_DATA_SIZE];
typedef uint8_t sht3x_status_t[SHT3X_STATUS_SIZE];

/**
 * Possible measurement modes
 */
typedef enum
{
    SHT3X_SINGLE_SHOT = 0,  //!< one single measurement
    SHT3X_PERIODIC_05MPS,   //!< periodic with 0.5 measurements per second (mps)
    SHT3X_PERIODIC_1MPS,    //!< periodic with   1 measurements per second (mps)
    SHT3X_PERIODIC_2MPS,    //!< periodic with   2 measurements per second (mps)
    SHT3X_PERIODIC_4MPS,    //!< periodic with   4 measurements per second (mps)
    SHT3X_PERIODIC_10MPS    //!< periodic with  10 measurements per second (mps)
} sht3x_mode_t;

/**
 * Possible repeatability modes
 */
typedef enum
{
    SHT3X_HIGH = 0,
    SHT3X_MEDIUM,
    SHT3X_LOW
} sht3x_repeat_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_master_bus_handle_t *bus_mst_handle; //!< master bus handle 
    i2c_master_dev_handle_t *sht_dev_handle; //!< sht's i2c device handle

    sht3x_raw_data_t raw_data;
    float temperature;
    float humidity;
    sht3x_status_t status;

    sht3x_mode_t mode;            //!< used measurement mode
    sht3x_repeat_t repeatability; //!< used repeatability
    bool measurement_started;
} sht3x_t;

/**
 * @brief Initializes the sht3x device descriptor 
 * 
 * @param dev        A pointer to the sht3x device descriptor 
 * @param bus_handle A pointer to the i2c master bus handle
 * @param sht_handle A pointer to the sht3x's i2c device handle
 * 
 * @return ESP_OK on success - ESP_ERR_INVALID_ARG if sht_handle or bus_handle doesn't exist
 */
esp_err_t sht3x_init_desc(sht3x_t *dev, i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *sht_handle);

/**
 * @brief Initializes the state of the sht3x sensor
 * 
 * @param dev A pointer to the sht3x device descriptor 
 * 
 * @return    ESP_OK on success - ESP_ERR_INVALID_ARG if dev doesn't exist
 */
esp_err_t sht3x_init(sht3x_t *dev);

/**
 * @brief Start the measurement in single shot or periodic mode with a chosen repeatability
 * 
 * The function starts the measurement either in *single shot mode*
 * (exactly one measurement) or *periodic mode* (periodic measurements)
 * with given repeatability.
 * 
 * In the *single shot mode*, this function has to be called for each
 * measurement. The measurement duration has to be waited every time
 * before the results can be fetched.
 *               
 * In the *periodic mode*, this function has to be called only once. Also
 * the measurement duration has to be waited only once until the first
 * results are available. After this first measurement, the sensor then
 * automatically performs all subsequent measurements. The rate of periodic
 * measurements can be 10, 4, 2, 1 or 0.5 measurements per second (mps).
 * 
 * @note Due to inaccuracies in timing of the sensor, the user task
 *       should fetch the results at a lower rate. The rate of the periodic
 *       measurements is defined by the parameter \p mode.
 * 
 * @note Thread safe
 *
 * @param dev    A pointer to the sht3x device descriptor 
 * @param mode   The measurement mode of the measure, see type ::sht3x_mode_t
 * @param repeat The repeatability mode of the measure, see type ::sht3x_repeat_t
 * 
 * @return       ESP_OK on success
 */
esp_err_t sht3x_start_measurement_single_shot(sht3x_t *dev, sht3x_repeat_t repeat);

/**
 * @brief Start the measurement in single shot or periodic mode with a chosen repeatability
 * 
 * The function starts the measurement either in *single shot mode*
 * (exactly one measurement) or *periodic mode* (periodic measurements)
 * with given repeatability.
 * 
 * In the *single shot mode*, this function has to be called for each
 * measurement. The measurement duration has to be waited every time
 * before the results can be fetched.
 *               
 * In the *periodic mode*, this function has to be called only once. Also
 * the measurement duration has to be waited only once until the first
 * results are available. After this first measurement, the sensor then
 * automatically performs all subsequent measurements. The rate of periodic
 * measurements can be 10, 4, 2, 1 or 0.5 measurements per second (mps).
 * 
 * @note Due to inaccuracies in timing of the sensor, the user task
 *       should fetch the results at a lower rate. The rate of the periodic
 *       measurements is defined by the parameter \p mode.
 * 
 * @note Thread safe
 *
 * @param dev    A pointer to the sht3x device descriptor 
 * @param mode   The measurement mode of the measure, see type ::sht3x_mode_t
 * @param repeat The repeatability mode of the measure, see type ::sht3x_repeat_t
 * 
 * @return       ESP_OK on success
 */
esp_err_t sht3x_start_periodic_measurement(sht3x_t *dev, sht3x_mode_t mode, sht3x_repeat_t repeat);

/**
 * @brief Read measurement results from sensor as raw data
 *
 * The function read measurement results from the sensor, checks the CRC
 * checksum and stores them in the byte array as following as unsigned byte integers.
 *
 *      data[0] = Temperature MSB
 *      data[1] = Temperature LSB
 *      data[2] = Temperature CRC
 *      data[3] = Humidity MSB
 *      data[4] = Humidity LSB
 *      data[2] = Humidity CRC
 *
 * In case that there are no new data that can be read, the function fails.
 *
 * @note Thread safe
 * 
 * @param dev      A pointer to the sht3x device descriptor
 * @param raw_data Byte array in which raw data are stored as unsigned integers
 *
 * @return         ESP_OK on success - ESP_ERR_INVALID_CRC if the crc failed - ESP_ERR_INVALID_STATE if measurement isn't started or is still running
 */
esp_err_t sht3x_fetch_data(sht3x_t *dev);

/**
 * @brief Turns the internal heater of the sht3x sensor on or off
 * 
 * @note Thread safe
 * 
 * @param dev    A pointer to the sht3x device descriptor 
 * @param enable True means turning the heater on - False means turning the heater off 
 * 
 * @return       ESP_OK on success - ESP_ERR_INVALID_ARG if dev doesn't exist
 */
esp_err_t sht3x_set_heater(sht3x_t *dev, bool enable);

/**
 * @brief Computes the real value of temperature and/or humidity from the raw data
 * 
 * @param raw_data    The raw_data coming from the sensor as an 8 bits unsigned integers array
 * @param temperature A pointer to a float in wich will be stock the temperature in °C
 * @param humidity    A pointer to a float in wich will be stock the relative humidity in %
 * 
 * @return            ESP_OK on success - ESP_ERR_INVALID_ARG if raw_data and/or both temperature and humidity don't exists
 */
esp_err_t sht3x_compute_values(sht3x_raw_data_t raw_data, float *temperature, float *humidity);

/**
 * @brief High level measurement function
 *
 * For convenience this function comprises all three steps to perform
 * one measurement in only one function:
 *
 * 1. Starts a measurement in single shot mode with high reliability
 * 2. Waits using `vTaskDelay()` until measurement results are available
 * 3. Returns the results in kind of floating point sensor values
 *
 * This function is the easiest way to use the sensor. It is most suitable
 * for users that don't want to have the control on sensor details.
 *
 * @note The function delays the calling task up to 30 ms to wait for
 *       the measurement results. This might lead to problems when function
 *       is called from a software timer callback function.
 * 
 * @note Thread safe
 *
 * @param dev         A pointer to the sht3x device descriptor 
 * @param temperature A pointer to a float in wich will be stock the temperature in °C
 * @param humidity    A pointer to a float in wich will be stock the relative humidity in %
 * 
 * @return            ESP_OK on success - ESP_ERR_INVALID_ARG if dev and/or both temperature and humidity don't exists
 */
esp_err_t sht3x_measure(sht3x_t *dev);

/**
 * @brief Stops the periodic mode measurements
 *
 * The function stops the measurements  in *periodic mode*
 * (periodic measurements) and the sensor returns in *single shot mode*
 *
 * @note Thread safe
 * 
 * @param dev A pointer to the sht3x device descriptor 
 * 
 * @return    ESP_OK on success
 */
esp_err_t sht3x_stop_periodic_measurement(sht3x_t *dev);

/**
 * @brief Compute the real value of temperature and/or humidity from the raw data
 *
 * The function combines function ::sht3x_get_raw_data() and function
 * ::sht3x_compute_values() to get the measurement results.
 *
 * In case that there are no results that can be read, the function fails.
 * 
 * @note Thread safe
 * 
 * @param dev         A pointer to the sht3x device descriptor
 * @param temperature A pointer to a float in wich will be stock the temperature in °C
 * @param humidity    A pointer to a float in wich will be stock the relative humidity in %
 * 
 * @return            ESP_OK on success - ESP_ERR_INVALID_ARG if both temperature and humidity don't exists
 */
esp_err_t sht3x_get_results(sht3x_t *dev);

esp_err_t sht3x_reset(sht3x_t *dev);

esp_err_t sht3x_read_status(sht3x_t *dev);

#endif /* __SHT3X_H__ */
