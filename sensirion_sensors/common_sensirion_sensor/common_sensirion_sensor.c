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
 * @file common_sensirion_sensor.c
 *
 * ESP-IDF driver for Sensirion's sensors, it works at least with sht3x, sgp30 and sen5x sensors
 *
 * Copyright (c) 2024 Cédric Toulotte <https://github.com/Pl0uf-tlt>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "common_sensirion_sensor.h"

uint8_t crc8(uint8_t data[], int len)
{
    // initialization value
    uint8_t crc = 0xff;

    // iterate over all bytes
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int i = 0; i < 8; i++)
        {
            bool xor = crc & 0x80;
            crc = crc << 1;
            crc = xor ? crc ^ G_POLYNOM : crc;
        }
    }
    return crc;
}

esp_err_t execute_command(i2c_master_dev_handle_t *dev, uint8_t cmd[], uint8_t command_len, uint16_t delay, uint8_t *read_data, uint8_t read_len) 
{
    // Tries to write the commande and the parameters associated to it to the device
    CHECK(i2c_master_transmit(*dev, &cmd[0], command_len, pdMS_TO_TICKS(CONFIG_I2C_TIMEOUT)));

    // Waits for device to process command and measure desired value
    vTaskDelay(delay / portTICK_PERIOD_MS);

    // Checks if there is data to be read from the user, (or if it's just a simple write command)
    if (read_len == 0) {
        return ESP_OK;
    }

    // Tries to read device reply
    CHECK(i2c_master_receive(*dev, read_data, read_len, pdMS_TO_TICKS(CONFIG_I2C_TIMEOUT)));

    // Calculates expected CRC and compares it with the response
    for (uint8_t i = 0; i*3 < read_len; i++) {
        if (crc8(read_data + i * 3, 2) != read_data[2 + i * 3])
        {
            return ESP_ERR_INVALID_CRC;
        }
    }
    return ESP_OK;
}








