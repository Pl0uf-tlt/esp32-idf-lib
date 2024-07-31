# Driver for **SGP30** digital **air quality sensor**

The driver is to be used with an ESP32 and ESP-IDF 

## About this driver

This driver is based upon a (driver for the sgp30)[https://github.com/co-env/esp32_SGP30] by (IsaBologna)[https://github.com/IsaBologna] and (Renato Freitas)[https://github.com/freitas-renato]. The goal of the present version is to use the native i2c_master driver for i2cdev is deprecated and we had to declare our own i2c read and write functions. 

This driver has only been tested on **ESP32-H2** with **esp-idf v5.2.2**. I can't affirm that it will neither work or won't work with other version of esp-idf or with other soc.

## About the sensor

SGP30 is a digital air quality sensor that mesures H2 and ethanol concentration in the aire to determine the concentration oof total volatile organic compound (TVOC) and CO2. It uses an I2C interface with up to 400kHz communication speed. 

## Measurement process

Once the SGP30 sensor is initialized it needs 15 seconds to calibrate the algorithme it uses for air quality signal. During this time it will only send values of 400ppm of CO2 and 0ppb of TVOC. After this the user task will send a measurement command periodically, wait for the measurement to finish and fetch the data.  

## Initialization

The sensor needs to be initialized after each reset, it will spend 15 seconds to calibrate and then it will send actual measures. We can start the initialization with ```sgp30_init```. We first must have initialized the i2c bus and i2c device descriptor using ```sgp30_init_desc```.

## Measurement

Once the initialization finished the mesurement off tvoc and co2 can be started and the results fetched with ```sgp30_measure``` or function ```sgp30_measure_raw``` can be used to measure and fetch the raw results(H2 and Ethanol). 

These two functions fetch the data as bytes arrays and check the CRC for validation. 

## Error Handling

The driver uses the native error handling of the ESP32. It uses ```esp_err_t``` as return values to indicate success or error. 


## Usage

Before using the SGP30 driver's functions, both the bus and the sensor must be set up. Here is an example :

```C
#include "sgp30/sgp30.h"

//...

i2c_master_bus_handle_t bus_mst_handle;
i2c_master_dev_handle_t sgp_handle;

//...
i2c_master_bus_config_t bus_config = {
    .i2c_port = CONFIG_I2C_MASTER_PORT_NUM,
    .sda_io_num = CONFIG_I2C_MASTER_SDA,
    .scl_io_num = CONFIG_I2C_MASTER_SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .flags.enable_internal_pullup = 1,
};
i2c_device_config_t sgp_dev_cfg = {
    .device_address = SGP30_I2C_ADDR,
    .scl_speed_hz = CONFIG_I2C_MASTER_FREQUENCY
};

//...

i2c_new_master_bus(&bus_config, &bus_mst_handle);
i2c_master_bus_add_device(bus_mst_handle, &sgp_dev_cfg, &sgp_handle);

//...
```

Once I2C interfaces to be used are initialized, function ```sgp30_init_desc``` and ```sgp30_init``` have to be called for each SGP30 sensor to initialize the sensor. The parameters specify the I2C bus to which it is connected and its I2C master handle.

```C
static sgp30_t sgp_device;
i2c_master_bus_handle_t bus_mst_handle;
i2c_master_dev_handle_t sgp_handle;

//...

memset(&sgp_device, 0, sizeof(sgp30_t));
sgp30_init_desc(&sgp_device, &bus_mst_handle, &sgp_handle);
sgp30_init(&sgp_device);

//...
```

For 15 seconds the user needs to perform a measure once per second. like this :

```C
for (int i = 0; i < 16; i++) {
    sgp30_measure(&sgp_device);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("SGP30 still needs %d seconds to calibrate.\n", 15-i);
}
ESP_LOGI("sgp", "Initialisation du sgp30 faite");```

**Please note:** To avoid concurrency situations when driver functions are used to access the sensor, for example to read data, the user task must not be created until the sensor configuration is completed.

The user task needs to perform a periodic measurement. 

It can therefore look like this : 
```C
void sgp30_task() {
    i2c_device_config_t sgp30_dev_cfg = {
        .device_address = SGP30_I2C_ADDR,
        .scl_speed_hz = CONFIG_I2C_MASTER_FREQUENCY
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_mst_handle, &sgp30_dev_cfg, &sgp30_handle));
    memset(&sgp30_dev, 0, sizeof(sgp30_t));
    ESP_ERROR_CHECK(sgp30_init_desc(&sgp30_dev, &bus_mst_handle, &sgp30_handle));
    ESP_ERROR_CHECK(sgp30_init(&sgp30_dev));
    for (int i = 0; i < 16; i++) {
        sgp30_measure(&sgp30_dev, &tvoc, &co2);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("SGP30 still needs %d seconds to calibrate.\n", 15-i);
    }
    ESP_LOGI("sgp", "Initialisation du sgp30 faite");
    while (1)
    {
        ESP_ERROR_CHECK(sgp30_measure(&sgp30_dev));
        measured_co2 = sgp30_dev->co2;
        measured_voc = sgp30_dev->tvoc;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

```

## Full Example 

```C

/* -- includes ----------------------------------------------------- */

#include "sgp30.h"

#define TASK_STACK_DEPTH 2048

// I2C interface defintions for ESP32
#define I2C_BUS       CONFIG_I2C_MASTER_PORT_NUM
#define I2C_SCL_PIN   CONFIG_I2C_MASTER_SCL
#define I2C_SDA_PIN   CONFIG_I2C_MASTER_SDA
#define I2C_FREQ      CONFIG_I2C_MASTER_FREQUENCY

/* -- user tasks --------------------------------------------------- */

static sgp30_t sgp_device; /* Sgp30 sensor descriptor */
i2c_master_bus_handle_t bus_mst_handle;
i2c_master_dev_handle_t sgp_handle;

/*
 * User task that initializes the sensor and then triggers a measurement every 5 seconds.
 */
void user_task (void *pvParameters) {
    for (int i = 0; i < 16; i++) {
        sgp30_measure(&sgp30_dev, &tvoc, &co2);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("SGP30 still needs %d seconds to calibrate.\n", 15-i);
    }
    ESP_LOGI("sgp", "Initialisation du sgp30 faite");
    while (1)
    {
        ESP_ERROR_CHECK(sgp30_measure(&sgp30_dev));
        measured_co2 = sgp30_dev->co2;
        measured_voc = sgp30_dev->tvoc;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


/* -- main program ------------------------------------------------- */

void user_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = CONFIG_I2C_MASTER_PORT_NUM,
        .sda_io_num = CONFIG_I2C_MASTER_SDA,
        .scl_io_num = CONFIG_I2C_MASTER_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = 1,
    };
    i2c_device_config_t sgp_dev_cfg = {
        .device_address = SGP30_I2C_ADDR_GND,
        .scl_speed_hz = CONFIG_I2C_MASTER_FREQUENCY
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_mst_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_mst_handle, &sgp30_dev_cfg, &sgp30_handle));
    memset(&sgp30_dev, 0, sizeof(sgp30_t));
    ESP_ERROR_CHECK(sgp30_init_desc(&sgp30_dev, &bus_mst_handle, &sgp30_handle));
    ESP_ERROR_CHECK(sgp30_init(&sgp30_dev));

    // Create a user task that uses the sensor.
    xTaskCreate(user_task, "user_task", TASK_STACK_DEPTH, NULL, 2, NULL);

    // That's it.
}
```