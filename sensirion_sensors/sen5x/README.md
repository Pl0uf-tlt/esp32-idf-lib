# Driver for **SEN5X** digital **air quality sensor**

The driver is to be used with an ESP32 and ESP-IDF 

## About this driver

This driver uses the native i2c_master library. It has been made with the same logic that the drivers for sht3x and sgp30. 

This driver has only been tested on **ESP32-H2** with **esp-idf v5.2.2**. I can't affirm that it will neither work or won't work with other version of esp-idf or with other soc.

This driver is based on this (datasheet)[https://sensirion.com/media/documents/6791EFA0/62A1F68F/Sensirion_Datasheet_Environmental_Node_SEN5x.pdf] that one can find on (the product page)[https://sensirion.com/products/catalog/SEN50].

## About the sensor

SEN5X is a digital air quality sensor that measures particulate matter, VOC, NOx, humidity, and temperature. There are 3 types of SEN5X, the SEN50, the SEN54 and the SEN55. This driver isn't complete, it lacks fonctions oriented toward VOC, NOCx and RH/T. It is meant for SEN50 only.

## Measurement process

Once the SEN5X sensor is initialized we can start a measurement. In measurement mode it will have new values periodically every second. The user can therfore fetch new data every seconds.   

## Initialization

The I2C initialization is done partially using ```sen5x_init_desc```. Then the function ```sen5x_init``` is used to initialized the device, it fetches the product name, the serial number and the firmware version. 

## Measurement

Once the device is initialized we can start the measurement mode using ```sen5x_start_measurement```. New values will be available every seconds. The data ready flag will be set when new values are available. User can either fetch data directly with ```sen5x_read_measured_values``` or he can check if new values are available with ```sen5x_read_data_ready_flag``` before fetching it with ```sen5x_read_measured_values```. 
These two functions fetch the data as bytes arrays and check the CRC for validation. 

## Error Handling

The driver uses the native error handling of the ESP32. It uses ```esp_err_t``` as return values to indicate success or error. 


## Usage

Before using the SEN5X driver's functions, both the bus and the sensor must be set up. Here is an example :

```C
#include "sen5x/sen5x.h"

//...

i2c_master_bus_handle_t bus_mst_handle;
i2c_master_dev_handle_t sen_handle;

//...
i2c_master_bus_config_t bus_config = {
    .i2c_port = CONFIG_I2C_MASTER_PORT_NUM,
    .sda_io_num = CONFIG_I2C_MASTER_SDA,
    .scl_io_num = CONFIG_I2C_MASTER_SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .flags.enable_internal_pullup = 1,
};
i2c_device_config_t sen_dev_cfg = {
    .device_address = SEN5X_I2C_ADDR,
    .scl_speed_hz = CONFIG_I2C_MASTER_FREQUENCY
};

//...

i2c_new_master_bus(&bus_config, &bus_mst_handle);
i2c_master_bus_add_device(bus_mst_handle, &sen_dev_cfg, &sen_handle);

//...
```

Once I2C interfaces to be used are initialized, function ```sen5x_init_desc``` and ```sen5x_init``` have to be called for each SEN5X sensor to initialize the sensor. The parameters specify the I2C bus to which it is connected and its I2C master handle.

```C
static sen5x_t sen_device;
i2c_master_bus_handle_t bus_mst_handle;
i2c_master_dev_handle_t sen_handle;

//...

memset(&sen_device, 0, sizeof(sen5x_t));
sen5x_init_desc(&sen_device, &bus_mst_handle, &sen_handle);
sen5x_init(&sen_device);

//...
```


**Please note:** To avoid concurrency situations when driver functions are used to access the sensor, for example to read data, the user task must not be created until the sensor configuration is completed.

The user task needs to perform a periodic measurement. 

It can therefore look like this : 
```C
void sen5x_task() {
    i2c_device_config_t sen5x_dev_cfg = {
        .device_address = SEN5X_I2C_ADDR,
        .scl_speed_hz = CONFIG_I2C_MASTER_FREQUENCY
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_mst_handle, &sen5x_dev_cfg, &sen5x_handle));
    memset(&sen5x_dev, 0, sizeof(sen5x_t));
    ESP_ERROR_CHECK(sen5x_init_desc(&sen5x_dev, &bus_mst_handle, &sen5x_handle));
    ESP_ERROR_CHECK(sen5x_init(&sen5x_dev));
    }
    ESP_LOGI("sen", "Initialization done");

    ESP_ERROR_CHECK(sen5x_start_measurement(&sen5x_dev));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while (1)
    {
        ESP_ERROR_CHECK(sen5x_read_measured_values(&sen5x_dev));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

```

## Full Example 

```C

/* -- includes ----------------------------------------------------- */

#include "sen5x.h"

#define TASK_STACK_DEPTH 2048

// I2C interface defintions for ESP32
#define I2C_BUS       CONFIG_I2C_MASTER_PORT_NUM
#define I2C_SCL_PIN   CONFIG_I2C_MASTER_SCL
#define I2C_SDA_PIN   CONFIG_I2C_MASTER_SDA
#define I2C_FREQ      CONFIG_I2C_MASTER_FREQUENCY

/* -- user tasks --------------------------------------------------- */

static sen5x_t sen_device; /* Sen30 sensor descriptor */
i2c_master_bus_handle_t bus_mst_handle;
i2c_master_dev_handle_t sen_handle;

/*
 * User task that initializes the sensor and then triggers a measurement every 5 seconds.
 */
void user_task (void *pvParameters) {
    ESP_ERROR_CHECK(sen5x_start_measurement(&sen5x_dev));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while (1)
    {
        ESP_ERROR_CHECK(sen5x_read_measured_values(&sen5x_dev));
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
    i2c_device_config_t sen_dev_cfg = {
        .device_address = SEN5X_I2C_ADDR_GND,
        .scl_speed_hz = CONFIG_I2C_MASTER_FREQUENCY
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_mst_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_mst_handle, &sen5x_dev_cfg, &sen5x_handle));
    memset(&sen5x_dev, 0, sizeof(sen5x_t));
    ESP_ERROR_CHECK(sen5x_init_desc(&sen5x_dev, &bus_mst_handle, &sen5x_handle));
    ESP_ERROR_CHECK(sen5x_init(&sen5x_dev));

    // Create a user task that uses the sensor.
    xTaskCreate(user_task, "user_task", TASK_STACK_DEPTH, NULL, 2, NULL);

    // That's it.
}
```