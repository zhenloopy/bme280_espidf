#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define BME280_ADDR 0x76
#define BME280_WHO_AM_I_ADDR 0xD0
#define BME280_POWER_MGMT_ADDR 0x74 // bits 1, 0
#define BME280_TEMP_ADDR 0xFB
#define BME280_CTRL_TEMP_ADDR 0xF4 // bits 7, 6, 5
#define BME280_HUM_ADDR 0xFE
#define BME280_CTRL_HUM_ADDR 0xF2 // bits 2, 1, 0
#define BME280_PRESS_ADDR 0xF8
#define BME280_CTRL_PRESS_ADDR 0xF4 // bits 4, 3, 2
#define BME280_RESET_ADDR 0xE0 // write as 0xB6 to reset

static const char *TAG = "MAIN";


/* I2C INIT AND DEINIT */

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));

}

static void i2c_master_deinit(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t dev_handle) {
	ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
	ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
}


/* BME280 SPECIFIC LOGGING */

static esp_err_t bme280_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len) 
{
	return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t bme280_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
	uint8_t write_buf[2] = {reg_addr, data};
	return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

struct weatherData {
	int32_t temp;
	uint32_t hum;
	uint32_t press;
};

struct weatherData bme280_weather_monitor(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t dev_handle) {
	struct weatherData data;
	data.temp = 0;
	data.hum = 0;
	data.press = 0;
	return data;
}


/* MAIN */

void app_main(void)
{
	while(1) {
		uint8_t whoami[2];
		uint8_t powerSet[2];
		
		uint8_t temp[1];
		uint8_t hum[1];
		uint8_t press[1];

		i2c_master_bus_handle_t bus_handle;
		i2c_master_dev_handle_t dev_handle;
		i2c_master_init(&bus_handle, &dev_handle);
		ESP_LOGI(TAG, "I2C initialized successfully\n");

		ESP_ERROR_CHECK(bme280_register_read(dev_handle, BME280_WHO_AM_I_ADDR, whoami, 1)); //data[0] is who_am_i value
		
		ESP_ERROR_CHECK(bme280_register_read(dev_handle, BME280_POWER_MGMT_ADDR, powerSet, 1));
		ESP_LOGI(TAG, "current full 0xF4 register: %b", powerSet[0]);

		ESP_ERROR_CHECK(bme280_register_write_byte(dev_handle, BME280_POWER_MGMT_ADDR, 3));

		i2c_master_deinit(bus_handle, dev_handle);
		ESP_LOGI(TAG, "I2C deinitialized successfully\n");

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
