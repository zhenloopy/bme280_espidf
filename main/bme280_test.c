#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "bme280.h"
#include <rom/ets_sys.h>

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define BME280_ADDR 0x76

static const char *TAG = "MAIN";

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

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


/* BME280 SPECIFIC LOGIC */

BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len) 
{
	return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}


BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, uint8_t *data)
{
	uint8_t write_buf[2] = {reg_addr, *data};
	return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void bme280_delay_us(uint32_t period, void *intf_ptr) {
	ets_delay_us(period);
}


void bme280_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BME280_OK)
    {
        printf("%s\t", api_name);

        switch (rslt)
        {
            case BME280_E_NULL_PTR:
                printf("Error [%d] : Null pointer error.", rslt);
                printf(
                    "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
                break;

            case BME280_E_COMM_FAIL:
                printf("Error [%d] : Communication failure error.", rslt);
                printf(
                    "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
                break;

            case BME280_E_DEV_NOT_FOUND:
                printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                       rslt);
                break;

            case BME280_E_INVALID_LEN:
                printf("Error [%d] : Invalid length error. It occurs when write is done with invalid length\r\n", rslt);
                break;

            default:
                printf("Error [%d] : Unknown error code\r\n", rslt);
                break;
        }
    }
}

void taskMeasure(void * params) {
	struct bme280_dev dev;
	struct bme280_settings settings;
	uint32_t period;
	struct bme280_data comp_data;
	int8_t rslt;

	dev.delay_us = &bme280_delay_us;
	dev.read = &bme280_i2c_read;
	dev.write = &bme280_i2c_write;
	dev.intf = BME280_I2C_INTF;
	dev.chip_id = BME280_I2C_ADDR_PRIM;

	rslt = bme280_init(&dev);
	bme280_error_codes_print_result("bme280_init", rslt);

	rslt = bme280_get_sensor_settings(&settings, &dev);
	bme280_error_codes_print_result("bme280_get_sensor_settings", rslt);

	settings.filter = BME280_FILTER_COEFF_OFF;
	settings.osr_h = BME280_OVERSAMPLING_1X;
	settings.osr_p = BME280_OVERSAMPLING_1X;
	settings.osr_t = BME280_OVERSAMPLING_1X;
	uint8_t changeSettings = BME280_SEL_ALL_SETTINGS;

	rslt = bme280_set_sensor_settings(changeSettings, &settings, &dev);
	bme280_error_codes_print_result("bme280_set_sensor_settings", rslt);

		uint8_t status;
		uint8_t readSettings[1];
		uint8_t writeSettings[1] = {32};

		status = bme280_i2c_write(0xF4, writeSettings);
		status = bme280_i2c_read(0xF4, readSettings, 1);

		printf("what's in our settings config MANUAL WRITE: %u\n\n", readSettings[0]);
	


	rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &dev);
	bme280_error_codes_print_result("bme280_set_sensor_mode", rslt);
	rslt = bme280_cal_meas_delay(&period, &settings);

	while(1) {
		rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &dev);
		bme280_delay_us(period + 1000, &(dev.intf_ptr));

		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
		bme280_error_codes_print_result("bme280_get_sensor_data", rslt);

		printf("temperature: %f\n", comp_data.temperature);
		printf("humidity: %f\n", comp_data.humidity);
		printf("pressure: %f\n\n", comp_data.pressure);

		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}
/* MAIN */

void app_main(void)
{
	i2c_master_init(&bus_handle, &dev_handle);
	
	xTaskCreate(&taskMeasure, "take and print measurements", 2048, NULL, 6, NULL);

}
