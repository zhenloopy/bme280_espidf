idf_component_register(SRCS "bme280_drivers.c" "bme280.c"
                    INCLUDE_DIRS "."
					REQUIRES "driver" "esp_timer")
