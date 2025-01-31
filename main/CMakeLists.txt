idf_component_register(SRCS "main.c" "bme280/bme280.c"
                    INCLUDE_DIRS "." "bme280/"
					REQUIRES "driver" "esp_timer")
