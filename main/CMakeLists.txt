idf_component_register(SRCS   "main.c"
                              "ctxlink.c"
                              "ctxlink_preferences.c"
                              "tasks/task_monitor.c"
                              "tasks/task_server.c"
                              "tasks/task_spi_comms.c"
                              "tasks/task_wifi.c"
                              "tasks/task_client.c"
                       PRIV_REQUIRES spi_flash driver esp_driver_spi esp_wifi ctxlink_spi_protocol nvs_flash
                       INCLUDE_DIRS ""
                                    "include"
                                    "tasks"
                        )
