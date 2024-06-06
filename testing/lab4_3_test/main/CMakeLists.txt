idf_component_register(SRCS "ble_hidd_demo_main_test.c"
                            "esp_hidd_prf_api.c"
                            "hid_dev.c"
                            "hid_device_le_prf.c"
                            "icm42670.c"
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
