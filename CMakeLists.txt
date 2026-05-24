idf_component_register(
    SRCS "vd56g3.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "private_include"
    PRIV_REQUIRES esp_driver_gpio esp_driver_i2c esp_cam_sensor
)

#if(CONFIG_CAMERA_VD56G3_AUTO_DETECT_MIPI_INTERFACE_SENSOR)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u vd56g3_detect")
#endif()