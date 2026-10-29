idf_component_register(
    SRCS "remoteESP.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_wifi esp_netif esp_event nvs_flash lwip
             micro_ros_espidf_component
)
