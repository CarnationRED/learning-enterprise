# Embed CA, certificate & key directly into binary
idf_component_register(SRCS "spi.c" "ch395cmd.c" "ch395.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_wifi esp_hw_support wpa_supplicant esp_event nvs_flash esp_netif lwip mqtt)
                    