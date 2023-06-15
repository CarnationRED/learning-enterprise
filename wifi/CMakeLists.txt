# Embed CA, certificate & key directly into binary
idf_component_register(SRCS "wifi.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_wifi wpa_supplicant esp_event nvs_flash esp_netif lwip)
                    
