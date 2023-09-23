# Embed CA, certificate & key directly into binary
idf_component_register(SRCS 
# "../main/can.c"
                    INCLUDE_DIRS "." "../main/include" "../mcp2518" "../components/u8g2/include" "../ch395"
                    REQUIRES driver esp_wifi wpa_supplicant esp_event nvs_flash esp_netif lwip)
                    
