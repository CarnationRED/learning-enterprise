# Embed CA, certificate & key directly into binary
idf_component_register(SRCS "wifi_enterprise_main.c" "oled.c" "can.c" "task.c" "fifo.c" "msg.c" "wifi.c" "mqtt.c" "cQueue.c"
                    INCLUDE_DIRS "./include" "../components/u8g2/include" "../mcp2518" "../wifi"
                    REQUIRES driver esp_wifi wpa_supplicant esp_event nvs_flash esp_netif lwip mqtt)
                    
