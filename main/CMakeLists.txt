# Embed CA, certificate & key directly into binary
idf_component_register(SRCS "uds.c" "wifi_enterprise_main.c" "oled.c" "can.c" "task.c" "fifo.c" "msg.c" "wifi.c" "mqtt.c" "cQueue.c" "doip.c" "led.c"
                    INCLUDE_DIRS "./include" "../components/u8g2/include" "../mcp2518" "../wifi" "../ch395"
                    REQUIRES driver esp_wifi wpa_supplicant esp_event nvs_flash esp_netif lwip mqtt esp_hw_support)
                    
