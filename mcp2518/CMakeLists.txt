# Embed CA, certificate & key directly into binary
idf_component_register(SRCS "drv_canfdspi_api.c" "drv_spi.c"
                    INCLUDE_DIRS ".")
                    
