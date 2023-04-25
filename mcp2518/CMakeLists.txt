# Embed CA, certificate & key directly into binary
idf_component_register(SRCS "drv_spi.c" "drv_canfdspi_api.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver)
                    
