idf_component_register(SRCS "main.c" "../Application_Layer/calibration.c" "../Application_Layer/measurement.c"
                    INCLUDE_DIRS "." "../Application_Layer" "../Middle_Ware"
                    REQUIRES Middle_Ware driver)
