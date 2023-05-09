idf_component_register(SRCS "logic_analyzer_sump.c"
                    INCLUDE_DIRS 
                        "./include" 
                    REQUIRES 
                        driver  
                        logic_analyzer_hal
)