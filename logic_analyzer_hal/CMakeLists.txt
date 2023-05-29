idf_component_register(SRCS  "logic_analyzer_triggered_isr_start.c" "logic_analyzer_hal.c" "logic_analyzer_ll.c"
                            "logic_analyzer_hi_interrupt_handler.s"
                    INCLUDE_DIRS "./include"
                    REQUIRES driver
)
if(CONFIG_ESP_SYSTEM_CHECK_INT_LEVEL_4)
        target_link_libraries(${COMPONENT_LIB} INTERFACE "-u la_include_hi_interrupt")
endif()