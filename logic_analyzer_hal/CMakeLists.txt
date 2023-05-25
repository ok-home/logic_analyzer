idf_component_register(SRCS  "logic_analyzer_triggered_isr_start.c" "logic_analyzer_hal.c" "logic_analyzer_ll.c"
                            "fake_hi_interrupt_s.s"
                    INCLUDE_DIRS "./include"
                    REQUIRES driver
)
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u la_include_hi_interrupt")