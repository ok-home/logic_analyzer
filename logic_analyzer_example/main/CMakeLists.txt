idf_component_register(
                SRCS 
                    "logic_analyzer_example.c"
                    "../logic_analyzer_test_sample/logic_analyzer_test_sample.c"
                REQUIRES  
                    #logic_analyzer_sump 
                    #logic_analyzer_ws
)