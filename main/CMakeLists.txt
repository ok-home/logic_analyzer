#idf_component_register(SRCS "main_la_tst.c" "logic_analizer.c" "logic_analizer_ll.c"
#                    INCLUDE_DIRS "../include")

idf_component_register(SRCS "main_la.c" "logic_analizer.c" "logic_analizer_ll.c" "sump.c"
                    INCLUDE_DIRS "../include")
