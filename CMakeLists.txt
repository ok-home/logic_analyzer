set( srcs 
    logic_analyzer_hal/logic_analyzer_hal.c
)
set( includedir 
    logic_analyzer_hal/include 
    )
set( priv_includedir 
    logic_analyzer_hal/private_include 
    )
set( priv_require 
    driver
    protocol_examples_common
    esp_http_server 
    esp_wifi 
    nvs_flash 
    esp_eth 
    esp_mm
    esp_psram 
    )

  if(IDF_TARGET STREQUAL "esp32")
    list(APPEND srcs
        logic_analyzer_hal/esp32/logic_analyzer_ll.c      
    )
    list(APPEND priv_includedir
        logic_analyzer_hal/esp32/private_include
    )
    if(CONFIG_ANALYZER_USE_HI_LEVEL_INTERRUPT)
        list( APPEND srcs 
            logic_analyzer_hal/esp32/logic_analyzer_triggered_isr_start.c
            logic_analyzer_hal/esp32/logic_analyzer_hi_interrupt_handler.s 
            )
    endif()
  endif()

  if(IDF_TARGET STREQUAL "esp32s3")
    list(APPEND srcs
        logic_analyzer_hal/esp32s3/logic_analyzer_ll.c      
    )
    list(APPEND priv_includedir
        logic_analyzer_hal/esp32s3/private_include
    )
    if(CONFIG_ANALYZER_USE_HI_LEVEL_INTERRUPT)
        list( APPEND srcs 
            logic_analyzer_hal/esp32s3/logic_analyzer_triggered_isr_start.c
            logic_analyzer_hal/esp32s3/logic_analyzer_hi_interrupt_handler.s 
            )
    endif()
  endif()

  if(IDF_TARGET STREQUAL "esp32c3")
    list(APPEND srcs
        logic_analyzer_hal/esp32c3/logic_analyzer_ll.c      
    )
    list(APPEND priv_includedir
        logic_analyzer_hal/esp32c3/private_include
    )
#    if(CONFIG_ANALYZER_USE_HI_LEVEL_INTERRUPT)
#        list( APPEND srcs 
#            logic_analyzer_hal/esp32c3/logic_analyzer_triggered_isr_start.c
#            )
#    endif()
  endif()

if(CONFIG_ANALYZER_USE_WS)
    list( APPEND srcs
        logic_analyzer_ws/logic_analyzer_ws.c
    )
    list( APPEND includedir 
        logic_analyzer_ws/include 
    )
    list( APPEND priv_includedir 
        logic_analyzer_ws/private_include 
    )
    set(embed  
        logic_analyzer_ws/logic_analyzer_ws.html
    )
    if(CONFIG_ANALYZER_START_WS_SERVER)
        list( APPEND srcs
            logic_analyzer_ws/logic_analyzer_ws_server.c
        )
    endif()
endif()

if(CONFIG_ANALYZER_USE_SUMP)
    list( APPEND srcs
        logic_analyzer_sump/logic_analyzer_sump.c
    )
    list( APPEND includedir 
        logic_analyzer_sump/include 
    )
    list( APPEND priv_includedir 
        logic_analyzer_sump/private_include 
    )
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${includedir}
    PRIV_INCLUDE_DIRS ${priv_includedir}
    #REQUIRES ${require}
    PRIV_REQUIRES ${priv_require}
    EMBED_FILES ${embed}
)
if( IDF_TARGET STREQUAL "esp32s3" OR IDF_TARGET STREQUAL "esp32" )
 if(CONFIG_ANALYZER_USE_HI_LEVEL_INTERRUPT)
        target_link_libraries(${COMPONENT_LIB} INTERFACE "-u la_include_hi_interrupt")
 endif()
endif()
