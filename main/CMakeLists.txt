if(CONFIG_PSX_ESP_ENABLE_WEBSERVER)
    set(html_file ${PROJECT_DIR}/build/index.html.gz)
endif()

idf_component_register(SRCS "psx_serial_tunnel.c"
                    INCLUDE_DIRS "."
                    EMBED_FILES "${html_file}")

if (CONFIG_PSX_ESP_ENABLE_WEBSERVER)
    add_custom_command(OUTPUT ${PROJECT_DIR}/build/index.html.gz
         COMMAND gzip -k -9 -c ${PROJECT_DIR}/main/index.html > ${PROJECT_DIR}/build/index.html.gz
         DEPENDS ${PROJECT_DIR}/main/index.html
         VERBATIM)
    add_custom_target(gen_gz_files
                      DEPENDS ${PROJECT_DIR}/build/index.html.gz)
    add_dependencies(${COMPONENT_LIB} gen_gz_files)

    set_property(DIRECTORY "${COMPONENT_DIR}" APPEND PROPERTY
            ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_DIR}/build/display.html.gz)
endif()
