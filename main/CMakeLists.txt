file(GLOB_RECURSE SOURCES "./src/*.c" "./src/*.cpp")

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "." "./include")
                    
 littlefs_create_partition_image(littlefs ../flash_data FLASH_IN_PROJECT)
