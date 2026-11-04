cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME BIUBIUBIU)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
App/Config/config.h
App/Bsp/bsp_rc.h
App/Bsp/bsp_rc.c
App/Driver/drv_dbus.h
App/Driver/drv_dbus.c
App/Task/task_rc.h
App/Task/task_rc.c
App/Bsp/bsp_can.h
App/Bsp/bsp_can.c)
# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/App/*.c)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${APP_SOURCES})

# 头文件搜索路径：每一层都要加，这样 #include "safety.h" 才能找到
# （放在子目录里的头文件，靠这几行"摊平"到搜索路径上，不用改任何 #include）
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/App
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Config
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Bsp
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Driver
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Task
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Application
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Algorithm
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Devices)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

# 用了 retarget.c 的 printf 重定向，必须屏蔽"半主机"
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -specs=nosys.specs)