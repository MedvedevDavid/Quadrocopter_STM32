set(COMPONENT_NAME MPU6050)
add_library(${COMPONENT_NAME})

target_sources(
    ${COMPONENT_NAME}
    PRIVATE
        MPU6050.cpp
)

# Compiler and linker options
target_compile_options(${COMPONENT_NAME} PUBLIC
    ${GLOBAL_COMPILER_OPTIONS}
    )

target_include_directories(
    ${COMPONENT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../../interfaces
)