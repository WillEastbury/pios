cmake_minimum_required(VERSION 3.20)
project(pios_kepler_assembler C)

if(NOT ENVY_ROOT)
    message(FATAL_ERROR "Set ENVY_ROOT to envytools f102b82381f3f11cee113d16374c87091db039d9")
endif()
file(TO_CMAKE_PATH "${ENVY_ROOT}" ENVY_ROOT)
execute_process(COMMAND git -C "${ENVY_ROOT}" rev-parse HEAD
    OUTPUT_VARIABLE ENVY_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY)
if(NOT ENVY_REVISION STREQUAL "f102b82381f3f11cee113d16374c87091db039d9")
    message(FATAL_ERROR "Unexpected envytools revision")
endif()
include_directories("${ENVY_ROOT}/include")
add_compile_options(-std=gnu11)
add_subdirectory("${ENVY_ROOT}/util" util)
add_subdirectory("${ENVY_ROOT}/easm" easm)
add_subdirectory("${ENVY_ROOT}/envydis" envydis)
