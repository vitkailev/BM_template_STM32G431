cmake_minimum_required(VERSION 3.20)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1.0)

set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(BM_template_STM32G431 C ASM)

set(CMAKE_SYSTEM_PROCESSOR cortex-m4)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

set(CMAKE_C_STANDARD 11)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/startup/STM32G431RBTX_FLASH.ld)

set(CMAKE_ASM_FLAGS "-mcpu=${CMAKE_SYSTEM_PROCESSOR} -g3 -c -x assembler-with-cpp --specs=nano.specs -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=${CMAKE_SYSTEM_PROCESSOR} -std=gnu${CMAKE_C_STANDARD} -g3 -O0 -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -Wall -Wextra -Wpedantic -Wshadow -Wconversion --specs=nano.specs -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_EXE_LINKER_FLAGS "-mcpu=${CMAKE_SYSTEM_PROCESSOR} -g3 -T ${LINKER_SCRIPT} --specs=nosys.specs -Wl,-Map=${PROJECT_NAME}.map -Wl,--gc-sections -static --specs=nano.specs -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group")

set(CMAKE_C_LINK_EXECUTABLE "<CMAKE_C_COMPILER> <LINK_FLAGS> -o <TARGET> <OBJECTS>")

set(STARTUP_LOCATION ${CMAKE_SOURCE_DIR}/startup/startup_stm32g431xx.s)

include_directories(core)
include_directories(app/inc)
include_directories(system/inc)
include_directories(lib/hal/inc)

file(GLOB HEADER_FILES app/inc/*.h core/*.h system/inc/*.h lib/hal/inc/*.h lib/hal/inc/Legacy/*.h)
file(GLOB SOURCE_FILES app/src/*.c system/src/*.c lib/hal/src/*.c lib/hal/src/Legacy/*.c)

add_executable(${PROJECT_NAME} ${STARTUP_LOCATION} ${HEADER_FILES} ${SOURCE_FILES})

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY} ARGS -Oihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY} ARGS -Obinary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${SIZE} ARGS --format=berkeley ${PROJECT_NAME}.elf)
