cmake_minimum_required(VERSION 3.20)

set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_OBJCOPY_UTIL arm-none-eabi-objcopy)
set(CMAKE_SIZE_UTIL arm-none-eabi-size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(BM_template_STM32G431 ASM C CXX)

set(CMAKE_SYSTEM_PROCESSOR cortex-m4)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(LINKER_FILE ${CMAKE_SOURCE_DIR}/startup/STM32G431RBTX_FLASH.ld)
set(STARTUP_FILE ${CMAKE_SOURCE_DIR}/startup/startup_stm32g431xx.s)
file(GLOB SOURCE_FILES CONFIGURE_DEPENDS "app/src/*.c" "system/src/*.c" "lib/hal/src/*.c")

add_executable(${PROJECT_NAME} ${STARTUP_FILE} ${SOURCE_FILES})
target_compile_definitions(${PROJECT_NAME} PRIVATE
        -DSTM32G431xx
        -DUSE_HAL_DRIVER
        -DUSE_FULL_ASSERT)
target_include_directories(${PROJECT_NAME} PRIVATE
        core app/inc system/inc lib/hal/inc)
target_compile_options(${PROJECT_NAME} PRIVATE
        -mcpu=${CMAKE_SYSTEM_PROCESSOR}
        $<$<COMPILE_LANGUAGE:C:-std=gnu${CMAKE_C_STANDARD}>:-std=gnu++${CMAKE_CXX_STANDARD}>
        -g -O2
        -mthumb
        -mfpu=fpv4-sp-d16 -mfloat-abi=hard
        -Wall -Wextra -Wconversion -Wshadow -Wunused -Wuninitialized -Wmissing-declarations -Wpointer-arith -Wlogical-op -Waggregate-return -Wfloat-equal
        -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections)
target_link_options(${PROJECT_NAME} PRIVATE
        -T ${LINKER_FILE}
        -mcpu=${CMAKE_SYSTEM_PROCESSOR}
        -mthumb
        -mfpu=fpv4-sp-d16 -mfloat-abi=hard
        --specs=nano.specs
        --specs=nosys.specs
        -lc -lm
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        -Xlinker -print-memory-usage -Xlinker)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY_UTIL} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY_UTIL} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_SIZE_UTIL} --format=berkeley ${PROJECT_NAME}.elf)
