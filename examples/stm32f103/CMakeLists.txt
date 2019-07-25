cmake_minimum_required(VERSION 3.13)

set(LINKER_FLAGS "-T ${CMAKE_CURRENT_SOURCE_DIR}/stm32.ld --static -nostartfiles -specs=nano.specs -specs=nosys.specs")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_EXE_LINKER_FLAGS "${LINKER_FLAGS}")

include_directories(libopencm3/include/)
link_directories(libopencm3/lib/)

add_executable(controller.elf main.c)
target_link_libraries(controller.elf enc28j60 opencm3_stm32f1)

add_custom_target(controller.bin ALL arm-none-eabi-objcopy -O binary controller.elf controller.bin)
add_dependencies(controller.bin controller.elf)
