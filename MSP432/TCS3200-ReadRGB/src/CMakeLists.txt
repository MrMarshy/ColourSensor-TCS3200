cmake_minimum_required(VERSION 3.18.4)

message(STATUS "Adding CMakeLists inside src dir..")

set(LDSCRIPT "${CMAKE_SOURCE_DIR}/msp432p401r.lds")
set(LIBNAME "driverlib")
add_definitions(
	-D__MSP432P401R__ 
	-DTARGET_IS_MSP432P4XX 
	-DNO_MSP_CLASSIC_DEFINES
)


set(FP_FLAGS "-mfloat-abi=hard -mfpu=fpv4-sp-d16")
set(ARCH_FLAGS "-mthumb -mcpu=cortex-m4 ${FP_FLAGS}")

set(DRIVERLIB_DIR "${CMAKE_SOURCE_DIR}/lib/driverlib")

include_directories(${DRIVERLIB_DIR}/inc)
include_directories(${DRIVERLIB_DIR}/inc/CMSIS)

#link_directories(${DRIVERLIB_DIR}/lib)

set(CMAKE_ASM_FLAGS "-D__ASSEMBLY__ -mcpu=cortex-m4 -mthumb")
set(CMAKE_C_FLAGS " \
${COMMON_C_FLAGS} \
-fsingle-precision-constant -Wdouble-promotion -Wfloat-conversion \
${ARCH_FLAGS}")

set(CMAKE_CXX_FLAGS " \
${COMMON_CXX_FLAGS} \
-fsingle-precision-constant -Wdouble-promotion -Wfloat-conversion \
${ARCH_FLAGS}")

set(CMAKE_EXE_LINKER_FLAGS " \
${ARCH_FLAGS} \
-T${LDSCRIPT} \
--specs=nano.specs \
 -Wl,-Map=final.map")

#add_custom_target(libopencm3 make WORKING_DIRECTORY ${OPENCM3_DIR})

#add_library(driverlib STATIC IMPORTED)
#set_property(TARGET driverlib PROPERTY IMPORTED_LOCATION ${DRIVERLIB_DIR}/msp432p4xx_driverlib.a)

# Src 
# Conditional to change float computation between stm32 and testing host
#add_definitions(-DSTM32)

include_directories(../include)


file(GLOB USER_SOURCES 
    ${CMAKE_SOURCE_DIR}/src/*.cpp 
    ${CMAKE_SOURCE_DIR}/src/*.c
    #${CMAKE_SOURCE_DIR}/src/lcd/*.c 
    #${CMAKE_SOURCE_DIR}/src/shapes/*.cpp 
    #${CMAKE_SOURCE_DIR}/src/patterns/*.cpp
)

add_executable(final.elf ${USER_SOURCES})
add_executable(final.axf ${USER_SOURCES})
#target_link_libraries(final.elf driverlib)
#target_link_libraries(final.axf driverlib)

# Print out the firmware size after compiling and building
add_custom_target(
        print_firmware_size ALL
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/final.axf
        COMMAND python ${CMAKE_SOURCE_DIR}/get-fw-size.py ${CMAKE_SOURCE_DIR}/build/src/final.axf 0x44000 0x10000
)

