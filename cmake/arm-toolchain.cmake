# CMake 工具链文件 - ARM Cortex-M7
# 用于交叉编译 STM32H743 项目

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

if(WIN32)
    set(_ARM_CANDIDATES
        "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/14.2 rel1/bin"
        "C:/Program Files (x86)/GNU Tools Arm Embedded/9 2019-q4-major/bin"
        "C:/Program Files/GNU Tools Arm Embedded/9 2019-q4-major/bin"
    )
    set(ARM_TOOLCHAIN_PREFIX "")
    foreach(_cand IN LISTS _ARM_CANDIDATES)
        if(EXISTS "${_cand}/arm-none-eabi-gcc.exe")
            set(ARM_TOOLCHAIN_PREFIX "${_cand}")
            set(_ARM_GCC_SUFFIX ".exe")
            break()
        elseif(EXISTS "${_cand}/arm-none-eabi-gcc")
            set(ARM_TOOLCHAIN_PREFIX "${_cand}")
            set(_ARM_GCC_SUFFIX "")
            break()
        endif()
    endforeach()
    if(ARM_TOOLCHAIN_PREFIX STREQUAL "")
        message(FATAL_ERROR "arm-none-eabi-gcc not found. Install GNU Arm Embedded Toolchain.")
    endif()
else()
    set(_ARM_CANDIDATES
        "/opt/arm-none-eabi/bin"
        "/usr/bin"
    )
    set(ARM_TOOLCHAIN_PREFIX "")
    foreach(_cand IN LISTS _ARM_CANDIDATES)
        if(EXISTS "${_cand}/arm-none-eabi-gcc")
            set(ARM_TOOLCHAIN_PREFIX "${_cand}")
            set(_ARM_GCC_SUFFIX "")
            break()
        endif()
    endforeach()
    if(ARM_TOOLCHAIN_PREFIX STREQUAL "")
        set(ARM_TOOLCHAIN_PREFIX "/usr/bin")
    endif()
endif()

if(NOT DEFINED _ARM_GCC_SUFFIX)
    set(_ARM_GCC_SUFFIX "")
endif()

set(CMAKE_C_COMPILER   "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-gcc${_ARM_GCC_SUFFIX}"   CACHE FILEPATH "C compiler" FORCE)
set(CMAKE_CXX_COMPILER "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-g++${_ARM_GCC_SUFFIX}"   CACHE FILEPATH "C++ compiler" FORCE)
set(CMAKE_ASM_COMPILER "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-gcc${_ARM_GCC_SUFFIX}"   CACHE FILEPATH "ASM compiler" FORCE)
set(CMAKE_LINKER       "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-gcc${_ARM_GCC_SUFFIX}"   CACHE FILEPATH "Linker" FORCE)
set(CMAKE_AR           "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-ar${_ARM_GCC_SUFFIX}"    CACHE FILEPATH "Archiver" FORCE)
set(CMAKE_OBJCOPY      "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-objcopy${_ARM_GCC_SUFFIX}" CACHE FILEPATH "Objcopy" FORCE)
set(CMAKE_OBJDUMP      "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-objdump${_ARM_GCC_SUFFIX}" CACHE FILEPATH "Objdump" FORCE)
set(CMAKE_SIZE         "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-size${_ARM_GCC_SUFFIX}"  CACHE FILEPATH "Size" FORCE)

set(CPU_PARAMETERS
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
)

set(CMAKE_C_FLAGS_INIT "${CPU_PARAMETERS} -Wall -Wextra -Wno-unused-parameter")
set(CMAKE_C_FLAGS_DEBUG_INIT "-g3 -O0")
set(CMAKE_C_FLAGS_RELEASE_INIT "-Os -DNDEBUG")

set(CMAKE_EXE_LINKER_FLAGS_INIT "${CPU_PARAMETERS} -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
