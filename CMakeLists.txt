cmake_minimum_required(VERSION 3.15)

project(test_g4_peripherals LANGUAGES C CXX ASM)

find_bsp(ID com.sysprogs.project_importers.stm32.cubemx SOURCE_PROJECT test_g4_peripherals.ioc)

bsp_include_directories(Application/Inc)
add_bsp_based_executable(
	NAME test_g4_peripherals
	GENERATE_BIN
	GENERATE_MAP
	SOURCES
		${BSP_ROOT}/Application/Src/Integer.cpp
		${BSP_ROOT}/Application/Src/Measure.cpp
		${BSP_ROOT}/Application/Src/Serial.cpp
		${BSP_ROOT}/Application/Src/Command.cpp
		${BSP_ROOT}/Application/Src/Loop.cpp)

# Add Application/Inc to the include path
target_include_directories(test_g4_peripherals PRIVATE
	Application/Inc
	Application/Src)
