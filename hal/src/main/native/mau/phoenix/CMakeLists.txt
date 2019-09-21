cmake_minimum_required(VERSION 3.3.0)

project(CTRE_PhoenixPlatform)

# Specificy library installation location on target
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/frc/third-party/lib")

# Include header files (Phoenix includes, HAL includes, Mau Includes
include_directories(/home/pi/Phoenix-Linux-SocketCAN-Example/include ../../include/ ..)

# Create shared library
add_library(${PROJECT_NAME} SHARED Platform_mau.cpp)

find_library(WPI_HAL wpiHal PATHS "${CMAKE_CURRENT_BINARY_DIR}/../../../../../../lib")
message("CMAKE_CURRENT_BINARY_DIR:  ${CMAKE_CURRENT_BINARY_DIR}, WPI_HAL:  ${WPI_HAL}")
target_link_libraries(${PROJECT_NAME} PUBLIC "${WPI_HAL}")

# Install Library
install(TARGETS ${PROJECT_NAME} DESTINATION frc/third-party/lib)
