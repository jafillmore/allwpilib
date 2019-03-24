cmake_minimum_required(VERSION 3.3.0)

project(CTRE_PhoenixPlatform)

# Specificy library installation location on target
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/frc/third-party/lib")

# Include header files (VMX-pi includes, Phoenix includes, HAL includes, Mau Includes
include_directories(/usr/local/include/vmxpi /home/pi/Phoenix-Linux-SocketCAN-Example/include ../../include/ ..)

# Create shared library
add_library(${PROJECT_NAME} SHARED Platform_mau.cpp)

# Install Library
install(TARGETS ${PROJECT_NAME} DESTINATION frc/third-party/lib)
