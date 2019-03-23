project(hal)

file(STRINGS src/generate/Instances.txt RAW_INSTANCES)
file(STRINGS src/generate/ResourceType.txt RAW_RESOURCE_TYPES)

SET(usage_reporting_types_cpp "")
SET(usage_reporting_instances_cpp "")
SET(usage_reporting_types "")
SET(usage_reporting_instances "")

foreach(ITEM ${RAW_INSTANCES})
    list(APPEND usage_reporting_instances_cpp "    ${ITEM},")
    list(APPEND usage_reporting_instances "\n    public static final int ${ITEM};")
endforeach()

foreach(ITEM ${RAW_RESOURCE_TYPES})
    list(APPEND usage_reporting_types_cpp "    ${ITEM},")
    list(APPEND usage_reporting_types "\n    public static final int ${ITEM};")
endforeach()

string(REPLACE ";" "\n" usage_reporting_types_cpp "${usage_reporting_types_cpp}")
string(REPLACE ";" "\n" usage_reporting_instances_cpp "${usage_reporting_instances_cpp}")

# Copy ctre sources to mau
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/main/native/mau/ctre)
file(GLOB CTRE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main/native/athena/ctre/*)
file(COPY ${CTRE_SOURCES} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/src/main/native/mau/ctre)

# Java bindings
if (NOT WITHOUT_JAVA)
    find_package(Java REQUIRED)
    find_package(JNI REQUIRED)
    include(UseJava)
    set(CMAKE_JAVA_COMPILE_FLAGS "-Xlint:unchecked")

    file(GLOB
        hal_shared_jni_src src/main/native/cpp/jni/*.cpp
        hal_sim_jni_src src/main/native/sim/jni/*.cpp)

    file(GLOB_RECURSE JAVA_SOURCES
        src/generated/java/*.java
        src/main/java/*.java)
    set(CMAKE_JNI_TARGET true)

    if(${CMAKE_VERSION} VERSION_LESS "3.11.0")
        set(CMAKE_JAVA_COMPILE_FLAGS "-h" "${CMAKE_CURRENT_BINARY_DIR}/jniheaders")
        add_jar(hal_jar ${JAVA_SOURCES} INCLUDE_JARS wpiutil_jar OUTPUT_NAME wpiHalMau)
    else()
        add_jar(hal_jar ${JAVA_SOURCES} INCLUDE_JARS wpiutil_jar OUTPUT_NAME wpiHalMau GENERATE_NATIVE_HEADERS hal_jni_headers)
    endif()

    get_property(HAL_JAR_FILE TARGET hal_jar PROPERTY JAR_FILE)
    install(FILES ${HAL_JAR_FILE} DESTINATION "${java_lib_dest}")

    set_property(TARGET hal_jar PROPERTY FOLDER "java")

endif()

file(GLOB
    hal_shared_native_src src/main/native/cpp/cpp/*.cpp
    hal_shared_native_src src/main/native/cpp/handles/*.cpp
    hal_sim_native_src src/main/native/mau/*.cpp
    hal_sim_native_src src/main/native/mau/Translator/*.cpp
    hal_sim_native_src src/main/native/mau/DriveStation/*.cpp
    hal_sim_native_src src/main/native/mau/ctre/*.cpp
    hal_sim_native_src src/main/native/mau/phoenix/*.cpp)
add_library(hal ${hal_shared_native_src} ${hal_shared_jni_src})

if(USE_EXTERNAL_HAL)
    include(${EXTERNAL_HAL_FILE})
else()
    target_sources(hal PRIVATE ${hal_sim_native_src})
endif()

configure_file(src/generate/FRCUsageReporting.h.in gen/hal/FRCUsageReporting.h)

set_target_properties(hal PROPERTIES OUTPUT_NAME "wpiHalMau")

target_include_directories(hal PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/main/native/include>
                            $<INSTALL_INTERFACE:${include_dest}/hal>)

target_include_directories(hal PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/gen>
			    $<INSTALL_INTERFACE:${include_dest}/hal>)

target_include_directories(hal PUBLIC /usr/local/include/vmxpi)
target_include_directories(hal PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/main/native/include/HAL/handles>)
target_include_directories(hal PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/main/native/include/HAL>)
target_include_directories(hal PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/main/native/mau>)
target_include_directories(hal PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/main/native/mau/DriveStation/include>)

target_include_directories(hal PUBLIC 
		$<BUILD_INTERFACE:~/Phoenix-Linux-SocketCAN-Example/include>)

target_link_libraries(hal PUBLIC wpiutil)

set_property(TARGET hal PROPERTY FOLDER "libraries")

if (NOT WITHOUT_JAVA)
    if(${CMAKE_VERSION} VERSION_LESS "3.11.0")
        target_include_directories(hal PRIVATE ${JNI_INCLUDE_DIRS})
        target_include_directories(hal PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/jniheaders")
    else()
        target_link_libraries(hal PRIVATE hal_jni_headers)
    endif()
    add_dependencies(hal hal_jar)
endif()

install(TARGETS hal EXPORT hal DESTINATION "${main_lib_dest}")
install(DIRECTORY src/main/native/include/ DESTINATION "${include_dest}/hal")
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/gen DESTINATION "${include_dest}/hal")

if (NOT WITHOUT_JAVA AND MSVC)
    install(TARGETS hal RUNTIME DESTINATION "${jni_lib_dest}" COMPONENT Runtime)
endif()

if (MSVC)
    set (hal_config_dir ${wpilib_dest})
else()
    set (hal_config_dir share/hal)
endif()

install(FILES hal-config.cmake DESTINATION ${hal_config_dir})
install(EXPORT hal DESTINATION ${hal_config_dir})