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

# Copy ctre sources to mau (this allows mau to reuse the CTRE PCM code in athena)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/main/native/mau/ctre)
file(GLOB CTRE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main/native/athena/ctre/*)
file(COPY ${CTRE_SOURCES} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/src/main/native/mau/ctre)

# Java bindings
if (NOT WITHOUT_JAVA)
    find_package(Java REQUIRED)
    find_package(JNI REQUIRED)
    include(UseJava)
    set(CMAKE_JAVA_COMPILE_FLAGS "-Xlint:unchecked")

    configure_file(src/generate/FRCNetComm.java.in FRCNetComm.java)

    file(GLOB
        hal_shared_jni_src src/main/native/cpp/jni/*.cpp
	hal_shared_jni_src src/main/native/mau/navX/jni/*.cpp)
#        hal_sim_jni_src src/main/native/sim/jni/*.cpp)

    file(GLOB_RECURSE JAVA_SOURCES
        ${CMAKE_CURRENT_BINARY_DIR}/FRCNetComm.java
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

    add_library(haljni ${hal_shared_jni_src})

    if(USE_EXTERNAL_HAL)
        include(${EXTERNAL_HAL_FILE})
    else()
        target_sources(haljni PRIVATE ${hal_sim_jni_src})
    endif()

    set_target_properties(haljni PROPERTIES OUTPUT_NAME "wpiHaljni")

    target_link_libraries(haljni PUBLIC hal wpiutil)

    set_property(TARGET haljni PROPERTY FOLDER "libraries")

    if(${CMAKE_VERSION} VERSION_LESS "3.11.0")
        target_include_directories(haljni PRIVATE ${JNI_INCLUDE_DIRS})
        target_include_directories(haljni PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/jniheaders")
    else()
        target_link_libraries(haljni PRIVATE hal_jni_headers)
    endif()
    add_dependencies(haljni hal_jar)

    if (MSVC)
        install(TARGETS haljni RUNTIME DESTINATION "${jni_lib_dest}" COMPONENT Runtime)
    endif()

    install(TARGETS haljni EXPORT haljni DESTINATION "${main_lib_dest}")

    file(GLOB hal_sim_native_src src/main/native/mau/simjni/*.cpp)

endif()

file(GLOB
    hal_shared_native_src src/main/native/cpp/cpp/*.cpp
    hal_shared_native_src src/main/native/cpp/handles/*.cpp
    hal_sim_native_src src/main/native/mau/*.cpp
    hal_sim_native_src src/main/native/mau/Translator/*.cpp
    hal_sim_native_src src/main/native/mau/DriveStation/*.cpp    
    hal_sim_native_src src/main/native/mau/ctre/*.cpp
    hal_sim_native_src src/main/native/mau/navX/src/*.cpp)
add_library(hal ${hal_shared_native_src} ${hal_shared_jni_src})

if(USE_EXTERNAL_HAL)
    include(${EXTERNAL_HAL_FILE})
else()
    target_sources(hal PRIVATE ${hal_sim_native_src})
endif()

configure_file(src/generate/FRCUsageReporting.h.in gen/hal/FRCUsageReporting.h)

set_target_properties(hal PROPERTIES OUTPUT_NAME "wpiHalMau")
set_target_properties(hal PROPERTIES DEBUG_POSTFIX "d")

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

# Specify link to VMXPI HAL Library
find_library(VMXPI_HAL_CPP vmxpi_hal_cpp PATHS /usr/local/lib/vmxpi)
target_link_libraries(hal PUBLIC wpiutil rt "${VMXPI_HAL_CPP}")

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
install(DIRECTORY src/main/native/mau/navX/include/ DESTINATION "${include_dest}/hal")

if (NOT WITHOUT_JAVA AND MSVC)
    install(TARGETS hal RUNTIME DESTINATION "${jni_lib_dest}" COMPONENT Runtime)
endif()

if (MSVC)
    set (hal_config_dir ${wpilib_dest})
else()
    set (hal_config_dir share/hal)
endif()

# Add Phoenix platform library to built items
add_subdirectory(src/main/native/mau/phoenix)

install(FILES hal-config.cmake DESTINATION ${hal_config_dir})
install(EXPORT hal DESTINATION ${hal_config_dir})

# Copy gradle distribution build files to build directory
# These can be used after compilation/linkage to construct maven repo artifacts
file(COPY gradle DESTINATION .)
file(COPY gradle/gradlew DESTINATION .)
file(COPY gradle/gradlew.bat DESTINATION .)
# Copy distribution scripts files to build directory
# These can be used to distribute maven repo artifacts
file(COPY dist/upload_to_kauailabs_maven_repo.sh DESTINATION .)

# Todo:  Acquire the package version dynamically
SET(dist_packageVersion '2019.4.1')
SET(dist_packageNamespace 'com.kauailabs.first.hal')
SET(dist_packageArtifactId 'hal-cpp')

# Set dist variables used by generated build.gradle file
# NOTE:  dist_packageType must not be enclosed in single or double-quotes

if (BUILD_SHARED_LIBS)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(dist_packageType packageSharedDebugDistribution)
	SET(dist_packageClassifier 'linuxraspbiandebug')
    else()
	SET(dist_packageType packageSharedReleaseDistribution)
	SET(dist_packageClassifier 'linuxraspbian')
    endif()
else()
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(dist_packageType packageStaticDebugDistribution)
	SET(dist_packageClassifier 'linuxraspbianstaticdebug')
    else()
	SET(dist_packageType packageStaticReleaseDistribution)
	SET(dist_packageClassifier 'linuxraspbianstatic')
    endif()
endif()

configure_file(gradle/build.gradle.in build.gradle)
