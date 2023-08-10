cmake_minimum_required(VERSION 3.8)
project(linear_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Define path of libmodbus folder here.
find_path(LIBMODBUS_PATH
  NAMES "README.md"
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../libmodbus/"
)
message("Libmodbus path dir = ${LIBMODBUS_PATH}")
# Run autogen.sh if config.h.in does not exist.
find_path(CONFIGURE_IN_PATH
  NAMES "config.h.in"
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../libmodbus/"
)
if(NOT CONFIGURE_IN_PATH)
  execute_process(
    COMMAND "${LIBMODBUS_PATH}/autogen.sh"
    WORKING_DIRECTORY ${LIBMODBUS_PATH}
    RESULT_VARIABLE AUTOGEN_RESULT
    OUTPUT_VARIABLE AUTOGEN_OUTPUT
    ERROR_VARIABLE AUTOGEN_ERROR
  )
  if(AUTOGEN_RESULT)
    message("Libmodbus autogen RESULT = ${AUTOGEN_RESULT}")
  endif()
  if(AUTOGEN_OUTPUT)
    message("Libmodbus autogen OUTPUT = ${AUTOGEN_OUTPUT}")
  endif()
  if(AUTOGEN_ERROR)
    message("Libmodbus autogen ERROR = ${AUTOGEN_ERROR}")
  endif()
endif()

# Run configure if config.h does not exist.
find_path(CONFIGURE_H_PATH
  NAMES "config.h"
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../libmodbus/"
)
if(NOT CONFIGURE_H_PATH)
  execute_process(
    COMMAND ${LIBMODBUS_PATH}/configure
    WORKING_DIRECTORY ${LIBMODBUS_PATH}
    RESULT_VARIABLE CONFIGURE_RESULT
    OUTPUT_VARIABLE CONFIGURE_OUTPUT
    ERROR_VARIABLE CONFIGURE_ERROR
  )
  if(CONFIGURE_RESULT)
    message("Libmodbus configure RESULT = ${CONFIGURE_RESULT}")
  endif()
  if(CONFIGURE_OUTPUT)
    message("Libmodbus configure OUTPUT = ${CONFIGURE_OUTPUT}")
  endif()
  if(CONFIGURE_ERROR)
    message("Libmodbus configure ERROR = ${CONFIGURE_ERROR}")
  endif()
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(LIBMODBUS_CORE_FILES
  ${LIBMODBUS_PATH}/src/modbus-data.c
  ${LIBMODBUS_PATH}/src/modbus-rtu.c
  ${LIBMODBUS_PATH}/src/modbus-tcp.c
  ${LIBMODBUS_PATH}/src/modbus.c
)

#add_library(${PROJECT_NAME} SHARED
#  src/hiwin_libmodbus.cpp
#  ${LIBMODBUS_CORE_FILES}
#)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.


#install(
#  DIRECTORY include/
#  DESTINATION include
#)

#install(
#  TARGETS ${PROJECT_NAME}_ctype_wrapper
#  EXPORT export_${PROJECT_NAME}_ctype_wrapper
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)





#install(TARGETS 
  #hiwinmodbus_server
  #hiwinlibmodbus_server
  #hiwin_libmodbus_node
  #DESTINATION lib/${PROJECT_NAME})

#ament_python_install_package(${PROJECT_NAME})
#install(PROGRAMS
  #scripts/Hiwin_API_test.py
  #DESTINATION lib/${PROJECT_NAME}
#)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  linear_motion
)
#ament_export_targets(
#  export_${PROJECT_NAME}
#)

ament_package()