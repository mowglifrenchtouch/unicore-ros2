cmake_minimum_required(VERSION 3.8)
project(unicore_gnss)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(compass_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)

add_library(${PROJECT_NAME}_core STATIC
  src/serial_port.cpp
  src/unicore_binary_nav.cpp
  src/unicore_transport.cpp
  src/unicore_parser.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

set(UNICORE_NODE_TARGETS
  unicore_node
  um982_node
)

foreach(node_target IN LISTS UNICORE_NODE_TARGETS)
  add_executable(${node_target}
    src/unicore_node.cpp
  )

  target_link_libraries(${node_target}
    ${PROJECT_NAME}_core
  )

  ament_target_dependencies(${node_target}
    rclcpp
    sensor_msgs
    compass_msgs
    diagnostic_msgs
    rtcm_msgs
  )
endforeach()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  TARGETS ${PROJECT_NAME}_core ${UNICORE_NODE_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_unicore_parser
    test/test_unicore_parser.cpp
  )
  target_link_libraries(test_unicore_parser
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_diagnostic_state
    test/test_diagnostic_state.cpp
  )

  ament_add_gtest(test_unicore_transport
    test/test_unicore_transport.cpp
  )
  target_link_libraries(test_unicore_transport
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_unicore_binary_nav
    test/test_unicore_binary_nav.cpp
  )
  target_link_libraries(test_unicore_binary_nav
    ${PROJECT_NAME}_core
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()
