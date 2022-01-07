cmake_minimum_required(VERSION 3.8)
project(topic_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(${ament_cmake_python_VERSION} VERSION_GREATER_EQUAL 1.2.0)
  ament_python_install_package(${PROJECT_NAME}
    SETUP_CFG
      ${PROJECT_NAME}/setup.cfg
    SCRIPTS_DESTINATION
      lib/${PROJECT_NAME}
  )
else()
  ament_python_install_package(${PROJECT_NAME}
    SETUP_CFG
      ${PROJECT_NAME}/setup.cfg
  )

  set(build_dir "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${PROJECT_NAME}")

  file(MAKE_DIRECTORY "${build_dir}/scripts")  # setup.py may or may not create it

  add_custom_target(
    ament_cmake_python_build_${PROJECT_NAME}_scripts ALL
    COMMAND ${PYTHON_EXECUTABLE} setup.py install_scripts -d scripts
    WORKING_DIRECTORY "${build_dir}"
    DEPENDS ${egg_dependencies}
  )

  if(NOT AMENT_CMAKE_SYMLINK_INSTALL)
    # Not needed for nor supported by symlink installs
    set(_extra_install_args USE_SOURCE_PERMISSIONS)
  endif()

  install(
    DIRECTORY "${build_dir}/scripts/"
    DESTINATION "lib/${PROJECT_NAME}/"
    ${_extra_install_args}
  )
endif()

ament_auto_add_library(relay_node SHARED
  src/relay_node.cpp
  src/tool_base_node.cpp
)
target_compile_definitions(relay_node PRIVATE "TOPIC_TOOLS_BUILDING_LIBRARY")

rclcpp_components_register_nodes(relay_node "topic_tools::RelayNode")

ament_auto_add_executable(relay
  src/relay.cpp
)

target_link_libraries(relay
  relay_node
)

ament_auto_add_library(throttle_node SHARED
  src/throttle_node.cpp
  src/tool_base_node.cpp
)
target_compile_definitions(throttle_node PRIVATE "TOPIC_TOOLS_BUILDING_LIBRARY")

rclcpp_components_register_nodes(throttle_node "topic_tools::ThrottleNode")

ament_auto_add_executable(throttle
  src/throttle.cpp
)

target_link_libraries(throttle
  throttle_node
)

ament_auto_add_library(drop_node SHARED
  src/drop_node.cpp
  src/tool_base_node.cpp
)
target_compile_definitions(drop_node PRIVATE "TOPIC_TOOLS_BUILDING_LIBRARY")

rclcpp_components_register_nodes(drop_node "topic_tools::DropNode")

ament_auto_add_executable(drop
  src/drop.cpp
)

target_link_libraries(drop
  drop_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
