cmake_minimum_required(VERSION 3.8)
project(irob_ros)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IROBController.msg"
)

add_executable(iROB_ESP src/iROB_ESP.c)
add_dependencies(iROB_ESP ${PROJECT_NAME}__rosidl_generator_c)
ament_target_dependencies(iROB_ESP rcl)

rosidl_get_typesupport_target(c_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_c")
target_link_libraries(iROB_ESP "${c_typesupport_target}")

install(TARGETS iROB_ESP
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY arduino
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
