cmake_minimum_required(VERSION 3.8)
project(irob_ros)

# ใช้ภาษา C สำหรับ node iROB_ESP และ iROB_CMD
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# เปิด warning เพิ่มเมื่อใช้ compiler ตระกูล GNU/Clang
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependency หลักของ ROS2 C node และระบบ generate .msg
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate message สำหรับ feedback และ command ของ iROB
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IROBCommand.msg"
  "msg/IROBController.msg"
)

# node รับ feedback จาก ESP32 แล้ว publish /iROB_controller
add_executable(iROB_ESP src/iROB_ESP.c)
add_dependencies(iROB_ESP ${PROJECT_NAME}__rosidl_generator_c)
ament_target_dependencies(iROB_ESP rcl)

# node รับ /iROB_command แล้วส่ง command ไป ESP32
add_executable(iROB_CMD src/iROB_CMD.c)
add_dependencies(iROB_CMD ${PROJECT_NAME}__rosidl_generator_c)
ament_target_dependencies(iROB_CMD rcl rmw)

# link executable ทั้งสองตัวเข้ากับ type support ของ message ที่ generate เอง
rosidl_get_typesupport_target(c_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_c")
target_link_libraries(iROB_ESP "${c_typesupport_target}")
target_link_libraries(iROB_CMD "${c_typesupport_target}")

# ติดตั้ง executable ให้ ros2 run เรียกใช้ได้
install(TARGETS
  iROB_ESP
  iROB_CMD
  DESTINATION lib/${PROJECT_NAME}
)

# ติดตั้ง Arduino sketch ไปไว้ใน share ของ package เพื่อเก็บคู่กับ ROS2 package
install(DIRECTORY arduino
  DESTINATION share/${PROJECT_NAME}
)

# export runtime dependency สำหรับ package อื่นที่ import message ของ irob_ros
ament_export_dependencies(rosidl_default_runtime)
ament_package()
