cmake_minimum_required(VERSION 3.8)
project(final_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 필수 라이브러리 찾기
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. 서비스 메시지(.srv) 빌드
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetColor.srv"
)

# 3. 파이썬 스크립트 설치
install(PROGRAMS
  scripts/led_service.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()