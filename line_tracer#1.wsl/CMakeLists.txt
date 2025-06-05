cmake_minimum_required(VERSION 3.16)
project(line_tracer)

# C++14 사용
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 컴파일 경고 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(wsl_node src/wsl.cpp)
ament_target_dependencies(wsl_node
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
  cv_bridge
)
target_link_libraries(wsl_node ${OpenCV_LIBRARIES})

# 설치 설정
install(TARGETS
  wsl_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
