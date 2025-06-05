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

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# 헤더 include
include_directories(
  include
  /usr/local/include/dynamixel_sdk
  ${OpenCV_INCLUDE_DIRS}
)

# Dynamixel 라이브러리 연결
link_directories(/usr/local/lib)

# Jetson 노드
add_executable(jetson_node src/jetson.cpp src/dxl.cpp)
ament_target_dependencies(jetson_node rclcpp sensor_msgs std_msgs OpenCV cv_bridge)
target_link_libraries(jetson_node dxl_x64_cpp ${OpenCV_LIBRARIES})

# 설치
install(TARGETS
  jetson_node
  DESTINATION lib/${PROJECT_NAME})

# 패키지 선언
ament_package()
