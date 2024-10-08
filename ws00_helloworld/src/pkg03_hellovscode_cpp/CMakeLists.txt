cmake_minimum_required(VERSION 3.8)
project(pkg03_hellovscode_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(hellovscode src/hellovscode.cpp)
add_executable(hellovscode2 src/hellovscode2.cpp)

target_include_directories(hellovscode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hellovscode PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  hellovscode
  "rclcpp"
)
ament_target_dependencies(
  hellovscode2
  "rclcpp"
)

install(TARGETS hellovscode
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS hellovscode2
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
