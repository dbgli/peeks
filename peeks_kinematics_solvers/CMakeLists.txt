# CMake版本要求
cmake_minimum_required(VERSION 3.8...3.28)
if(${CMAKE_VERSION} VERSION_LESS 3.12)
  cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

# 项目名
project(peeks_kinematics_solvers)

# C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 构建类型
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "CMAKE_BUILD_TYPE is Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
else()
  message(STATUS "CMAKE_BUILD_TYPE is Release")
  set(CMAKE_BUILD_TYPE "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

# 额外的警告信息
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

# ROS体系下的依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 非ROS体系下的依赖
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED NO_MODULE)

# 正向运动学求解器
add_executable(fk_solver src/fk_solver.cpp)
target_compile_features(fk_solver PUBLIC cxx_std_20)
target_include_directories(fk_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(fk_solver
  Boost::boost
  Boost::system
  Eigen3::Eigen)
ament_target_dependencies(fk_solver
  rclcpp)
install(TARGETS fk_solver
  DESTINATION lib/${PROJECT_NAME})

# 逆向运动学求解器
add_executable(ik_solver src/ik_solver.cpp)
target_compile_features(ik_solver PUBLIC cxx_std_20)
target_include_directories(ik_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(ik_solver
  Boost::boost
  Boost::system
  Eigen3::Eigen)
ament_target_dependencies(ik_solver
  rclcpp)
install(TARGETS ik_solver
  DESTINATION lib/${PROJECT_NAME})

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出Ament包
ament_package()