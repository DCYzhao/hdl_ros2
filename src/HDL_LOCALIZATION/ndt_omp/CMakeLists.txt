cmake_minimum_required(VERSION 3.16)
project(ndt_omp)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 编译优化选项
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3 -DNDEBUG)
endif()

# SIMD指令编译选项
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
elseif(BUILD_WITH_MARCH_NATIVE)
  add_compile_options(-march=native)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common io filters registration kdtree segmentation visualization)
find_package(OpenMP REQUIRED)

# 打印调试信息
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")
message(STATUS "PCL_DEFINITIONS: ${PCL_DEFINITIONS}")
message(STATUS "OpenMP found: ${OpenMP_CXX_FOUND}")

# 创建主库
add_library(ndt_omp SHARED
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

# 设置目标属性
target_compile_features(ndt_omp PRIVATE cxx_std_14)

# 包含目录
target_include_directories(ndt_omp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(ndt_omp PUBLIC
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

# 添加可执行文件（如果需要）
add_executable(align apps/align.cpp)
ament_target_dependencies(align rclcpp)
target_link_libraries(align 
  ndt_omp
  ${PCL_LIBRARIES}
)
install(TARGETS align
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装规则
install(TARGETS ndt_omp
  EXPORT ndt_omp_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 导出依赖
ament_export_include_directories(include)
ament_export_libraries(ndt_omp)
ament_export_dependencies(rclcpp PCL OpenMP)

ament_package()