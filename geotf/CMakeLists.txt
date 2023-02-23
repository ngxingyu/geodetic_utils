cmake_minimum_required(VERSION 3.10)
project(geotf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(dependencies)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(GDAL REQUIRED)


ament_auto_add_library(${PROJECT_NAME} src/geodetic_converter.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} gdal yaml-cpp)

add_executable(demo_node src/demo_node.cc)
target_link_libraries(demo_node ${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(TARGETS
  demo_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

# if(AMENT_ENABLE_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(geo_tf_test test/test_axis_swaps.cc)
    target_link_libraries(geo_tf_test ${PROJECT_NAME})
# endif()
