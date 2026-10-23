cmake_minimum_required(VERSION 3.14)
project(uav_path_planning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(uav_planner src/main.cpp)
target_include_directories(uav_planner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(uav_planner PRIVATE -Wall -Wextra -Wpedantic -O2)
endif()

# Copy sample map next to binary for convenience when run from build/
add_custom_command(TARGET uav_planner POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:uav_planner>/configs
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_SOURCE_DIR}/configs/sample_map.json
          $<TARGET_FILE_DIR:uav_planner>/configs/sample_map.json
)

enable_testing()
add_test(NAME demo COMMAND uav_planner --demo
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
