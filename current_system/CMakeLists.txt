cmake_minimum_required(VERSION 3.10)
project(current_system)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(current_system SHARED src/current_system.cc)
target_link_libraries(current_system gz-sim8::gz-sim8)
target_compile_features(current_system PUBLIC cxx_std_17)
target_link_libraries(current_system gz-msgs10::gz-msgs10)

install(TARGETS current_system
  LIBRARY DESTINATION lib)

ament_package()
