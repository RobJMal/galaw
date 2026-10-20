cmake_minimum_required(VERSION 3.16)
project(galaw_cpp_bench CXX)

# Export compile commands so VS Code IntelliSense can find include paths.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Match the robotpkg build flags exactly: -O3 -DNDEBUG, no -march=native.
# This ensures a fair comparison with the prebuilt libpinocchio_default.so.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")

# --- Find Pinocchio ---
# robotpkg installs to /opt/openrobots; accept that or any CMAKE_PREFIX_PATH.
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")
find_package(pinocchio REQUIRED)

# --- FK benchmark ---
add_executable(fk_bench fk_bench.cpp)
target_link_libraries(fk_bench PRIVATE pinocchio::pinocchio)
target_include_directories(fk_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# --- Jacobian benchmark ---
add_executable(jacobian_bench jacobian_bench.cpp)
target_link_libraries(jacobian_bench PRIVATE pinocchio::pinocchio)
target_include_directories(jacobian_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
