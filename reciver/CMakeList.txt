cmake_minimum_required(VERSION 3.18)
project(rpi-sx1262-reciver)

# If RadioLib is cloned next to this folder, for example:
#   parent/
#     ├── RadioLib/
#     └── reciver/
# then this relative path is correct:
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../RadioLib" "${CMAKE_CURRENT_BINARY_DIR}/RadioLib")

# Add your receiver source file
add_executable(${PROJECT_NAME} reciver.cpp)

# Link with RadioLib and lgpio
target_link_libraries(${PROJECT_NAME} RadioLib lgpio)

# Optional debug output (uncomment to enable verbose SPI/debug logs)
# target_compile_definitions(RadioLib PUBLIC RADIOLIB_DEBUG_BASIC RADIOLIB_DEBUG_PORT=stdout)
