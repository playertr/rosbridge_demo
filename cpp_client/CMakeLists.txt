cmake_minimum_required(VERSION 3.9)
project(cpp_client)

# # Need to set the CMake search directory due to open issue with deb
# # https://github.com/microsoft/cpprestsdk/issues/686
set(cpprestsdk_DIR /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/)
find_package(cpprestsdk REQUIRED)

# Build  and link this project
add_executable(client client.cpp)
target_link_libraries(client cpprestsdk::cpprest)