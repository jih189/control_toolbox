cmake_minimum_required(VERSION 3.16)
project(RemoteAPIClientExample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the external library
add_subdirectory(third/zmqRemoteApi/clients/cpp)

# Add your executable
add_executable(my_client src/example.cpp)

# Set include directories (correct target!)
target_include_directories(my_client PRIVATE
    third/zmqRemoteApi/clients/cpp
)

# Link against their provided target
target_link_libraries(my_client PRIVATE RemoteAPIClient)