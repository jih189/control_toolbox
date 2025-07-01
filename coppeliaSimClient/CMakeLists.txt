cmake_minimum_required(VERSION 3.16)
project(RemoteAPIClientExample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the external library
add_subdirectory(third/zmqRemoteApi/clients/cpp)

# Add your executable
add_executable(sim_timer src/sim_timer.cpp)

# Set include directories
target_include_directories(sim_timer PRIVATE
    third/zmqRemoteApi/clients/cpp
)

# Link against their provided target
target_link_libraries(sim_timer PRIVATE RemoteAPIClient)