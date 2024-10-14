cmake_minimum_required(VERSION 3.15)

#find_package(Python3 REQUIRED COMPONENTS Development Interpreter)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

message(STATUS "**********************************Check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "+++++++++++++++++++++++++++++++Compiling python bindings (${pybind11_LIBRARIES})")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    pybind11_add_module(rosbot_param_server_py rosbot_param_server_py.cpp)
    target_link_libraries(rosbot_param_server_py PRIVATE parameter_manager ${catkin_LIBRARIES})
    install(TARGETS rosbot_param_server_py
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

else()
    message(STATUS "-----------------------------------pybind not found")
endif()

