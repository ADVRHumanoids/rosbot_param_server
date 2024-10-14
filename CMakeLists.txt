cmake_minimum_required(VERSION 3.0.2)
project(rosbot_param_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
)
find_package(yaml-cpp REQUIRED)


add_service_files(
    FILES
    SetString.srv
    GetParameterInfo.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rosbot_param_server
    CATKIN_DEPENDS message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(parameter_manager src/parameter_manager.cpp)
target_link_libraries(parameter_manager ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

install(TARGETS parameter_manager
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

add_subdirectory(bindings/python)
