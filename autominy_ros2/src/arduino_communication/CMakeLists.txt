cmake_minimum_required(VERSION 3.22)
project(arduino_communication)

# Mantén C++14 si tu código lo requiere, o sube a C++17 si necesitas características nuevas:
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Asegúrate de encontrar ament_cmake:
find_package(ament_cmake REQUIRED)

find_package(rclcpp           REQUIRED)
find_package(autominy_msgs    REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(sensor_msgs      REQUIRED)
find_package(std_srvs         REQUIRED)
find_package(serial           REQUIRED)
find_package(Boost            REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
)

# Ejecutable principal: agrupa ahí todos los .cpp y .h que necesites
add_executable(arduino_communication_node
  src/ArduinoCommunication.cpp
  src/ArduinoCommunicationNode.cpp
  src/MessageType.h
  src/COBS.h
)

# Linkeos
target_link_libraries(arduino_communication_node
  ${Boost_LIBRARIES}
)

ament_target_dependencies(arduino_communication_node
  rclcpp
  autominy_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  serial
)

# Instalación del ejecutable
install(TARGETS arduino_communication_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Instalación del launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(
  rclcpp
  autominy_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  serial
)

ament_package()

