cmake_minimum_required(VERSION 3.8)
project(camera_cpp_nodes)

# 1. Estándar C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. Rutas de LibTorch (Prioridad)
list(APPEND CMAKE_PREFIX_PATH "/home/raynel/libtorch_setup/libtorch")

# 3. Buscar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui)
find_package(Torch REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
# nlohmann_json suele encontrarse vía simple inclusión o find_package según la distro
find_package(nlohmann_json REQUIRED) 

find_package(Boost REQUIRED COMPONENTS system)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# 4. Ejecutables
add_executable(camera_node src/camera/camera.cpp)
add_executable(segmenter_node src/camera/yolop.cpp)
add_executable(extractor_node src/camera/candidate_extractor.cpp)
add_executable(ipm_node src/camera/pixel_to_meter.cpp)
add_executable(esp32_control src/esp32.cpp)

# 5. Dependencias de ROS y Externas (ament_target_dependencies)
# Esto maneja rclcpp, sensor_msgs, etc.
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

ament_target_dependencies(segmenter_node
  rclcpp
  sensor_msgs
  cv_bridge
  custom_interfaces
  OpenCV
)

ament_target_dependencies(extractor_node
  rclcpp
  sensor_msgs
  cv_bridge
  custom_interfaces
  OpenCV
)

ament_target_dependencies(esp32_control
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_geometry_msgs
  Boost
)

# El nodo de IPM necesita Eigen y JSON
ament_target_dependencies(ipm_node
  rclcpp
  sensor_msgs
  custom_interfaces
)

# 6. Configuración específica para el Segmentador (LibTorch + CUDA)
target_link_directories(segmenter_node PRIVATE "/home/raynel/libtorch_setup/libtorch/lib")

target_link_libraries(segmenter_node
  ${OpenCV_LIBRARIES}
  "-Wl,--no-as-needed"
  "/home/raynel/libtorch_setup/libtorch/lib/libcudart-d0da41ae.so.11.0"
  ${TORCH_LIBRARIES}
  "torch_cuda"
  "c10_cuda"
  "-Wl,--as-needed"
)

# 7. Configuración específica para IPM (Eigen y JSON)
target_include_directories(ipm_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(ipm_node nlohmann_json::nlohmann_json)

# 8. Configuración para Extractor (OpenCV)
target_link_libraries(extractor_node
  ${OpenCV_LIBRARIES}
)

# 9. Configuración para Camera Node
target_link_libraries(camera_node
  ${OpenCV_LIBRARIES}
)

# 10. Flags y definiciones
target_compile_definitions(segmenter_node PRIVATE ${TORCH_CXX_FLAGS})

# 11. Instalación de todos los ejecutables
install(TARGETS
  camera_node
  segmenter_node
  extractor_node
  ipm_node
  esp32_control
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar los archivos launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()