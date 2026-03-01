cmake_minimum_required(VERSION 3.8)
project(camera_cpp_nodes)

# 1. Estándar C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. Buscar dependencias de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui)
find_package(custom_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED) 
find_package(Boost REQUIRED COMPONENTS system)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# En la sección 3 del CMakeLists.txt
find_library(NVINFER_LIB nvinfer PATHS /usr/lib/x86_64-linux-gnu)
find_library(NVONNX_LIB nvonnxparser PATHS /usr/lib/x86_64-linux-gnu)
# Añade esta nueva librería requerida en TRT 10 para runtime
find_library(NVINFER_RUNTIME_LIB nvinfer_dispatch PATHS /usr/lib/x86_64-linux-gnu)

set(TENSORRT_LIBS ${NVINFER_LIB} ${NVONNX_LIB} ${NVINFER_RUNTIME_LIB})

# 4. Ejecutables
add_executable(camera_node src/camera/camera.cpp)
add_executable(segmenter_node src/camera/yolop.cpp)
add_executable(extractor_node src/camera/candidate_extractor.cpp)
add_executable(ipm_node src/camera/pixel_to_meter.cpp)
add_executable(esp32_control src/esp32.cpp)

# 5. Dependencias de ROS (ament_target_dependencies)
ament_target_dependencies(camera_node rclcpp sensor_msgs cv_bridge OpenCV)
ament_target_dependencies(extractor_node rclcpp sensor_msgs cv_bridge custom_interfaces OpenCV)
ament_target_dependencies(ipm_node rclcpp sensor_msgs custom_interfaces)
ament_target_dependencies(esp32_control rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs tf2 tf2_geometry_msgs Boost)

# Segmenter_node ya no depende de Torch
ament_target_dependencies(segmenter_node
  rclcpp
  sensor_msgs
  cv_bridge
  custom_interfaces
  OpenCV
)

# 6. CONFIGURACIÓN ESPECÍFICA PARA EL SEGMENTADOR (TensorRT + CUDA)
target_link_libraries(segmenter_node
  ${OpenCV_LIBRARIES}
  ${TENSORRT_LIBS}
  ${CUDA_LIBRARIES}
  cudart
)

# 7. Configuración para IPM (Eigen y JSON)
target_include_directories(ipm_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(ipm_node nlohmann_json::nlohmann_json)

# 8. Configuración para otros nodos
target_link_libraries(extractor_node ${OpenCV_LIBRARIES})
target_link_libraries(camera_node ${OpenCV_LIBRARIES})

# 9. Instalación
install(TARGETS
  camera_node
  segmenter_node
  extractor_node
  ipm_node
  esp32_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()