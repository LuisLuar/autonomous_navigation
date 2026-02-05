# custom_interfaces/msg/PixelPoint.msg
std_msgs/Header header
uint32[] x_coordinates    # Coordenadas X en imagen (píxeles)
uint32[] y_coordinates    # Coordenadas Y en imagen (píxeles)
float32[] confidences     # Confianza por punto (0-1)
float32 global_quality    # Calidad global de detección (0-1)
bool is_valid             # Si la detección es válida