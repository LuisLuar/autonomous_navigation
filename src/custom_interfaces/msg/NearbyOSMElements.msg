# NearbyOSMElements.msg - Versión simplificada
std_msgs/Header header

# Lista de elementos OSM cercanos
OSMElement[] nearby_elements

# Información contextual básica
int32 current_projection_idx    # Índice de proyección actual en OSM
float64 current_lateral_offset  # Offset lateral actual
bool lane_level_localization    # True si estamos en modo lane-level