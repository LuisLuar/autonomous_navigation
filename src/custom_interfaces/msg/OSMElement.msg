# OSMElement.msg - Versión simplificada para el nodo OSM
std_msgs/Header header

# Tipos de features
uint8 FEATURE_UNKNOWN = 0
uint8 FEATURE_TRAFFIC_CALMING = 1
uint8 FEATURE_CROSSING = 2
uint8 FEATURE_INTERSECTION = 3

# Información básica
uint8 feature_type
float64 distance  # metros a elemento
float64 latitude
float64 longitude
string osm_id
string feature_name
string sub_type   # "bump", "uncontrolled", "zebra", etc.
string[] tags

# Información relativa al lane_ego
float64 longitudinal_distance  # Distancia a lo largo del lane_ego (metros)
float64 lateral_offset         # Offset lateral desde lane_ego (metros)
float64 progress_along_path    # Porcentaje de progreso (0-100)
bool is_ahead                  # True si está adelante del robot