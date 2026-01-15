# ObjectInfo.msg
# Información completa para controlador

# Identificación
uint32 track_id
string class_name
uint32 class_id
float32 confidence

# Posición en imagen
uint32 x1
uint32 y1
uint32 x2
uint32 y2
float32 center_x
float32 center_y

# Posición 3D estimada (relativa al robot)
float32 distance          # distancia frontal (m)
float32 lateral_offset    # desplazamiento lateral (m, +derecha, -izquierda)
float32 ground_distance   # distancia real en plano (m)
bool distance_valid       # si la distancia es confiable

# Velocidades
float32 velocity_x        # velocidad en X (m/s, relativa al robot)
float32 velocity_y        # velocidad en Y (m/s, relativa al robot)
float32 speed             # velocidad total (m/s)
float32 relative_speed    # velocidad relativa al robot (m/s, +alejándose, -acercándose)

# Estados
bool is_static           # objeto estacionario (velocidad < umbral)
bool is_moving_toward    # se acerca al robot
bool is_moving_away      # se aleja del robot
bool is_in_path          # está en trayectoria del robot
float32 time_to_collision # TTC estimado (segundos)

# Calidad de track
uint32 track_age         # frames seguidos
bool is_lost             # track perdido temporalmente
float32 quality_score    # 0-1, confianza general

# Fuentes de información
uint8 distance_source    # 0=none, 1=depth_camera, 2=ipm_estimate, 3=size_based