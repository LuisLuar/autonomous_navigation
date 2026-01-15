# Hipótesis de carril completo
nav_msgs/Path[] lanes          # Todos los carriles detectados (centros)
int32 ego_lane_index           # Índice del carril propio
float32[] lane_widths          # Anchos estimados de cada carril
bool[] has_marking             # Si tiene marcación visible