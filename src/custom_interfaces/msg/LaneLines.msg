# LaneLines.msg - Conjunto de líneas detectadas
std_msgs/Header header           # Timestamp y frame_id
LaneLine[] lines                 # Array de líneas
bool is_in_curve                 # Indica si está en curva
int32 frame_id                   # ID del frame