# LaneLine.msg - Información de una línea detectada
geometry_msgs/Point[] points     # Puntos que forman la línea
float32 mean_rel_y               # Posición Y promedio en base_footprint
float32 quality                  # Calidad (0.0-1.0)
float32 angle                    # Ángulo en radianes
float32 length                   # Longitud en metros
float32 perp_std                 # Desviación estándar perpendicular
int32 n_points                   # Número de puntos
float32 x_mean                   # Posición X promedio