from folium import Element
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                               QPushButton, QMessageBox)
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtCore import QUrl, Qt
import tempfile
import os
from PySide6.QtCore import Qt, QTimer

from utils.styles_light import get_app_style, get_map_widget_style, get_theme_colors

import math

class MapWidget(QWidget):
    def __init__(self, ros_node=None, offline_tiles_path=None, leaflet_path=None):
        super().__init__()
        self.ros_node = ros_node
        self.offline_tiles_path = offline_tiles_path or "/home/raynel/Documents/offline_title/GoogleImagenes" #OpenStreetMap GoogleImagenes
        
        # Ruta a los recursos de Leaflet offline
        if leaflet_path:
            self.leaflet_path = leaflet_path
        else:
            # Intentar encontrar la ruta automáticamente
            possible_paths = [
                os.path.expanduser("~/autonomous_navigation/src/system_management/gui/leaflet_offline"),
                os.path.join(os.path.dirname(__file__), "leaflet_offline"),
                "/home/raynel/autonomous_navigation/src/system_management/gui/leaflet_offline"
            ]
            self.leaflet_path = None
            for path in possible_paths:
                if os.path.exists(path):
                    self.leaflet_path = path
                    break
        
        self.current_position = None
        self.current_yaw = 0.0
        self.destination = None
        self.temp_destination = None  # Destino temporal del click en mapa
        self.html_file = None
        self.setup_ui()
        self.setup_map()

        # Timer para actualizar UI desde ros_node
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_from_ros)
        self.update_timer.start(500)  # cada 500 ms

        # Path global
        self.current_path = None

    
    def setup_ui(self):
        self.setStyleSheet(get_app_style() + get_map_widget_style())

        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        
        # Barra de controles del mapa
        controls_layout = QHBoxLayout()
        
        # Etiqueta de información
        self.info_label = QLabel("Haga clic en el mapa para seleccionar un destino")
        self.info_label.setObjectName("info_label")  # Asignar un ID para el selector CSS
        
        # Botón para establecer destino
        self.set_dest_btn = QPushButton("🎯 Establecer Destino")
        self.info_label.setObjectName("info_label")  # Asignar un ID para el selector CSS
        
        # Botón para establecer destino
        self.set_dest_btn = QPushButton("🎯 Establecer Destino")
        self.set_dest_btn.setObjectName("set_dest_btn")
        self.set_dest_btn.setEnabled(False)
        self.set_dest_btn.clicked.connect(self.confirm_destination)
        
        # Botón para limpiar destino
        self.clear_dest_btn = QPushButton("🗑️ Limpiar Destino")
        self.clear_dest_btn.setObjectName("clear_dest_btn")
        self.clear_dest_btn.clicked.connect(self.clear_destination)
        
        controls_layout.addWidget(self.info_label)
        controls_layout.addStretch()
        controls_layout.addWidget(self.set_dest_btn)
        controls_layout.addWidget(self.clear_dest_btn)
        
        layout.addLayout(controls_layout)
        
        # Widget del mapa
        self.map_view = QWebEngineView()
        
        # IMPORTANTE: Configurar permisos para acceso a archivos locales
        self.map_view.settings().setAttribute(
            self.map_view.settings().WebAttribute.LocalContentCanAccessFileUrls, True
        )
        self.map_view.settings().setAttribute(
            self.map_view.settings().WebAttribute.LocalContentCanAccessRemoteUrls, True
        )
        
        layout.addWidget(self.map_view)
        
    
    def create_offline_map_html(self):
        """Crear HTML del mapa con Leaflet offline"""
        initial_lat = -0.317604
        initial_lng = -78.444711
        
        # Determinar rutas de recursos
        if self.leaflet_path and os.path.exists(self.leaflet_path):
            leaflet_css = f"file://{self.leaflet_path}/leaflet.css"
            leaflet_js = f"file://{self.leaflet_path}/leaflet.js"
            #print(f"[MapWidget] ✓ Usando Leaflet OFFLINE desde: {self.leaflet_path}")
        else:
            # Fallback a CDN
            leaflet_css = "https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
            leaflet_js = "https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
            #print(f"[MapWidget] ⚠️ Leaflet offline no encontrado, usando CDN")
        
        tiles_url = f"file://{self.offline_tiles_path}/{{z}}/{{x}}/{{y}}.png"
        
        # Preparar rutas de iconos (offline o fallback)
        if self.leaflet_path and os.path.exists(self.leaflet_path):
            icon_car = f"file://{self.leaflet_path}/images/car-icon.png"
            icon_red = f"file://{self.leaflet_path}/images/marker-icon-red.png"
            icon_green = f"file://{self.leaflet_path}/images/marker-icon-green.png"
            icon_shadow = f"file://{self.leaflet_path}/images/marker-shadow.png"
        else:
            icon_car = "https://cdn-icons-png.flaticon.com/512/744/744465.png"  # ← ICONO DE CARRO ONLINE
            icon_red = "https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-red.png"
            icon_green = "https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-green.png"
            icon_shadow = "https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png"
        
        # Preparar marcadores
        robot_lat, robot_lng = self.current_position if self.current_position else (initial_lat, initial_lng)
        
        destination_marker_html = ""
        if self.destination:
            dest_lat, dest_lng = self.destination
            destination_marker_html = f"""
            var destinationMarker = L.marker([{dest_lat}, {dest_lng}], {{
                icon: L.icon({{
                    iconUrl: '{icon_red}',
                    shadowUrl: '{icon_shadow}',
                    iconSize: [25, 41],
                    iconAnchor: [12, 41],
                    popupAnchor: [1, -34],
                    shadowSize: [41, 41]
                }})
            }}).addTo(map);
            destinationMarker.bindPopup("Destino<br>Lat: {dest_lat:.6f}<br>Lng: {dest_lng:.6f}");
            """
        
        html_content = f"""
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Mapa Offline</title>
    <link rel="stylesheet" href="{leaflet_css}" />
    <script src="{leaflet_js}"></script>
    <script src="file://{self.leaflet_path}/leaflet.rotatedMarker.js"></script>
    <style>
        body {{
            margin: 0;
            padding: 0;
        }}
        #map {{
            width: 100%;
            height: 100vh;
        }}
    </style>
</head>
<body>
    <div id="map"></div>
        

    <script>

        // ============================
        // RUTA GLOBAL
        // ============================
        window.globalPathLayer = null;

        function drawGlobalPath(pathCoords) {{
            if (window.globalPathLayer) {{
                map.removeLayer(window.globalPathLayer);
            }}

            window.globalPathLayer = L.polyline(pathCoords, {{
                color: 'blue',
                weight: 5,
                opacity: 0.9
            }}).addTo(map);
        }}

        // Crear mapa
        var map = L.map('map').setView([{robot_lat}, {robot_lng}], 17);
        
        // Añadir capa de tiles offline
        L.tileLayer('{tiles_url}', {{
            attribution: 'Offline Map',
            maxZoom: 20,
            minZoom: 1
        }}).addTo(map);
        

        // Marcador del robot
        var robotMarker = L.marker([{robot_lat}, {robot_lng}], {{
            icon: L.icon({{
                iconUrl: '{icon_car}',
                shadowUrl: '{icon_shadow}',
                iconSize: [41, 25],
                iconAnchor: [15, 15],
                popupAnchor: [0, -15],
                shadowSize: [41, 41]
            }}),
            rotationAngle: {self.current_yaw},  // ← AGREGAR ROTACIÓN
            rotationOrigin: 'center'    // ← ORIGEN DE ROTACIÓN
        }}).addTo(map);

        window.robotMarker = robotMarker;
        robotMarker.bindPopup("Robot<br>Lat: {robot_lat:.6f}<br>Lng: {robot_lng:.6f}<br>Yaw: {self.current_yaw:.1f}°");
        
        // Marcador de destino (si existe)
        {destination_marker_html}
        
        // Control de escala
        L.control.scale().addTo(map);
        
        // Manejador de clicks
        map.on('click', function(e) {{
            var lat = e.latlng.lat;
            var lng = e.latlng.lng;
            console.log("Click en mapa: " + lat + ", " + lng);
            
            // Guardar coordenadas en variable global para Python
            window.selectedLat = lat;
            window.selectedLng = lng;
            
            // Remover marcador anterior si existe
            if (window.tempDestMarker) {{
                map.removeLayer(window.tempDestMarker);
            }}
            
            // Crear nuevo marcador temporal (naranja para indicar "pendiente")
            window.tempDestMarker = L.marker([lat, lng], {{
                icon: L.icon({{
                    iconUrl: '{icon_red}',
                    shadowUrl: '{icon_shadow}',
                    iconSize: [25, 41],
                    iconAnchor: [12, 41],
                    popupAnchor: [1, -34],
                    shadowSize: [41, 41]
                }})
            }}).addTo(map);
            window.tempDestMarker.bindPopup("Destino Temporal<br>Lat: " + lat.toFixed(6) + "<br>Lng: " + lng.toFixed(6) + "<br><i>Presione 'Establecer Destino'</i>").openPopup();
            
            // Notificar a Python que hay un nuevo punto seleccionado
            document.title = "MAP_CLICK:" + lat + "," + lng;
        }});
        
        console.log("Mapa inicializado correctamente");
    </script>
</body>
</html>
        """
        
        return html_content
    
    def setup_map(self):
        # Coordenadas iniciales
        initial_lat = -0.317604
        initial_lng = -78.444711
        
        # Verificar directorios
        if not os.path.exists(self.offline_tiles_path):
            #print(f"[MapWidget] ADVERTENCIA: No se encuentra el directorio de tiles: {self.offline_tiles_path}")
            pass
        
        # Crear HTML del mapa
        html_content = self.create_offline_map_html()
        
        # Guardar y cargar
        self.save_and_load_map(html_content)
        
        # Conectar señal para detectar cambios en el título (comunicación JS->Python)
        self.map_view.titleChanged.connect(self.on_map_title_changed)
        
        #print(f"[MapWidget] Mapa configurado en: {initial_lat:.6f}, {initial_lng:.6f}")
    
    def on_map_title_changed(self, title):
        """Detectar clicks en el mapa desde JavaScript"""
        if title.startswith("MAP_CLICK:"):
            coords = title.replace("MAP_CLICK:", "")
            try:
                lat, lng = map(float, coords.split(","))
                self.temp_destination = (lat, lng)
                
                # Actualizar UI
                self.info_label.setText(f"📍 Punto seleccionado: {lat:.6f}, {lng:.6f}")
                self.set_dest_btn.setEnabled(True)
                
                #print(f"[MapWidget] Punto temporal seleccionado: {lat:.6f}, {lng:.6f}")
            except Exception as e:
                #print(f"[MapWidget] Error procesando click: {e}")
                pass
    
    def save_and_load_map(self, html_content=None):
        # Limpiar archivo anterior
        if self.html_file and os.path.exists(self.html_file):
            try:
                os.unlink(self.html_file)
            except:
                pass
        
        # Crear archivo temporal para el mapa
        with tempfile.NamedTemporaryFile(suffix='.html', delete=False, mode='w', encoding='utf-8') as f:
            self.html_file = f.name
            if html_content:
                f.write(html_content)
            else:
                html_content = self.create_offline_map_html()
                f.write(html_content)
        
        # Cargar el mapa en el widget web
        url = QUrl.fromLocalFile(self.html_file)
        self.map_view.setUrl(url)
        
        #print(f"[MapWidget] Mapa HTML guardado en: {self.html_file}")
    
    def update_global_path(self, path_latlon):
        """
        Dibuja el path recibido desde ROS en Leaflet
        """
        if not path_latlon or len(path_latlon) < 2:
            return

        js_array = "[" + ",".join(
            f"[{lat},{lon}]" for lat, lon in path_latlon
        ) + "]"

        js_code = f"""
        if (typeof drawGlobalPath === 'function') {{
            drawGlobalPath({js_array});
        }}
        """

        self.map_view.page().runJavaScript(js_code)


    def confirm_destination(self):
        """Confirmar y establecer el destino seleccionado en el mapa"""
        if not self.temp_destination:
            QMessageBox.warning(self, "Error", "No hay ningún punto seleccionado en el mapa")
            return
        
        lat, lng = self.temp_destination
        
        # Guardar como destino confirmado
        self.destination = self.temp_destination
        self.temp_destination = None
        
        # Recrear el mapa con el destino confirmado
        self.setup_map()
        
        # Actualizar interfaz
        self.info_label.setText(f"✓ Destino establecido: {lat:.6f}, {lng:.6f}")
        self.set_dest_btn.setEnabled(False)

        # Imprimir en terminal (monitor serial)
        """print(f"\n{'='*60}")
        print(f"DESTINO ESTABLECIDO")
        print(f"{'='*60}")
        print(f"  Latitud:  {lat:.6f}")
        print(f"  Longitud: {lng:.6f}")
        print(f"{'='*60}\n")"""
        
        # Publicar a ROS si está disponible
        if self.ros_node:
            try:                
                # OPCIÓN B: Nuevo método para lat/lon (RECOMENDADO)
                if hasattr(self.ros_node.ros_bridge, 'publish_goal_latlon'):
                    self.ros_node.ros_bridge.publish_goal_latlon(lat, lng)
                else:
                    # Fallback al método antiguo
                    self.ros_node.publish_goal(lat, lng)
                    
                #print(f"[MapWidget] Goal lat/lon publicado: {lat:.6f}, {lng:.6f}")
            except Exception as e:
                #print(f"[MapWidget] Error publicando goal: {e}")
                pass
    
    def set_destination(self, lat, lng):
        """Establecer destino directamente (usado por update_robot_position si es necesario)"""
        self.destination = (lat, lng)
        self.setup_map()
        
        self.info_label.setText(f"✓ Destino: {lat:.6f}, {lng:.6f}")
        
        #print(f"[MapWidget] DESTINO_ESTABLECIDO: {lat:.6f}, {lng:.6f}")

        if self.ros_node:
            try:
                self.ros_node.publish_goal(lat, lng)
            except Exception as e:
                #print(f"[MapWidget] Error publicando goal: {e}")
                pass
    
    def clear_destination(self):
        self.destination = None
        self.temp_destination = None
        
        self.info_label.setText("Haga clic en el mapa para seleccionar un destino")
        self.set_dest_btn.setEnabled(False)

        # Recrear mapa sin destino
        self.setup_map()

        # Publicar a ROS si está disponible
        if self.ros_node:
            try:                
                # OPCIÓN B: Nuevo método para lat/lon (RECOMENDADO)
                if hasattr(self.ros_node.ros_bridge, 'clear_path'):
                    self.ros_node.ros_bridge.clear_path()
                    
                #print(f"[MapWidget] Goal lat/lon publicado: {lat:.6f}, {lng:.6f}")
            except Exception as e:
                #print(f"[MapWidget] Error publicando goal: {e}")
                pass
        
        #print(f"\n{'='*60}")
        #print(f"🗑️ DESTINO LIMPIADO")
        #print(f"{'='*60}\n")
    
    def handle_map_click(self, lat, lng):
        """Manejador alternativo para clicks en el mapa"""
        self.temp_destination = (lat, lng)
        self.info_label.setText(f"Punto seleccionado: {lat:.6f}, {lng:.6f}")
        self.set_dest_btn.setEnabled(True)
    
    def update_robot_position(self, lat, lng):
        """Actualizar posición actual del robot en el mapa"""
        self.current_position = (lat, lng)
        
        # Recrear mapa con nueva posición
        self.setup_map()
        
        #print(f"[MapWidget] Posición del robot actualizada: {lat:.6f}, {lng:.6f}")
    
    def update_map_position(self, lat, lng, yaw):
        """Actualiza solo la posición del robot sin recrear todo el mapa"""
        # Crear JavaScript para mover el marcador
        js_code = f"""
        if (window.robotMarker) {{
            var newLatLng = L.latLng({lat}, {lng});
            window.robotMarker.setRotationAngle({yaw});
            window.robotMarker.setLatLng(newLatLng);
            window.robotMarker.getPopup().setContent("Robot<br>Lat: {lat:.6f}<br>Lng: {lng:.6f}<br>Yaw: {yaw:.1f}°");
            
            // Opcional: centrar el mapa en la nueva posición
            // map.panTo(newLatLng);
        }}
        """
        self.map_view.page().runJavaScript(js_code)

    # ==============================================================
    # ACTUALIZACIÓN DESDE ROS2
    # ==============================================================
    def update_from_ros(self):
        """Actualiza los datos desde ROS2 - VERSIÓN OPTIMIZADA."""
        if not self.ros_node:
            return

        rn = self.ros_node

        try:
            # Verificar el bridge
            if not hasattr(rn, 'ros_bridge') or rn.ros_bridge is None:
                self._show_no_bridge_error()
                return

            bridge = rn.ros_bridge

            if bridge.odom_local:
                orientation = bridge.odom_local.pose.pose.orientation
                yaw_rad = math.atan2(2.0*(orientation.w*orientation.z + orientation.x*orientation.y),
                                    1.0 - 2.0*(orientation.y*orientation.y + orientation.z*orientation.z))
                yaw_deg = math.degrees(yaw_rad)

                # Ajustar según REP-105: X hacia el Este = 0°
                # En Leaflet 0° = Norte, REP-105 0° = Este, así que sumamos 90°
                yaw_deg = (-1*yaw_deg ) % 360
                if yaw_deg < 0:
                    yaw_deg += 360
                
                if (abs(yaw_deg - self.current_yaw) > 5.0):  # Solo actualizar si hay un cambio significativo
                    
                    self.current_yaw =  yaw_deg  # ← ACTUALIZAR VARIABLE
                #self.get_logger().info(f"[MapWidget] Yaw actualizado: {self.current_yaw:.1f}°")
                    self.update_map_position(self.current_position[0], self.current_position[1], yaw_deg)

            if bridge.current_latlon:
                lat, lng = bridge.current_latlon

                if (not self.current_position or 
                    abs(lat - self.current_position[0]) > 1e-7 or 
                    abs(lng - self.current_position[1]) > 1e-7):

                    self.current_position = (lat, lng)
                    self.update_map_position(lat, lng, yaw_deg)


            # Actualizar path global si hay uno nuevo
            if bridge.global_path:
                if self.current_path != bridge.global_path:
                    self.current_path = bridge.global_path
                    self.update_global_path(self.current_path)
                                


        except AttributeError as e:
            #print(f"Error accediendo a datos ROS: {e}")
            pass
        except Exception as e:
            #print(f"Error inesperado en update_from_ros: {e}")
            pass

    def _show_no_bridge_error(self):
        """Muestra un mensaje de error cuando no hay bridge disponible."""
        #print("No hay bridge ROS disponible")
        pass
    
    def closeEvent(self, event):
        # Limpiar archivo temporal al cerrar
        if self.html_file and os.path.exists(self.html_file):
            try:
                os.unlink(self.html_file)
                #print(f"[MapWidget] Archivo temporal eliminado: {self.html_file}")
            except Exception as e:
                #print(f"[MapWidget] No se pudo eliminar archivo temporal: {e}")
                pass
        super().closeEvent(event)