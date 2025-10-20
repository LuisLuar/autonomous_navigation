import folium
import sys
import random
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                               QPushButton, QLineEdit, QMessageBox)
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtCore import QUrl, Qt
import tempfile
import os

from config.constants import Constants

class MapWidget(QWidget):
    def __init__(self, ros_node=None):
        super().__init__()
        self.ros_node = ros_node
        self.map = None
        self.current_position = None
        self.destination = None
        self.html_file = None
        self.setup_ui()
        self.setup_map()
    
    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        
        # Barra de controles del mapa
        controls_layout = QHBoxLayout()
        
        # Coordenadas actuales
        self.coord_label = QLabel("Coordenadas: No establecidas")
        self.coord_label.setStyleSheet("color: #ecf0f1; font-weight: bold;")
        
        # Entrada para coordenadas manuales
        self.coord_input = QLineEdit()
        self.coord_input.setPlaceholderText("Lat, Lng ej: -17.783, -63.182")
        self.coord_input.setStyleSheet("""
            QLineEdit {
                background-color: #2c3e50;
                color: #ecf0f1;
                border: 1px solid #3498db;
                border-radius: 3px;
                padding: 5px;
            }
        """)
        
        # Botón para establecer destino
        self.set_dest_btn = QPushButton("🎯 Establecer Destino")
        self.set_dest_btn.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #229954;
            }
        """)
        self.set_dest_btn.clicked.connect(self.set_destination_from_input)
        
        # Botón para limpiar destino
        self.clear_dest_btn = QPushButton("🗑️ Limpiar Destino")
        self.clear_dest_btn.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
        """)
        self.clear_dest_btn.clicked.connect(self.clear_destination)
        
        controls_layout.addWidget(self.coord_label)
        controls_layout.addWidget(QLabel("Destino:"))
        controls_layout.addWidget(self.coord_input)
        controls_layout.addWidget(self.set_dest_btn)
        controls_layout.addWidget(self.clear_dest_btn)
        controls_layout.addStretch()
        
        layout.addLayout(controls_layout)
        
        # Widget del mapa
        self.map_view = QWebEngineView()
        layout.addWidget(self.map_view)
        
        # Información de navegación
        self.nav_info = QLabel("Seleccione un destino en el mapa haciendo clic")
        self.nav_info.setStyleSheet("""
            color: #3498db; 
            font-weight: bold; 
            background-color: #2c3e50;
            padding: 8px;
            border-radius: 4px;
            border: 1px solid #3498db;
        """)
        self.nav_info.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.nav_info)
    
    def setup_map(self):
        # Coordenadas iniciales (Universidad ejemplo)
        initial_lat = -17.783
        initial_lng = -63.182
        
        # Crear mapa Folium
        self.map = folium.Map(
            location=[initial_lat, initial_lng],
            zoom_start=17,
            tiles='OpenStreetMap',
            control_scale=True
        )
        
        # Añadir capa de satélite
        folium.TileLayer(
            tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
            attr='Google Satellite',
            name='Satélite'
        ).add_to(self.map)
        
        # Añadir control de capas
        folium.LayerControl().add_to(self.map)
        
        # Añadir script para capturar clicks
        click_script = """
        <script>
        function getClickLatLng(e) {
            var lat = e.latlng.lat;
            var lng = e.latlng.lng;
            window.pywebview.api.handle_map_click(lat, lng);
        }
        
        function addClickHandler() {
            map.on('click', getClickLatLng);
        }
        
        // Esperar a que el mapa esté listo
        if (map) {
            addClickHandler();
        } else {
            setTimeout(addClickHandler, 1000);
        }
        </script>
        """
        
        # Añadir el script al mapa
        self.map.get_root().html.add_child(folium.Element(click_script))
        
        # Guardar mapa temporalmente y cargarlo
        self.save_and_load_map()
    
    def save_and_load_map(self):
        # Crear archivo temporal para el mapa
        with tempfile.NamedTemporaryFile(suffix='.html', delete=False) as f:
            self.html_file = f.name
            self.map.save(self.html_file)
        
        # Cargar el mapa en el widget web
        self.map_view.setUrl(QUrl.fromLocalFile(self.html_file))
    
    def set_destination_from_input(self):
        text = self.coord_input.text().strip()
        if text:
            try:
                lat, lng = map(float, text.split(','))
                self.set_destination(lat, lng)
            except ValueError:
                QMessageBox.warning(self, "Error", "Formato inválido. Use: lat, lng")
    
    def set_destination(self, lat, lng):
        # Limpiar destino anterior
        if self.destination:
            self.map.get_root().script.get("children").remove(self.destination)
        
        # Crear nuevo marcador de destino
        self.destination = folium.Marker(
            [lat, lng],
            popup=f"Destino\nLat: {lat:.6f}\nLng: {lng:.6f}",
            tooltip="Destino del Robot",
            icon=folium.Icon(color='red', icon='flag', prefix='fa')
        )
        self.destination.add_to(self.map)
        
        # Actualizar interfaz
        self.coord_label.setText(f"Destino: {lat:.6f}, {lng:.6f}")
        self.nav_info.setText(f"🎯 Destino establecido - Lat: {lat:.6f}, Lng: {lng:.6f}")
        
        # Recrear el mapa
        self.save_and_load_map()
        
        # Imprimir en terminal (para prueba)
        print(f"DESTINO_SELECCIONADO: {lat:.6f}, {lng:.6f}")

        if self.ros_node:
            try:
                # Publicar directamente usando el método helper
                self.ros_node.publish_goal(lat, lng)
            except Exception as e:
                print(f"[MapWidget] Error publicando goal: {e}")
    
    def clear_destination(self):
        if self.destination:
            self.map.get_root().script.get("children").remove(self.destination)
            self.destination = None
        
        self.coord_label.setText("Coordenadas: No establecidas")
        self.coord_input.clear()
        self.nav_info.setText("Destino limpiado - Seleccione nuevo destino")
        self.save_and_load_map()
        print("DESTINO_LIMPIADO")
    
    def handle_map_click(self, lat, lng):
        """Manejador para clicks en el mapa (será conectado después)"""
        self.coord_input.setText(f"{lat:.6f}, {lng:.6f}")
        self.set_destination(lat, lng)
    
    def closeEvent(self, event):
        # Limpiar archivo temporal al cerrar
        if self.html_file and os.path.exists(self.html_file):
            os.unlink(self.html_file)
        super().closeEvent(event)