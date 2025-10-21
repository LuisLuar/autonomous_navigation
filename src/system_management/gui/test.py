import sys
import folium
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl
import tempfile
import os

class MapViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Crear mapa
        tiles_path = "/home/raynel/Documents/offline_title"
        
        mapa = folium.Map(
            location=[-0.317604, -78.444711],
            zoom_start=17,
            tiles=None
        )
        
        folium.TileLayer(
            tiles=f'file://{tiles_path}/{{z}}/{{x}}/{{y}}.png',
            attr='Offline Map',
            name='Offline'
        ).add_to(mapa)
        
        # Guardar temporalmente
        with tempfile.NamedTemporaryFile(suffix='.html', delete=False) as f:
            mapa.save(f.name)
            temp_file = f.name
        
        # Configurar UI
        self.setWindowTitle("Mapa Offline")
        self.setGeometry(100, 100, 800, 600)
        
        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)
        
        self.web_view = QWebEngineView()
        self.web_view.setUrl(QUrl.fromLocalFile(temp_file))
        
        layout.addWidget(self.web_view)
        self.setCentralWidget(central_widget)
        
        # Limpiar al cerrar
        self.temp_file = temp_file
    
    def closeEvent(self, event):
        if os.path.exists(self.temp_file):
            os.unlink(self.temp_file)
        event.accept()

# Ejecutar
app = QApplication(sys.argv)
viewer = MapViewer()
viewer.show()
sys.exit(app.exec_())