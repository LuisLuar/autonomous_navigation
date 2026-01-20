#!/usr/bin/env python3
"""
Script para renombrar archivos de detecciones de _obj.json a _det.json
"""

import os
import glob
from pathlib import Path

def rename_detection_files(session_path):
    """Renombra archivos de detecciones en una sesión."""
    session_path = Path(session_path)
    detection_dir = session_path / "perception" / "detections"
    
    if not detection_dir.exists():
        print(f"Error: No existe el directorio {detection_dir}")
        return
    
    # Buscar todos los archivos _obj.json
    obj_files = list(detection_dir.glob("*_obj.json"))
    print(f"Encontrados {len(obj_files)} archivos *_obj.json")
    
    renamed_count = 0
    for obj_file in obj_files:
        # Crear nuevo nombre reemplazando _obj.json por _det.json
        new_name = str(obj_file).replace("_obj.json", "_det.json")
        new_path = Path(new_name)
        
        try:
            # Renombrar archivo
            os.rename(obj_file, new_path)
            renamed_count += 1
            print(f"  Renombrado: {obj_file.name} -> {new_path.name}")
        except Exception as e:
            print(f"  Error renombrando {obj_file.name}: {e}")
    
    print(f"\nTotal archivos renombrados: {renamed_count}/{len(obj_files)}")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Renombra archivos de detecciones de _obj.json a _det.json')
    parser.add_argument('session_path', type=str, 
                       help='Ruta a la carpeta de la sesión')
    
    args = parser.parse_args()
    
    rename_detection_files(args.session_path)

if __name__ == "__main__":
    main()
