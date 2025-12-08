#!/usr/bin/env python3
import torch
import sys
import os

# Agregar el path para importar los modelos
sys.path.append('/home/raynel/autonomous_navigation/src/perception_stack/semantic_segmentation')

from models.bisenetv1 import BiSeNetV1

def main():
    print("🚀 Convirtiendo BiSeNetV1 a ONNX...")
    
    # Cargar modelo
    model = BiSeNetV1(19)
    state = torch.load("model_final_v1_city_new.pth", map_location="cpu")
    model.load_state_dict(state, strict=False)
    model.eval()
    print("✅ Modelo BiSeNetV1 cargado")

    # Crear dummy input
    dummy_input = torch.randn(1, 3, 512, 512)
    print("✅ Input dummy creado")

    # Exportar a ONNX
    torch.onnx.export(
        model, 
        dummy_input,
        "bisenetv1.onnx",
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}},
        opset_version=17,
        verbose=True
    )
    print("✅ ONNX exportado exitosamente: bisenetv1.onnx")

if __name__ == "__main__":
    main()