#!/bin/bash

# Script para descargar Leaflet y recursos offline
# Uso: ./download_leaflet_offline.sh

set -e

# Colores para output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Descargando Leaflet para uso OFFLINE${NC}"
echo -e "${GREEN}========================================${NC}\n"

# Determinar directorio de destino
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEAFLET_DIR="${SCRIPT_DIR}/leaflet_offline"

echo -e "${YELLOW}Directorio de destino:${NC} $LEAFLET_DIR"

# Crear directorio si no existe
mkdir -p "$LEAFLET_DIR/images"

# Versión de Leaflet
LEAFLET_VERSION="1.9.4"
echo -e "\n${YELLOW}Descargando Leaflet v${LEAFLET_VERSION}...${NC}\n"

# Descargar CSS
echo -e "${GREEN}[1/8]${NC} Descargando leaflet.css..."
wget -q --show-progress -O "$LEAFLET_DIR/leaflet.css" \
    "https://unpkg.com/leaflet@${LEAFLET_VERSION}/dist/leaflet.css"

# Descargar JS
echo -e "${GREEN}[2/8]${NC} Descargando leaflet.js..."
wget -q --show-progress -O "$LEAFLET_DIR/leaflet.js" \
    "https://unpkg.com/leaflet@${LEAFLET_VERSION}/dist/leaflet.js"

# Descargar imágenes
echo -e "${GREEN}[3/8]${NC} Descargando marker-icon.png..."
wget -q --show-progress -O "$LEAFLET_DIR/images/marker-icon.png" \
    "https://unpkg.com/leaflet@${LEAFLET_VERSION}/dist/images/marker-icon.png"

echo -e "${GREEN}[4/8]${NC} Descargando marker-icon-2x.png..."
wget -q --show-progress -O "$LEAFLET_DIR/images/marker-icon-2x.png" \
    "https://unpkg.com/leaflet@${LEAFLET_VERSION}/dist/images/marker-icon-2x.png"

echo -e "${GREEN}[5/8]${NC} Descargando marker-shadow.png..."
wget -q --show-progress -O "$LEAFLET_DIR/images/marker-shadow.png" \
    "https://unpkg.com/leaflet@${LEAFLET_VERSION}/dist/images/marker-shadow.png"

echo -e "${GREEN}[6/8]${NC} Descargando layers.png..."
wget -q --show-progress -O "$LEAFLET_DIR/images/layers.png" \
    "https://unpkg.com/leaflet@${LEAFLET_VERSION}/dist/images/layers.png"

echo -e "${GREEN}[7/8]${NC} Descargando layers-2x.png..."
wget -q --show-progress -O "$LEAFLET_DIR/images/layers-2x.png" \
    "https://unpkg.com/leaflet@${LEAFLET_VERSION}/dist/images/layers-2x.png"

# Descargar marcadores de colores para uso offline
echo -e "\n${YELLOW}Descargando marcadores de colores...${NC}"

echo -e "${GREEN}[8/14]${NC} Descargando marker-icon-red.png..."
wget -q --show-progress -O "$LEAFLET_DIR/images/marker-icon-red.png" \
    "https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-red.png"

echo -e "${GREEN}[9/14]${NC} Descargando marker-icon-green.png..."
wget -q --show-progress -O "$LEAFLET_DIR/images/marker-icon-green.png" \
    "https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-green.png"

echo -e "${GREEN}[10/14]${NC} Descargando marker-icon-blue.png..."
wget -q --show-progress -O "$LEAFLET_DIR/images/marker-icon-blue.png" \
    "https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-blue.png"

echo -e "${GREEN}[11/14]${NC} Descargando marker-icon-orange.png..."
wget -q --show-progress -O "$LEAFLET_DIR/images/marker-icon-orange.png" \
    "https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-orange.png"

echo -e "${GREEN}[12/14]${NC} Descargando marker-icon-yellow.png..."
wget -q --show-progress -O "$LEAFLET_DIR/images/marker-icon-yellow.png" \
    "https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-yellow.png"

echo -e "${GREEN}[13/14]${NC} Descargando marker-icon-violet.png..."
wget -q --show-progress -O "$LEAFLET_DIR/images/marker-icon-violet.png" \
    "https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-violet.png"

# Corregir rutas en el CSS
echo -e "${GREEN}[14/14]${NC} Corrigiendo rutas en leaflet.css..."
sed -i 's|url(images/|url(./images/|g' "$LEAFLET_DIR/leaflet.css"

# Verificar descargas
echo -e "\n${YELLOW}Verificando archivos descargados...${NC}"
TOTAL_FILES=7
FOUND_FILES=0

FILES_TO_CHECK=(
    "leaflet.css"
    "leaflet.js"
    "images/marker-icon.png"
    "images/marker-icon-2x.png"
    "images/marker-shadow.png"
    "images/layers.png"
    "images/layers-2x.png"
)

for file in "${FILES_TO_CHECK[@]}"; do
    if [ -f "$LEAFLET_DIR/$file" ]; then
        SIZE=$(du -h "$LEAFLET_DIR/$file" | cut -f1)
        echo -e "  ${GREEN}✓${NC} $file (${SIZE})"
        ((FOUND_FILES++))
    else
        echo -e "  ${RED}✗${NC} $file (NO ENCONTRADO)"
    fi
done

# Resumen
echo -e "\n${GREEN}========================================${NC}"
if [ $FOUND_FILES -eq $TOTAL_FILES ]; then
    echo -e "${GREEN}✓ Descarga completada exitosamente!${NC}"
    echo -e "${GREEN}  Archivos: ${FOUND_FILES}/${TOTAL_FILES}${NC}"
    echo -e "\n${YELLOW}Ubicación:${NC} $LEAFLET_DIR"
    echo -e "\n${YELLOW}Siguiente paso:${NC}"
    echo -e "  Asegúrate de que tu MapWidget use esta ruta:"
    echo -e "  ${GREEN}leaflet_path='$LEAFLET_DIR'${NC}"
else
    echo -e "${RED}✗ Algunos archivos no se descargaron${NC}"
    echo -e "${RED}  Archivos: ${FOUND_FILES}/${TOTAL_FILES}${NC}"
    exit 1
fi
echo -e "${GREEN}========================================${NC}\n"