#!/bin/bash
# Script para construir la imagen Docker del proyecto

set -e

echo "==========================================="
echo "  Construyendo imagen Docker"
echo "  Proyecto: cuatri-evasion-obstaculos"
echo "==========================================="

# Colores para output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Verificar que estamos en el directorio correcto
if [ ! -f "Dockerfile" ]; then
    echo -e "${RED}Error: Dockerfile no encontrado${NC}"
    echo "Ejecuta este script desde el directorio del proyecto"
    exit 1
fi

# Permitir X11 para GUI
echo -e "${YELLOW}Permitiendo conexiones X11...${NC}"
xhost +local:docker || echo "Warning: xhost falló, RViz puede no funcionar"

# Construir imagen
echo -e "${YELLOW}Construyendo imagen Docker...${NC}"
docker compose build cuatri_navigation

echo -e "${GREEN}✓ Imagen construida exitosamente${NC}"
echo ""
echo "Para ejecutar el contenedor:"
echo "  ./docker-run.sh"
echo ""
echo "Para ejecutar con docker-compose:"
echo "  docker compose up -d"
