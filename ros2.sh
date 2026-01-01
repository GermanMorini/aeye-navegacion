#!/bin/bash
# Script para ejecutar el contenedor Docker

set -e

# Colores
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "==========================================="
echo "  Ejecutando contenedor Docker"
echo "  Proyecto: cuatri-evasion-obstaculos"
echo "==========================================="

# Permitir X11
xhost +local:docker 2>/dev/null || true

# Verificar si el contenedor ya existe
if [ "$(docker ps -aq -f name=ros2_humble)" ]; then
    echo -e "${YELLOW}Contenedor existente encontrado${NC}"

    # Si está corriendo, hacer exec
    if [ "$(docker ps -q -f name=ros2_humble)" ]; then
        echo -e "${GREEN}Conectando al contenedor en ejecución...${NC}"
        docker exec -it ros2_humble bash
    else
        # Si existe pero no está corriendo, iniciarlo
        echo -e "${YELLOW}Iniciando contenedor...${NC}"
        docker start ros2_humble
        docker exec -it ros2_humble bash
    fi
else
    # No existe, usar docker-compose
    echo -e "${GREEN}Iniciando nuevo contenedor...${NC}"
    docker compose up -d ros2_humble
    docker exec -it ros2_humble bash
fi
