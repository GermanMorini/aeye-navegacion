# Dockerfile para Sistema de Evasión de Obstáculos con LIDAR 3D + Pixhawk
# Base: ROS 2 Humble en Ubuntu 22.04 ARM64

FROM ros:humble-perception

# Metadatos
LABEL maintainer="aeye"
LABEL description="Sistema de evasión de obstáculos con LIDAR 3D + Pixhawk para ROS 2 Humble ARM64"

# Evitar prompts interactivos durante instalación
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Actualizar sistema e instalar dependencias
RUN apt-get update && apt-get install -y \
    # Dependencias ROS 2
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-xacro \
    # MAVROS para Pixhawk (ArduPilot/PX4)
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    # Simulacion 
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    # Python
    python3-pip \
    python3-yaml \
    python3-pytest \
    python3-serial \
    # Herramientas útiles
    git \
    wget \
    curl \
    nano \
    vim \
    # Limpieza
    && rm -rf /var/lib/apt/lists/*

# Instalar GeographicLib datasets para MAVROS (necesario para GPS)
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# Crear workspace ROS 2
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Capa de dependencias Python (pip)
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --no-cache-dir --force-reinstall \
        numpy==1.26.4 \
        flask==2.3.0 \
        matplotlib==3.7.0 \
        websockets>=11.0.0 \
        pyserial==3.5 \
        pymavlink==2.4.43 && \
    python3 - <<'PY'
import flask, matplotlib, websockets, numpy
print('Flask, Matplotlib, websockets y numpy', numpy.__version__, 'instalados correctamente')
PY

# Copiar código fuente del proyecto
COPY . /ros2_ws/src/navegacion_gps

# Instalar dependencias de rosdep
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /ros2_ws && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    rm -rf /var/lib/apt/lists/*"

# Compilar el workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install"

# Configurar entrypoint
COPY docker-entrypoint.sh /ros2_ws/docker-entrypoint.sh
RUN chmod +x /ros2_ws/docker-entrypoint.sh

# Source automático en .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=0" >> /root/.bashrc

# Exponer puertos para ROS 2 DDS
EXPOSE 7400-7500/udp
EXPOSE 11811/tcp

# Volúmenes para persistencia
VOLUME ["/ros2_ws/src", "/ros2_ws/install", "/ros2_ws/log"]

ENTRYPOINT ["/ros2_ws/docker-entrypoint.sh"]
CMD ["bash"]
