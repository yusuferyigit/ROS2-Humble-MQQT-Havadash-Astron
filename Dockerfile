#ROS 2 Humble
FROM ros:humble-ros-base

# Gerekli Python araçlarını (pip, colcon) ve git'i kur
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/apt/lists/*

# Paho-MQTT kütüphanesini kur
RUN pip3 install paho-mqtt

# Çalışma klasörünü ayarla
WORKDIR /root/havadash_ws

# Projenin tüm dosyalarını (src, docker-compose, entrypoint) konteynere kopyala
COPY . .

# Projeyi derle 
RUN /bin/bash -c ". /opt/ros/humble/setup.bash && colcon build --packages-select havadash_bridge"

# Başlatma scriptini kopyala ve yetki ver
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Konteyner açılınca bu scripti çalıştır
ENTRYPOINT ["/entrypoint.sh"]