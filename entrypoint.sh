#!/bin/bash
set -e

# ROS ortamlarını ve projeyi yükle
source /opt/ros/humble/setup.bash
source /root/havadash_ws/install/setup.bash

echo "Havadash Drone Sistemi (Dockerize) Başlatılıyor..."

# 1. Sensor Simülasyonunu Arka Planda (&) Başlat
# Bu düğüm, GPS/Batarya verisini ROS Topic'ine basar.
ros2 run havadash_bridge sensor_sim &
PID_SENSOR=$!
echo "  [OK] Sensor Simülasyonu (PID: $PID_SENSOR) başlatıldı."

# 2. Bridge Node'u Ön Planda Başlat
# Bu düğüm, sensör verisini dinler, MQTT'ye basar ve Komutları dinler.
# Docker Compose'dan gelen DRONE_ID ve MQTT_BROKER parametrelerini kullanır.
echo "  [OK] Telemetry Bridge başlatılıyor..."

ros2 run havadash_bridge bridge_node --ros-args \
    -p drone_id:=${DRONE_ID} \
    -p mqtt_broker:=${MQTT_BROKER}

# Bridge Node durursa , sensor simülasyonunu da sonlandır
kill $PID_SENSOR
