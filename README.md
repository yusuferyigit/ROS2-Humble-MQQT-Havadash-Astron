ROS 2 – MQTT Bridge (Dockerized Sanal Drone Companion)

Bu proje, Havadash Firmware Mühendisliği Challenge kapsamında bir sanal drone companion yazılımının ROS 2 üzerinde simülasyonunu sağlar. Sistem Docker üzerinde çalışır ve MQTT aracılığıyla telemetri iletimi ve komut alma işlevlerini yerine getirir.
1. Sistem Mimarisi ve Veri Akışı

Sistem tek bir paket altında iki bağımsız ROS 2 Node’dan oluşur.

    1.1 sensor_sim_node: Bu node; GPS, irtifa, hız ve batarya gibi sahte sensör verilerini üretir ve ROS topic'lerine yayınlar. Gerçek bir uçuş kontrolcüsünün davranışını taklit edecek şekilde periyodik olarak veri sağlar.

    1.2 bridge_node: Bu node; sensör topic'lerini dinler, gelen veriyi Havadash Backend’in beklediği JSON formatına dönüştürür ve MQTT’ye iletir. Aynı zamanda bulut üzerinden gönderilen TAKEOFF ve LAND komutlarını dinler, doğrular ve karşılığında ACCEPTED veya REJECTED durumunda bir ACK mesajı üretir.

1.3 Telemetri ve Komut Protokolleri

Telemetri yapısı ISO8601 zaman damgası, konum (lat, lon, alt), speed, battery ve status alanlarını içerir.

Komut akışı TAKEOFF/LAND komutlarının alınması, doğrulanması ve uygun ACK mesajının yayınlanması prensibiyle çalışır. Konfigürasyon tamamen parametriktir; DRONE_ID, MQTT_BROKER gibi değerler docker-compose üzerinden set edilir.

Örnek Komut Gönderimi (Client -> Drone): Drone'a TAKEOFF emri vermek için MQTT Publisher kullanılır. Bu, Bridge Node'u IN_FLIGHT moduna geçirir.
Bash

# Terminalden örnek komut gönderme
mosquitto_pub -h test.mosquitto.org -t "havadash/commands/drone-01" -m '{
  "command_id": "cmd-takeoff-001",
  "type": "TAKEOFF",
  "params": {},
  "issued_at": "2025-12-02T16:55:00Z"
}'

2. Fail-Safe ve Gelişmiş Simülasyon Mantığı

Projede temel isterlerin ötesine geçen çeşitli güvenlik ve davranış mekanizmaları bulunmaktadır:

    2.1 Pil Kritik Seviyesinde RTL (Return-To-Launch): Batarya seviyesi yüzde 12’nin altına düştüğünde drone, status alanını “ERROR” olarak işaretler ve başlangıç konumuna dönüş (RTL) davranışını simüle eder.

    2.2 MQTT veya Sensör Kesintisi Koruması: MQTT bağlantısı koptuğunda veya ROS sensör topic’lerinden üç saniyeden uzun süre veri gelmediğinde sistem telemetriyi sessizce durdurmak yerine mevcut son veriyi “ERROR” status ile yayınlar.

    2.3 Dinamik Hızlanma Modeli: Drone TAKEOFF veya LAND komutu aldığında hız anlık olarak değişmez. Maksimum 16 m/s hıza ulaşana kadar yaklaşık 2.0 m/s ivme ile yükselir veya yavaşlar. Bu, fiziksel bir sistemin daha gerçekçi hatlarını verir.

3. Kurulum ve Çalıştırma

Aşağıdaki adımlar yalnızca Git ve Docker gerektirir. ROS 2 Humble doğrudan Docker imajı içinde çalışır.

    3.1 Repo’yu Klonlayın:
    Bash

git clone https://github.com/yusuferyigit/ROS2-Humble-MQQT-Havadash-Astron.git
cd ROS2-Humble-MQQT-Havadash-Astron

3.2 Sistemi İnşa Edin ve Başlatın: Bu komut ROS 2 ortamını hazırlar, paketleri derler ve hem sensor_sim hem de bridge_node’u başlatır.
Bash

docker compose up --build

(Varsayılan olarak DRONE_ID=drone-01 kullanılacaktır.)

3.3 MQTT Üzerinden Telemetriyi İzleyin: Başka bir terminalden telemetri akışını gözlemleyebilirsiniz:
Bash

docker run --rm eclipse-mosquitto mosquitto_sub -h test.mosquitto.org -t "havadash/telemetry/#"

3.4 Sistemi Durdurun:
Bash

docker compose down
