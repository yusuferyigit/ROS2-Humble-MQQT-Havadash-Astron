ROS 2 – MQTT Bridge (Dockerized Virtual Drone Companion)

Bu proje, Havadash Firmware Mühendisliği Challenge kapsamında bir sanal drone companion yazılımının ROS 2 üzerinde simülasyonunu sağlar. Sistem Docker üzerinde çalışır ve MQTT aracılığıyla telemetri iletimi ve komut alma işlevlerini yerine getirir.
I. Sistem Mimarisi ve Veri Akışı

Sistem, ROS 2'nin Single Responsibility Principle (SRP) gerekliliğine uygun olarak, tek bir paket içinde iki ayrı Node ile tasarlanmıştır.
A. Mimari Yapı
Node Adı	Rolü	Görev
sensor_sim_node	Veri Kaynağı / ROS Publisher	Mock (sahte) GPS, İrtifa ve Batarya verilerini ROS topic'lerine basar.
bridge_node	Köprü & Kontrolcü / MQTT Client	ROS topic'lerinden veriyi dinler, JSON'a çevirir, MQTT'ye basar ve Buluttan gelen komutları (LAND/TAKEOFF) dinler.
B. Veri Güvenliği ve Protokoller

    Telemetri Formatı: Veri, Havadash Backend'inin beklediği JSON yapısına (ISO8601 zaman damgalı, lat/lon/alt, speed, battery, status) dönüştürülür.

    Komut Akışı: Gelen komutlar işlenir ve ACCEPTED veya REJECTED durumunda komut ACK payload ile geri dönülür.

    Parametrik Konfigürasyon: DRONE_ID ve MQTT_BROKER adresleri, docker-compose.yml üzerinden dışarıdan yönetilir (ROS Parameters).

II. Güvenlik (Fail-Safe) ve Gelişmiş Simülasyon Mantığı

Proje, temel isterlerin ötesinde şu gelişmiş güvenlik ve fiziksel simülasyon mantığını içerir:

    Hata Durumunda Eve Dönüş (RTL): Pil seviyesi kritik seviyeye (%12) düştüğünde veya MQTT bağlantısı koptuğunda, drone otomatik olarak status: "ERROR" moduna geçer ve başlangıç noktasına geri dönüşü (RTL) simüle eder.

    Veri Kesintisi Koruması: ROS topic'lerinden 3 saniyeden fazla veri gelmezse (sensör kesintisi), Bridge Node, MQTT'ye en son bilinen konumu ve status: "ERROR" bilgisini basar (sessiz kalmak yerine sorunu raporlar).

    Dinamik Hızlanma Modeli: Komut alındığında (TAKEOFF/LAND), hız anlık olarak değişmez; 2.0m/s ivme ile yavaşça hızlanır/yavaşlar (Maksimum hız: 16.0m/s).

III. Kurulum ve Çalıştırma Talimatları

Projenin inşa edilmesi ve başlatılması için sadece Git ve Docker/Docker Compose gereklidir.
1. Ön Koşul Kurulumları (Önceden Kurulmadıysa)
bash

# A. Git ve Temel Araçları Kur
sudo apt update
sudo apt install git curl -y

# B. Docker Kurulumu (Ubuntu 22.04)
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin -y

# C. Kullanıcıyı Docker Grubuna Ekle (Yetkiyi çözer)
sudo usermod -aG docker $USER
# UYARI: Yetkinin aktifleşmesi için oturumu kapatıp yeniden açın (Log Out / Log In).

2. Projenin Başlatılması
bash

# A. Repo'yu Klonlayın
git clone https://github.com/yusuferyigit/ROS2-Humble-MQQT-Havadash-Astron.git
cd ROS2-Humble-MQQT-Havadash-Astron

# B. Sistemi İnşa Et ve Başlat
docker compose up --build

3. Test Komutları ve İzleme

Veriyi İzleme (Havadash Backend View):
bash

docker run --rm eclipse-mosquitto mosquitto_sub -h test.mosquitto.org -t "havadash/telemetry/#"

Örnek Komut Gönderimi (Client -> Drone):
bash

# KALKIŞ EMRİ
mosquitto_pub -h test.mosquitto.org -t "havadash/commands/drone-01" -m '{"command_id": "TEST-01", "type": "TAKEOFF", "params": {}, "issued_at": "2025-12-02T17:00:00Z"}'

# İNİŞ EMRİ
mosquitto_pub -h test.mosquitto.org -t "havadash/commands/drone-01" -m '{"command_id": "TEST-02", "type": "LAND", "params": {}, "issued_at": "2025-12-02T17:01:00Z"}'

4. Durdurma
bash

docker compose down
