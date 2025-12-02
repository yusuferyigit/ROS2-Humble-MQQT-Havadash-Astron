ROS 2 – MQTT Bridge (Dockerized Virtual Drone Companion)

Bu proje, Havadash Firmware Mühendisliği Challenge kapsamında sanal bir drone companion sisteminin ROS 2 üzerinde simülasyonunu sağlar. Sistem tamamen Docker üzerinde çalışır ve MQTT kullanarak telemetri iletimi ve komut işleme görevlerini yerine getirir.

I. Sistem Mimarisi ve Veri Akışı

Sistem, ROS 2'nin tek sorumluluk prensibine (SRP) uygun olarak tek bir paket içinde iki bağımsız node ile tasarlanmıştır.

1. Mimari Yapı
Node Adı	Rolü	Açıklama
sensor_sim_node	Veri Kaynağı / ROS Publisher	Mock GPS, irtifa ve batarya verilerini ROS topic'lerine yayınlar.
bridge_node	Köprü & Kontrolcü / MQTT Client	ROS verisini dinler, JSON formatına dönüştürerek MQTT'ye iletir; MQTT üzerinden gelen komutları işleyerek drone durumunu günceller.
2. Veri Güvenliği ve Protokoller

Telemetri Formatı: ISO8601 zaman damgası, konum, hız, batarya ve status alanlarıyla Havadash Backend formatına tam uyumludur.

Komut Akışı: TAKEOFF veya LAND komutları işlenir, sonuç olarak ACCEPTED veya REJECTED içerikli ACK geri gönderilir.

Parametrik Konfigürasyon: Drone ID, MQTT broker adresi gibi tüm değişkenler docker-compose.yml üzerinden yönetilir.

II. Güvenlik (Fail-Safe) ve Gelişmiş Simülasyon Mantığı

Bu proje yalnızca temel simülasyon görevlerini değil, gerçek sistem davranışına benzer gelişmiş güvenlik mekanizmalarını da içerir.

Kritik Pil Durumu (Auto-RTL):
Pil seviyesi %12’nin altına düştüğünde veya MQTT bağlantısı kesildiğinde drone “ERROR” durumuna geçer ve otomatik eve dönüş (RTL) davranışı simüle edilir.

Veri Kesintisi Koruması:
Sensör verisi 3 saniyeden uzun süre gelmezse bridge node en son bilinen konumu koruyarak durumu ERROR olarak MQTT'ye raporlar.

Dinamik Hızlanma Modeli:
TAKEOFF veya LAND komutlarında hız sabit olarak değil, 2.0 m/s ivme ile kademeli olarak değişir. Maksimum hız: 16.0 m/s.

III. Kurulum ve Çalıştırma Talimatları

Projeyi çalıştırmak için Git ve Docker/Docker Compose kurulumu yeterlidir.

1. Ön Koşul Kurulumları (Ubuntu 22.04)
# Git ve temel araçlar
sudo apt update && sudo apt install git curl -y

# Docker kurulumu
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") stable" \
| sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin -y

# Kullanıcı yetkilendirme
sudo usermod -aG docker $USER

IV. Projenin Çalıştırılması
1. Repo'nun Klonlanması
git clone https://github.com/yusuferyigit/ROS2-Humble-MQQT-Havadash-Astron.git
cd ROS2-Humble-MQQT-Havadash-Astron

2. Sistemin İnşa Edilmesi ve Başlatılması
docker compose up --build

V. Telemetri İzleme ve Komut Gönderme
1. Telemetri İzleme
docker run --rm eclipse-mosquitto mosquitto_sub \
  -h test.mosquitto.org -t "havadash/telemetry/#"

2. Örnek Komut Gönderimleri

Kalkış (TAKEOFF):

mosquitto_pub -h test.mosquitto.org \
  -t "havadash/commands/docker-drone-01" \
  -m '{"command_id": "TEST-01", "type": "TAKEOFF", "params": {}, "issued_at": "2025-12-02T17:00:00Z"}'


İniş (LAND):

mosquitto_pub -h test.mosquitto.org \
  -t "havadash/commands/docker-drone-01" \
  -m '{"command_id": "TEST-02", "type": "LAND", "params": {}, "issued_at": "2025-12-02T17:01:00Z"}'

VI. Sistemi Durdurma
docker compose down
