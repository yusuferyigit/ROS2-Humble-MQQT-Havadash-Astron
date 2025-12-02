Elbette! Projenin tüm kurulum, mimari ve test adımlarını içeren, Havadash firmasına teslim edilmeye hazır nihai README.md metnini aşağıda tek bir blok halinde sunuyorum.

Bu belgeyi, terminalde açtığınız README.md dosyasına olduğu gibi yapıştırabilirsiniz.
Markdown

# ROS 2 – MQTT Bridge (Dockerized Virtual Drone Companion)

Bu proje, Havadash Firmware Mühendisliği Challenge kapsamında bir sanal drone companion yazılımının **ROS 2** üzerinde simülasyonunu sağlar. Sistem **Docker** üzerinde çalışır ve **MQTT** aracılığıyla telemetri iletimi ve komut alma işlevlerini yerine getirir.

---

## I. Sistem Mimarisi ve Veri Akışı

Sistem, ROS 2'nin **Tek Sorumluluk Prensibi (SRP)** gerekliliğine uygun olarak, tek bir paket içinde iki ayrı Node ile tasarlanmıştır.

### A. Mimari Yapı

| Node Adı | Rolü | Görev |
| :--- | :--- | :--- |
| **`sensor_sim_node`** | **Veri Kaynağı / ROS Publisher** | Mock (sahte) GPS, İrtifa ve Batarya verilerini ROS topic'lerine basar. |
| **`bridge_node`** | **Köprü & Kontrolcü / MQTT Client** | ROS topic'lerinden veriyi dinler, JSON'a çevirir, MQTT'ye basar ve Buluttan gelen komutları (LAND/TAKEOFF) dinler. |

### B. Veri Güvenliği ve Protokoller

* [cite_start]**Telemetri Formatı:** Veri, Havadash Backend'inin beklediği JSON yapısına (ISO8601 zaman damgalı, lat/lon/alt, speed, battery, status [cite: 19-30]) dönüştürülür.
* [cite_start]**Komut Akışı:** Gelen komutlar (B. Komut Payload [cite: 31-38][cite_start]) işlenir ve `ACCEPTED` veya `REJECTED` durumunda C. Komut ACK Payload [cite: 39-46] ile geri dönülür.
* **Parametrik Konfigürasyon:** `DRONE_ID` ve `MQTT_BROKER` adresleri, `docker-compose.yml` üzerinden dışarıdan yönetilir (ROS Parameters).

---

## II. Güvenlik (Fail-Safe) ve Gelişmiş Mantık

Proje, temel isterlerin ötesinde şu gelişmiş güvenlik ve fiziksel simülasyon mantığını içerir:

1.  **Hata Durumunda Eve Dönüş (RTL):** Pil seviyesi kritik seviyeye (%12) düştüğünde veya MQTT bağlantısı koptuğunda, drone otomatik olarak `status: "ERROR"` moduna geçer ve **başlangıç noktasına geri dönüşü** (RTL) simüle eder.
2.  **Veri Kesintisi Koruması:** ROS topic'lerinden 3 saniyeden fazla veri gelmezse (sensör kesintisi), Bridge Node, MQTT'ye en son bilinen konumu ve `status: "ERROR"` bilgisini basar (sessiz kalmak yerine sorunu raporlar).
3.  **Dinamik Hızlanma Modeli:** Komut alındığında (TAKEOFF/LAND), hız anlık olarak değişmez; `2.0m/s` ivme ile yavaşça hızlanır/yavaşlar (Maksimum hız: `16.0m/s`).

---

## III. Kurulum ve Çalıştırma Talimatları

Projenin inşa edilmesi ve başlatılması için yalnızca **Git** ve **Docker/Docker Compose** gereklidir.

### 1. Ön Koşul Kurulumları (Ubuntu 22.04 VM)

Eğer Docker kurulu değilse, aşağıdaki adımları uygulayın:

```bash
# A. Git ve Temel Araçları Kur
sudo apt update
sudo apt install git curl -y

# B. Docker Kurulumu
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL [https://download.docker.com/linux/ubuntu/gpg](https://download.docker.com/linux/ubuntu/gpg) | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] [https://download.docker.com/linux/ubuntu](https://download.docker.com/linux/ubuntu) $(. /etc/os-release && echo "$UBUNTU_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin -y

# C. Kullanıcıyı Docker Grubuna Ekle (Yetkiyi çözer)
sudo usermod -aG docker $USER
# UYARI: Yetkinin aktifleşmesi için oturumu kapatıp yeniden açın (Log Out / Log In).

2. Projenin Başlatılması
Bash

# A. Repo'yu Klonlayın
git clone [https://github.com/yusuferyigit/ROS2-Humble-MQQT-Havadash-Astron.git](https://github.com/yusuferyigit/ROS2-Humble-MQQT-Havadash-Astron.git)
cd ROS2-Humble-MQQT-Havadash-Astron

# B. Sistemi İnşa Et ve Başlat
# Bu komut, Dockerfile'ı çalıştırır, ROS'u kurar, kodu derler ve iki Node'u eş zamanlı başlatır.
docker compose up --build

IV. Test Komutları ve Hata Senaryoları

Bu komutlar, sistemin dinamik hızlanma ve Fail-Safe mantığını kontrol etmenizi sağlar. Lütfen Docker Loglarının aktığı terminalden AYRI bir terminalde çalıştırın.
A. Komut Kontrolü (Hızlanma / Yavaşlama)
Bash

# 1. KALKIŞ EMRİ: Hızı kademeli olarak 16.0m/s'ye çıkarır.
mosquitto_pub -h test.mosquitto.org -t "havadash/commands/docker-drone-01" -m '{"command_id": "TEST-01", "type": "TAKEOFF", "params": {}, "issued_at": "2025-12-02T17:00:00Z"}'

# 2. İNİŞ EMRİ: Hızı kademeli olarak 0.0m/s'ye düşürür.
mosquitto_pub -h test.mosquitto.org -t "havadash/commands/docker-drone-01" -m '{"command_id": "TEST-02", "type": "LAND", "params": {}, "issued_at": "2025-12-02T17:01:00Z"}'

B. Sensör Kesintisi (FAIL-SAFE) Testi

    Drone uçarken (IN_FLIGHT durumunda) docker compose up terminalinde akışı durdurun (Ctrl+C).

    docker compose up komutunu tekrar verip konteyneri başlatın.

    Hemen ardından sensor_sim_node'u durdurun (Örn: docker exec havadash_drone_instance kill -9 [sensor_sim'in PID'si]).

    bridge_node loglarında 3 saniye sonra VERİ KESİNTİSİ... MQTT'ye ERROR statüsü basıldı. uyarısını görmelisiniz.
