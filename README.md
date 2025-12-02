# ROS 2 – MQTT Bridge (Dockerized Sanal Drone Companion)

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

## III. Kurulum ve Çalıştırma Talimatları (Havadash Ekibi İçin)

Projenin inşa edilmesi ve başlatılması için sadece **Git** ve **Docker/Docker Compose** gereklidir.

1.  **Repo'yu Klonlayın:**
    ```bash
    git clone https://github.com/yusuferyigit/ROS2-Humble-MQQT-Havadash-Astron.git
    cd ROS2-Humble-MQQT-Havadash-Astron
    ```

2.  **Sistemi İnşa Et ve Başlat:**
    Bu komut, ROS 2 ortamını kurar, kodu derler ve iki Node'u (`sensor_sim` ve `bridge_node`) eş zamanlı başlatır.
    ```bash
    docker compose up --build
    ```
    *(Varsayılan olarak DRONE_ID=drone-01 kullanılacaktır.)*

3.  **Veriyi İzleme (Havadash Backend View):**
    Başka bir terminalden telemetri verilerini canlı izleyebilirsiniz:
    ```bash
    docker run --rm eclipse-mosquitto mosquitto_sub -h test.mosquitto.org -t "havadash/telemetry/#"
    ```

4.  **Durdurma:**
    ```bash
    docker compose down
    ```
