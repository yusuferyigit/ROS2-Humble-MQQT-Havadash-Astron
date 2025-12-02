import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, BatteryState
import json
import math
import paho.mqtt.client as mqtt
from datetime import datetime

class HavadashBridge(Node):
    def __init__(self):
        super().__init__('havadash_bridge_node')
        
        # KONFİGÜRASYON
        self.declare_parameter('mqtt_broker', 'test.mosquitto.org')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('drone_id', 'test-drone-01')

        self.broker = self.get_parameter('mqtt_broker').get_parameter_value().string_value
        self.port = self.get_parameter('mqtt_port').get_parameter_value().integer_value
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        # MQTT CONFIG
        self.topic_telemetry = f"havadash/telemetry/{self.drone_id}"
        self.topic_command = f"havadash/commands/{self.drone_id}"
        self.topic_ack = f"havadash/commands/{self.drone_id}/ack"

        self.drone_status = "IN_FLIGHT"
        
        # Ev Konumu
        self.home_lat = 41.0123
        self.home_lon = 29.0567
        self.home_alt = 0.0

        # Anlık Konum
        self.current_lat = self.home_lat
        self.current_lon = self.home_lon
        self.current_alt = 0.0
        self.current_battery = None
        
        # Zaman Aşımı Kontrolü
        self.last_data_time = self.get_clock().now()
        self.data_timeout_threshold = 3.0 # 3 saniye tolerans
        
        # Fizik
        self.current_speed = 0.0
        self.max_speed = 16.0
        self.acceleration = 2.0

        # ROS SUBSCRIPTIONS
        self.create_subscription(NavSatFix, 'havadash/gps', self.gps_callback, 10)
        self.create_subscription(BatteryState, 'havadash/battery', self.battery_callback, 10)

        # MQTT CLIENT
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_client.on_message = self.on_message
        
        try:
            self.mqtt_client.connect(self.broker, self.port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT Bağlantı Hatası: {e}")
            self.drone_status = "ERROR" 

        self.timer = self.create_timer(1.0, self.publish_telemetry)

    def gps_callback(self, msg):
        self.last_data_time = self.get_clock().now()
        if self.drone_status != "ERROR":
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude
            self.current_alt = msg.altitude

    def battery_callback(self, msg):
        self.last_data_time = self.get_clock().now()
        self.current_battery = int(msg.percentage)
        
        if self.current_battery <= 12:
            if self.drone_status != "ERROR" and self.drone_status != "LANDED":
                self.get_logger().error(f"KRİTİK BATARYA (%{self.current_battery})! MOD: ERROR -> RTL")
                self.drone_status = "ERROR"
        elif self.current_battery <= 20:
             if self.drone_status == "IN_FLIGHT":
                 self.get_logger().warn(f"Düşük Pil Uyarısı: %{self.current_battery}")

    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            self.get_logger().info(f"MQTT Bağlandı. Kanal: {self.topic_command}")
            client.subscribe(self.topic_command)

    def on_disconnect(self, client, userdata, flags, rc, properties=None):
        self.get_logger().error(f"MQTT KOPTU! Kod: {rc}")
        if self.drone_status != "LANDED":
            self.drone_status = "ERROR"

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            cmd_type = payload.get("type", "UNKNOWN")
            cmd_id = payload.get("command_id", "unknown")
            
            self.get_logger().info(f"KOMUT: {cmd_type} (ID: {cmd_id})")

            ack_status = "REJECTED"
            ack_msg = "Gecersiz"

            # Sensör verisi yoksa veya ERROR modundaysa komut alma
            time_diff = (self.get_clock().now() - self.last_data_time).nanoseconds / 1e9
            
            if time_diff > self.data_timeout_threshold:
                 ack_status = "REJECTED"
                 ack_msg = "Sensör verisi yok (Timeout). Komut reddedildi."
            elif self.drone_status == "ERROR":
                ack_status = "REJECTED"
                ack_msg = "RTL Modunda (ERROR). Komut reddedildi."
            else:
                if cmd_type == "TAKEOFF":
                    if self.drone_status in ["LANDED", "IDLE"]:
                        self.drone_status = "IN_FLIGHT"
                        ack_status = "ACCEPTED"
                        ack_msg = "Kalkis baslatildi."
                    else:
                        ack_msg = "Zaten havada."

                elif cmd_type == "LAND":
                    if self.drone_status == "IN_FLIGHT":
                        self.drone_status = "LANDED"
                        ack_status = "ACCEPTED"
                        ack_msg = "Inis baslatildi."
                    else:
                        ack_msg = "Zaten yerde."

                elif cmd_type == "PAUSE":
                    self.drone_status = "IDLE"
                    ack_status = "ACCEPTED"
                    ack_msg = "Gorev duraklatildi."

            ack_payload = {
                "command_id": cmd_id,
                "drone_id": self.drone_id,
                "status": ack_status,
                "message": ack_msg
            }
            client.publish(self.topic_ack, json.dumps(ack_payload))
            self.get_logger().info(f"ACK: {ack_status} -> {ack_msg}")

        except Exception as e:
            self.get_logger().error(f"Komut Hatası: {e}")

    def execute_rtl_logic(self):
        lat_diff = self.home_lat - self.current_lat
        lon_diff = self.home_lon - self.current_lon
        
        if abs(lat_diff) > 0.00005 or abs(lon_diff) > 0.00005:
            step = 0.0001
            self.current_lat += step if lat_diff > 0 else -step
            self.current_lon += step if lon_diff > 0 else -step
            self.current_speed = 10.0
            self.get_logger().info(f"RTL: Eve Dönülüyor... (Mesafe: {abs(lat_diff)+abs(lon_diff):.5f})")
        else:
            if self.current_alt > 0.5:
                self.current_alt -= 0.5
                self.current_speed = 0.0
                self.get_logger().info(f"RTL: İniş Yapılıyor... Alt: {self.current_alt:.1f}m")
            else:
                self.current_alt = 0.0
                self.drone_status = "LANDED"
                self.get_logger().info("RTL Tamamlandı. Güvenli İniş Yapıldı.")

    def publish_telemetry(self):
        # 1. ZAMAN AŞIMI KONTROLÜ
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_data_time).nanoseconds / 1e9
        
        # Eğer veri gelmiyorsa ERROR statüsü ile son bildiğimiz konumu bas
        if time_diff > self.data_timeout_threshold:
            telemetry = {
                "drone_id": self.drone_id,
                "timestamp": datetime.utcnow().isoformat() + "Z",
                "lat": round(self.current_lat, 6),
                "lon": round(self.current_lon, 6),
                "alt": round(self.current_alt, 2),
                "speed": 0.0, # Veri yoksa hız 0 kabul edilir
                "battery": self.current_battery if self.current_battery else 0,
                "status": "ERROR" # Havadash sunucusu 'ERROR' görsün 
            }
            try:
                self.mqtt_client.publish(self.topic_telemetry, json.dumps(telemetry))
            except Exception:
                pass
            
            # Terminale de Uyarı Bas
            self.get_logger().warn(f"VERİ KESİNTİSİ ({time_diff:.1f}s)! MQTT'ye ERROR statüsü basıldı.")
            return # Normal fizik mantığını atla

        # Veri akışı varsa:
        if self.current_battery is None:
            self.get_logger().info("Sensör verisi bekleniyor...")
            return

        if self.drone_status == "ERROR":
            self.execute_rtl_logic()
        
        elif self.drone_status == "IN_FLIGHT":
            if self.current_speed < self.max_speed:
                self.current_speed += self.acceleration
                if self.current_speed > self.max_speed: self.current_speed = self.max_speed
        elif self.drone_status in ["LANDED", "IDLE"]:
            if self.current_speed > 0.0:
                self.current_speed -= self.acceleration
                if self.current_speed < 0.0: self.current_speed = 0.0

        telemetry = {
            "drone_id": self.drone_id,
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "lat": round(self.current_lat, 6),
            "lon": round(self.current_lon, 6),
            "alt": round(self.current_alt, 2),
            "speed": round(self.current_speed, 1),
            "battery": self.current_battery,
            "status": self.drone_status
        }
        
        try:
            self.mqtt_client.publish(self.topic_telemetry, json.dumps(telemetry))
        except Exception:
            pass
        
        log_msg = (f"ID:{self.drone_id} | Status:{self.drone_status} | "
                   f"Speed:{self.current_speed}m/s | Battery:%{self.current_battery} | "
                   f"Lat:{self.current_lat:.5f} Lon:{self.current_lon:.5f} Alt:{self.current_alt:.2f}m")
        self.get_logger().info(log_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HavadashBridge()
    rclpy.spin(node)
    node.mqtt_client.loop_stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()