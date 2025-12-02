import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, BatteryState # İstenen veri tipleri 
import random
import time

class SensorSimNode(Node):
    def __init__(self):
        super().__init__('sensor_sim_node')
        
        # 1. GPS Publisher (NavSatFix)
        self.gps_pub = self.create_publisher(NavSatFix, 'havadash/gps', 10)
        
        # 2. Batarya Publisher (BatteryState)
        self.bat_pub = self.create_publisher(BatteryState, 'havadash/battery', 10)
        
        # Başlangıç Değerleri
        self.lat = 41.0123
        self.lon = 29.0567
        self.alt = 10.0
        self.battery_level = 100.0
        
        # 1Hz ile yayın yap
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Mock sensör yayını Başladı: /havadash/gps ve /havadash/battery")

    def timer_callback(self):
        # GPS Verisi Oluştur
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_link"
        
        # Rastgele hareket
        self.lat += random.uniform(-0.0001, 0.0001)
        self.lon += random.uniform(-0.0001, 0.0001)
        self.alt += random.uniform(-0.1, 0.1)
        
        gps_msg.latitude = self.lat
        gps_msg.longitude = self.lon
        gps_msg.altitude = self.alt
        
        self.gps_pub.publish(gps_msg)
        
        # Batarya Verisi Oluştur
        bat_msg = BatteryState()
        bat_msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.battery_level > 0:
            self.battery_level -= 0.1
            
        bat_msg.percentage = self.battery_level
        bat_msg.voltage = 12.0 # Örnek voltaj
        
        self.bat_pub.publish(bat_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()