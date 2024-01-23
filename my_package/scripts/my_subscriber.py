#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32
from std_msgs.msg import Float32

class MinSub(Node):
    def __init__(self):
        super().__init__('number_subscriber')

        self.subscription = self.create_subscription(UInt32, 'number', self.listener_callback, 10) # Erstellung des Abonnements
        self.subscription

        self.diff_publisher = self.create_publisher(Float32, 'diff', 10) # Erstellung des Publishers für das 'diff'-Thema
        self.timepoint = time.time() # Speichern des aktuellen Zeitpunkts

    def listener_callback(self, msg):
        now = time.time()
        
        # Berechnung von differenz
        diff = Float32()
        diff.data = now - self.timepoint

        self.diff_publisher.publish(diff) # Veröffentlichen der Differenz
        
        self.timepoint = now # Aktualisieren des Zeitpunkts für nächste Berechnung
        self.get_logger().info('Received: "%i" after %.5f s' % (msg.data, diff.data)) # Ausgabe der empfangenen Nachricht und Zeitdifferenz

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinSub() # Erstellung eines Objekts der Klasse MinSub

    rclpy.spin(minimal_subscriber)
    
    minimal_subscriber.destroy_node() # Zerstörung des Knotens
    rclpy.shutdown()

if __name__ == '__main__':
    # Ausführung der Hauptfunktion, wenn das Skript direkt ausgeführt wird
    main()
