#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32
from std_msgs.msg import Float32

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('number_subscriber')


        self.subscription = self.create_subscription(UInt32, 'number', self.listener_callback, 10) # Erstellung des Abonnements für 'number
        self.subscription                #Sicherstellen, dass das Abonnement erstellt wurde

        self.diff_publisher = self.create_publisher(Float32, 'diff', 10)
        self.timepoint = time.time()   # Speichern des aktuellen Zeitpunkts


    def listener_callback(self, msg):

        now = time.time()
        diff = Float32() # Berechnung der Differenz
        diff.data = now - self.timepoint
        self.diff_publisher.publish(diff) # Veröffentlichen der Differenz auf 'diff'
        self.timepoint = now # Aktualisieren des Zeitpunkts für die nächste Berechnung

        self.get_logger().info('Received: "%i" after %.5f s' % (msg.data, diff.data))



def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber() # Erstellung eines Objekts der Klasse MinimalSubscriber
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main() # Ausführung der Hauptfunktion, wenn das Skript direkt ausgeführt wird
    