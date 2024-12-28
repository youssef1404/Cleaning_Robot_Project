import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float32MultiArray
import keyboard

class Keyboard(Node):
    def __init__(self):
        super().__init__("keyboard_publisher")
        self.publisher = self.create_publisher(Float32MultiArray, 'speed_setpoint', 10)
        self.get_logger().info("Keyboard Node Started")
        self.msgsent = [0.0,0.0]
        self.speed = 255.0
        self.turn_speed = 126.0

    def send_command(self, command):
        msg = Float32MultiArray()
        msg.data = self.msgsent
        self.publisher.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    K_publisher = Keyboard()

    try:

        while rclpy.ok():
            if keyboard.is_pressed('w'):
                K_publisher.msgsent = [K_publisher.speed, K_publisher.speed]
            elif keyboard.is_pressed('s'): #right
                K_publisher.msgsent = [0, K_publisher.speed]
            elif keyboard.is_pressed('a'): 
                K_publisher.msgsent = [K_publisher.speed,0 ]
            elif keyboard.is_pressed('d'):  
                K_publisher.msgsent = [0,0]
            elif keyboard.is_pressed('space'):  # Stop / start
                K_publisher.msgsent = [0,0]
        
    except KeyboardInterrupt:
        pass

    K_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()