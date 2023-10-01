import rclpy
import socket
from rclpy.node import Node
from std_msgs.msg import String

class socket_publisher(Node):

    def __init__(self):
        super().__init__('socket_pubulisher')
        self.pub_socket = self.create_publisher(String, 'socket_topic', 10)
        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        host_ip = '192.168.1.102'
        host_port = 12345

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.server_socket.bind((host_ip, host_port))
        
    def timer_callback(self):
        msg = String()
        self.server_socket.listen(1)
        print("待機中")
    
        client_socket, client_address = self.server_socket.accept()

        data = client_socket.recv(1024)
        
        msg.data = data.decode()
        
        print("受信メッセージ：", msg.data)
        
        self.pub_socket.publish(msg)
        

def main():
    rclpy.init()
    socket_node = socket_publisher()
    rclpy.spin(socket_node)
    socket_node.destroy_node()
    socket_publisher.client_socket.close()
    socket_publisher.server_socket.close()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
