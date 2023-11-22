import rclpy
import socket
import threading
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ros2_whill.msg import BoolMultiArray

#str_msg = 's'   #電動車いすを停止させるか判断するための変数

'''ソケット通信で受け取った文字を配信するノード'''
class Socket_Publisher(Node):

    def __init__(self):
        super().__init__('socket_publisher')
        self.pub_socket = self.create_publisher(String, 'socket_topic', 10)
        timer_socket_period = 0.1 #seconds
        self.timer_socket = self.create_timer(timer_socket_period, self.timer_socket_callback)
        
        host_ip = '192.168.1.102'
        host_port = 12345

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.server_socket.bind((host_ip, host_port))
        
    def timer_socket_callback(self):
        msg = String()
        self.server_socket.listen(1)
        print("待機中")
    
        client_socket, client_address = self.server_socket.accept()

        data = client_socket.recv(1024)
        
        msg.data = data.decode()
        
        print("受信メッセージ：", msg.data)
        
        self.pub_socket.publish(msg)
        
        if msg.data == 'q':
            client_socket.close()
            self.server_socket.close()
            self.timer_socket.cancel()

'''配信してある文字に基づいて走行制御を行うノード'''        
class Socket_Subscriber(Node):
    
    def __init__(self):
        super().__init__('socket_subscriber')
        self.sub_socket = self.create_subscription(String, 'socket_topic', self.callback, 10)
        self.pub_velocity = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        
        timer_velocity_period = 0.01 #seconds
        self.timer_velocity = self.create_timer(timer_velocity_period, self.timer_velocity_callback)
        
    def callback(self, msg):
        #str_msg = msg.data
        
        if msg.data == 'w':
            self.twist.linear.x = 1.6
            self.twist.angular.z = 0.0
        elif msg.data == 'x':
            self.twist.linear.x = -4.6
            self.twist.angular.z = 0.0
        elif msg.data == 'a':
            self.twist.linear.x = 0.0
            self.twist.angular.z = 1.1
        elif msg.data == 'd':
            self.twist.linear.x = 0.0
            self.twist.angular.z = -1.1
        elif msg.data == 'q':
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            exit()
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            
    def timer_velocity_callback(self):
        self.pub_velocity.publish(self.twist)
        
'''LiDARからの障害物情報に基づいて衝突防止動作を行うノード'''  
class Obstacle_Info_Subscriber(Node):
    global str_msg
    
    def __init__(self):
        super().__init__('obstacle_info_subscriber')
        #障害物がどこにあるかを購読する用
        self.sub_obstacle_info = self.create_subscription(BoolMultiArray, 'obstacle_info', self.callback_lidar, 10)
        
        #電動車いすを停止させるか判断するため文字を購読する用
        self.sub_socket_stop = self.create_subscription(String, 'socket_topic', self.callback_stop_judge, 10)
        
        #電動車いすを停止させるため文字を配信する用
        self.pub_socket_stop = self.create_publisher(String, 'socket_topic', 10)
        
        self.msg_stop = String()   
        self.msg_stop.data = 's'   #電動車いすを停止させるための文字
        
        self.msg_str = String()   #電動車いすを停止させるか判断するための文字
        
    def callback_stop_judge(self, msg):   
        self.msg_str = msg
         
    def callback_lidar(self, obstacle_info):
        #↓移動方向に障害物がある場合は停止
        #print(obstacle_info)
        if (obstacle_info.data[1]) and (self.msg_str.data == 'w'):
            self.pub_socket_stop.publish(self.msg_stop)
        elif(obstacle_info.data[3]) and (self.msg_str.data == 'd'):
            self.pub_socket_stop.publish(self.msg_stop)
        elif(obstacle_info.data[11]) and (self.msg_str.data == 'a'):
            self.pub_socket_stop.publish(self.msg_stop)
        elif(obstacle_info.data[7]) and (self.msg_str.data == 'x'):
            self.pub_socket_stop.publish(self.msg_stop)
                    
def main():
    rclpy.init()
    socket_node = Socket_Publisher()
    velocity_node = Socket_Subscriber()
    obstacl_info_sub_node = Obstacle_Info_Subscriber()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(socket_node)
    executor.add_node(velocity_node)
    executor.add_node(obstacl_info_sub_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    socket_node.destroy_node()
    velocity_node.destroy_node()
    obstacl_info_sub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
