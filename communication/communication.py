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
        
        if msg.data == 'f':
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

        self.config = self.read_config('/home/ubuntu/dev_ws/src/communication/communication/config.txt') # 設定速度を読み込む
        self.straight_V = float(self.config['直進速度'])
        self.rotation_V = float(self.config['旋回速度'])
        self.diagonal_V = float(self.config['斜め移動速度'])

    '''電動車いす速度のコンフィグレーションファイルを読み込む関数'''
    def read_config(self, filepath):
        config = {}
        with open(filepath, 'r') as file:
            for line in file:
                name, value = line.strip().split('=')
                config[name] = value
        print(config)
        return config

    '''購読した走行指令文字に基づいて速度設定して制御を行う関数'''
    def callback(self, msg):
        #str_msg = msg.data
        
        if msg.data == 'w': # 前進
            self.twist.linear.x = 1.5 * self.straight_V
            self.twist.angular.z = 0.0
        elif msg.data == 'x': # 後退
            self.twist.linear.x = -3.8 * self.straight_V #-4.6
            self.twist.angular.z = 0.0
        elif msg.data == 'q': # 左斜め前移動
            self.twist.linear.x = 0.7 * self.diagonal_V
            self.twist.angular.z = 1.1 * self.diagonal_V
        elif msg.data == 'e': # 右斜め前移動
            self.twist.linear.x = 0.7 * self.diagonal_V
            self.twist.angular.z = -1.1 * self.diagonal_V
        elif msg.data == 'c': # 映像から見て左斜め後移動
            self.twist.linear.x = -1.75 * self.diagonal_V
            self.twist.angular.z = 1.1 * self.diagonal_V
        elif msg.data == 'z': # 映像から見て右斜め後移動
            self.twist.linear.x = -1.75 * self.diagonal_V
            self.twist.angular.z = -1.1 * self.diagonal_V
        elif msg.data == 'a' or msg.data == 'b_a': # ccw旋回
            self.twist.linear.x = 0.0
            self.twist.angular.z = 1.1 * self.rotation_V
        elif msg.data == 'd' or msg.data == 'b_d': # cw旋回
            self.twist.linear.x = 0.0
            self.twist.angular.z = -1.1 * self.rotation_V
        elif msg.data == 'f': # 停止とプログラム終了
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            exit()
        else: # 停止
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
        if (obstacle_info.data[0]) and (self.msg_str.data == 'q'): # 左斜め前
            self.pub_socket_stop.publish(self.msg_stop)        
        elif (obstacle_info.data[1]) and (self.msg_str.data == 'w'): 
            self.pub_socket_stop.publish(self.msg_stop)
        elif (obstacle_info.data[2]) and (self.msg_str.data == 'e'): # 右斜め前
            self.pub_socket_stop.publish(self.msg_stop)
        elif (obstacle_info.data[3]) and (self.msg_str.data == 'd'):
            self.pub_socket_stop.publish(self.msg_stop)
        elif (obstacle_info.data[9]) and (self.msg_str.data == 'a'):
            self.pub_socket_stop.publish(self.msg_stop)
        elif (obstacle_info.data[5]) and (self.msg_str.data == 'c'): # 映像から見て左斜め後
            self.pub_socket_stop.publish(self.msg_stop)
        elif (obstacle_info.data[6]) and (self.msg_str.data == 'x'):
            self.pub_socket_stop.publish(self.msg_stop)
        elif (obstacle_info.data[7]) and (self.msg_str.data == 'z'): # 映像から見て右斜め後
            self.pub_socket_stop.publish(self.msg_stop)
        elif (obstacle_info.data[8]) and (self.msg_str.data == 'b_d'):
            self.pub_socket_stop.publish(self.msg_stop)
        elif (obstacle_info.data[4]) and (self.msg_str.data == 'b_a'):
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
