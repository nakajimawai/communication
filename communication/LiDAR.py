import rclpy
import time
import csv
import math
import socket
import pickle
import struct
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 #Point Cloud2を読み取るためのモジュール
from ros2_whill.msg import BoolMultiArray
from std_msgs.msg import String

class LiDAR_Subscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.sub = self.create_subscription(PointCloud2, 'rslidar_points', self.callback, 10)
        self.pub = self.create_publisher(BoolMultiArray, 'obstacle_info', 10)
        self.obstacle_info = BoolMultiArray()
            
    def callback(self, lidar_msg):
        xyz_points = point_cloud2.read_points(lidar_msg,field_names=('x','y','z','intensity'))
        self.Filter_Points(xyz_points)
        self.obstacle_info.data = [False, False, False, False]
        self.Obstacle_Detection()         
        
    def Filter_Points(self, xyz_points):
        self.obstacle = []
        #print(xyz_points)
        for data in xyz_points:
            if data[2] == 0:   # z=0のデータのみを取り出す
                x = data[0] - 0.5   #電動車いすに載せた際のオフセット
                y = data[1] 
                if math.sqrt((x**2)+(y**2)) <= 0.9:   # 
                    self.obstacle.append((data[0], data[1]))              

    '''障害物の方向を特定'''                    
    def Obstacle_Detection(self):
        if self.obstacle:   # 障害物が検出された場合
        
            for x, y in self.obstacle:
                direction = math.degrees(math.atan2(y, x))
                if (0 <= direction <=45) or (-45 <= direction <0):   #前方に検出した場合
                        self.obstacle_info.data[0] = True 

                elif (135 <= direction <=180) or (-180 < direction <=-135):   #後方に検出した場合
                        self.obstacle_info.data[3] = True
                
                elif (45 < direction <135):   #左方向に検出した場合
                        self.obstacle_info.data[2] = True 

                elif (-135 < direction <-45):   #右方向に検出した場合
                        self.obstacle_info.data[1] = True 

            #print(self.obstacle_info)
            self.pub.publish(self.obstacle_info)

        else:
            #self.get_logger().info("No obstacles detected.")
            self.pub.publish(self.obstacle_info)
            
class Send_Obstacle_Info(Node):
    def  __init__(self):
        super().__init__('send_obstacle_info')
        self.sub = self.create_subscription(BoolMultiArray, 'obstacle_info', self.obstacle_info_callback, 10)
        self.sub_socket = self.create_subscription(String, 'socket_topic', self.state_info_callback, 10)
        
        # 障害物情報を送信するサーバーソケットを作成
        server_address_obstacle = ('192.168.1.102', 50000)
        self.server_socket_o = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket_o.bind(server_address_obstacle)
        self.server_socket_o.listen(1)
        
        # 電動車いすの状態を送信するサーバーソケットを作成
        server_address_state = ('192.168.1.102', 50010)
        self.server_socket_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket_s.bind(server_address_state)
        self.server_socket_s.listen(1)
        
        #self.state = False
        
    '''障害物情報を購読してノートPC1に送信''' 
    def obstacle_info_callback(self, obstacle_msg):
        try:
            self.client_socket_o, self.client_address_o = self.server_socket_o.accept()   # クライアントからの接続を待機
            #print("クライアントが接続しました:", self.client_address_o)     
            data_o = pickle.dumps(obstacle_msg.data)    # ブール値の配列をバイト列に変換
            self.client_socket_o.send(data_o) 
            self.client_socket_o.close()
        except:
            print("クライアントが接続を切断しました")
            self.client_socket_o.close()
            self.server_socket_o.close()
               
    '''socket_topicの文字を購読して電動車いすの状態をノートPC1に送信'''            
    def state_info_callback(self, state_msg):
        if state_msg.data == 'q':
            self.client_socket_o.close()
            self.server_socket_o.close()
            self.client_socket_s.close()
            self.server_socket_s.close()
        else:
            self.client_socket_s, self.client_address_s = self.server_socket_s.accept()   # クライアントからの接続を待機
            print("クライアントが接続しました:", self.client_address_s) 
            print("購読文字：", state_msg.data)
            if state_msg.data == 's':
                bool_byte = struct.pack('?', False)
                self.client_socket_s.send(bool_byte)
                self.client_socket_s.close()
            else:
                bool_byte = struct.pack('?', True)
                self.client_socket_s.send(bool_byte)
                self.client_socket_s.close()
    
def main():
    rclpy.init()
    lidar_sub_node = LiDAR_Subscriber()
    socket_obstacle_node = Send_Obstacle_Info()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(lidar_sub_node)
    executor.add_node(socket_obstacle_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        lidar_sub_node.destroy_node()
        socket_obstacle_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
