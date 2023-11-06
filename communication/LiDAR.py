import rclpy
import time
import csv
import math
import socket
import struct
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 #Point Cloud2を読み取るためのモジュール
from ros2_whill.msg import BoolMultiArray

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

            print(self.obstacle_info)
            self.pub.publish(self.obstacle_info)

        else:
            #self.get_logger().info("No obstacles detected.")
            self.pub.publish(self.obstacle_info)
            
class Send_Obstacle_Info(Node):
    def  __init__(self):
        super().__init__('send_obstacle_info')
        self.sub = self.create_subscription(BoolMultiArray, 'obstacle_info', self.obstacle_info_callback, 10)
        
        self.HOST = '192.168.1.103'
        self.PORT = 50000
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
    '''障害物情報を購読してノートPC1に送信''' 
    '''
    def obstacle_info_callback(self, obstacle_msg):
        packed_data = struct.pack('b'*len(obstacle_msg.data), *obstacle_msg.data)
        self.client_socket.connect((self.HOST, self.PORT))
        print(packed_data)        
        self.client_socket.send(packed_data)
        self.client_socket.close()    
    '''
    
    def test(self):
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect(('192.168.1.103', 50000))
        data = "test"
        client.send(data.encode("utf-8"))
        buf = client.recv(4096)
        print(buf)
    
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
    executor.shutdown()
    lidar_sub_node.destroy_node()
    socket_obstacle_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
