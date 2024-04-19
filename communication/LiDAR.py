import rclpy
import time
import csv
import math
import socket
import pickle
import struct
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 #Point Cloud2を読み取るためのモジュール
from ros2_whill.msg import BoolMultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sklearn.cluster import KMeans

class LiDAR_Subscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.sub = self.create_subscription(PointCloud2, 'rslidar_points', self.callback, 10)
        self.pub = self.create_publisher(BoolMultiArray, 'obstacle_info', 10)
        self.obstacle_info = BoolMultiArray()
            
    def callback(self, lidar_msg):
        xyz_points = point_cloud2.read_points(lidar_msg,field_names=('x','y','z','intensity'))
        self.Filter_Points(xyz_points)
        self.obstacle_info.data = [False]*12
        self.Obstacle_Detection()         
        
    def Filter_Points(self, xyz_points):
        self.obstacle_list = [] # 障害物の座標データを格納するリスト
        self.forward_obstacle_list = [] # 前方停止範囲における障害物の座標データを格納するリスト（クラスタリングするため別で用意）
        x = []
        y = []
        z = []
        for data in xyz_points:
            if -1.6 <= data[2] <= 0.0: # nan以外のデータを取り出す
                direction = math.degrees(math.atan2(data[1], data[0])) # 障害物の方向を算出
                x = data[0]*1000 - 500 #電動車いすに乗せた際のオフセット
                y = data[1]*1000
                z = data[2]*1000
                intensity = data[3]
                distance = math.sqrt((x**2)+(y**2))
                if distance <= 900:
                    if -20 < direction < 20:
                        self.forward_obstacle_list.append((x+500, y, z, intensity)) # 角度の算出をするのでxをもとの値に戻す
                    else:
                        if -1550 < z:
                            self.obstacle_list.append((x+500, y, z, intensity))

    '''障害物の方向を算出、通信用リストに追加'''
    def Obstacle_Detection(self):
        noise_removal_cnt = [0]*12

        if self.forward_obstacle_list:
            near_floor_list = []# 床の外れ値を除くためのクラスタリング用リスト
            for data in self.forward_obstacle_list:
                if -1600 <= data[2] <= -1450: # 床付近を監視
                    near_floor_list.append(data)

                    if 2 <= len(near_floor_list): #データが2つ以上ないとクラスタリングはできない
                        filtered_obstacle_list = self.remove_outlier(near_floor_list)

                        for data in filtered_obstacle_list:
                            if -1540 <= data[2]: #前方警告範囲に検出
                                noise_removal_cnt[1] += 1
                
                elif -1450 < data[2]: # 床より上を監視
                    if 1000 <= data[0]: # ユーザと視線入力用ノートPCの距離データは除く
                        noise_removal_cnt[1] += 1

        if self.obstacle_list:   # 障害物が検出された場合
            for x, y, z, intensity in self.obstacle_list:
                direction = math.degrees(math.atan2(y, x))
                if (2.0 < intensity):   # 反射強度の低い座標データを除去
                    '''前の監視'''
                    if (20 <= direction < 45): #前方警告範囲に検出
                        noise_removal_cnt[0]+=1
                    
                    if (-45 <= direction <= -20): #前方警告範囲に検出
                        noise_removal_cnt[2]+=1
                        
                    '''右の監視'''
                    if (-85 <= direction < -45): #右方向停止範囲に検出
                        noise_removal_cnt[3]+=1  
                          
                    if (-110 <= direction < -85): #右方向警告範囲に検出
                        noise_removal_cnt[4]+=1

                    if (-135 <= direction < -110): #右方向警告範囲に検出
                       noise_removal_cnt[5]+=1 

                    '''後ろの監視'''
                    if (-160 <= direction < -135): #後方警告範囲に検出
                        noise_removal_cnt[6]+=1 
                                                                                    
                    if (-180 <= direction <= -160) or (160 <= direction < 180):   #後方停止範囲に検出
                        noise_removal_cnt[7]+=1
                        
                    if (135 <= direction < 160): #後方警告範囲に検出
                        noise_removal_cnt[8]+=1
                            
                    '''左の監視'''
                    if (110 <= direction < 135): #左方向警告範囲に検出
                        noise_removal_cnt[9]+=1 

                    if (85 <= direction < 110): #左方向警告範囲に検出
                        noise_removal_cnt[10]+=1 
                            
                    if (45 <= direction < 85):   #左方向停止範囲に検出
                        noise_removal_cnt[11]+=1 
            
            for i in range(len(noise_removal_cnt)):
                if (20 <= noise_removal_cnt[i]):   # データ数が少ない場合、ノイズとみなす
                    self.obstacle_info.data[i] = True

            self.pub.publish(self.obstacle_info)

        else:
            #self.get_logger().info("No obstacles detected.")
            self.pub.publish(self.obstacle_info)
    '''k-平均法による外れ値除去'''
    def remove_outlier(self, near_floor_list):
        z = [data[2] for data in near_floor_list]

        Z = np.array(z).reshape(-1, 1)
        kmeans = KMeans(n_clusters=2, random_state=0, init='k-means++')
        kmeans.fit(Z)
        labels = kmeans.labels_

        # 暫定の正常値と外れ値にクラスタリング
        count_cluster0 = np.sum(labels == 0)
        count_cluster1 = np.sum(labels == 1)

        if count_cluster0 < count_cluster1:  # 要素数が少ない方を暫定の外れ値クラスタとする
            normal_cluster_label = 1
        else:
            normal_cluster_label = 0

        normal_indices = np.where(labels == normal_cluster_label)[0] # 正常値クラスタのインデックス

        # 正常値クラスタの値のみで平均値μと標準偏差σを求める
        normal_values = np.array(z)[normal_indices]
        mu = np.mean(normal_values)
        sigma = np.std(normal_values)

        # 平均値から±3σ離れているz座標を最終的な外れ値とする
        lower_bound = mu - 3 * sigma
        upper_bound = mu + 3 * sigma

        # 最終的な外れ値となったz座標を含むnear_floor_listの行は削除して新たなリストを取得する
        final_near_floor_list = [data for i, data in enumerate(near_floor_list) if (z[i] >= lower_bound and z[i] <= upper_bound)]

        return final_near_floor_list
    

'''障害物情報を購読してノートPC1に送信するノード'''         
class Send_Obstacle_Info(Node):
    def  __init__(self):
        super().__init__('send_obstacle_info')
        self.sub = self.create_subscription(BoolMultiArray, 'obstacle_info', self.obstacle_info_callback, 10)
        # 障害物情報を送信するサーバーソケットを作成
        server_address_obstacle = ('192.168.1.102', 50000)
        self.server_socket_o = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket_o.bind(server_address_obstacle)
        self.server_socket_o.listen(1)

    def obstacle_info_callback(self, obstacle_msg):
        try:
            self.client_socket_o, self.client_address_o = self.server_socket_o.accept()   # クライアントからの接続を待機
            #print("クライアントが接続しました:", self.client_address_o)     
            data_o = pickle.dumps(obstacle_msg.data)    # ブール値の配列をバイト列に変換
            self.client_socket_o.send(data_o) 
            self.client_socket_o.close()
        except :
            print("クライアントが接続を切断しました")
            self.client_socket_o.close()
            self.server_socket_o.close()
               

'''socket_topicの文字を購読して電動車いすの状態をノートPC1に送信するノード'''
class Send_State(Node):
    def  __init__(self):
        super().__init__('send_state_info')
        self.sub_socket = self.create_subscription(String, 'socket_topic', self.state_info_callback, 10)
        
        server_address_state = ('192.168.1.102', 50010)
        self.server_socket_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket_s.bind(server_address_state)
        self.server_socket_s.listen(1)
        
        self.state = False

    def state_info_callback(self, state_msg):
        print("状態を購読", state_msg)
        
        if state_msg.data == 'q':
            self.client_socket_s.close()
            self.server_socket_s.close()
            print("切断")
        elif state_msg.data != 'q':
            self.client_socket_s, self.client_address_s = self.server_socket_s.accept()   # クライアントからの接続を待機
            print("状態受信用クライアントが接続しました:", self.client_address_s) 
            if state_msg.data == 's':
                bool_byte = struct.pack('?', False)
                self.client_socket_s.send(bool_byte)
                print('Falseを送信')
                self.client_socket_s.close()
            else:
                bool_byte = struct.pack('?', True)
                self.client_socket_s.send(bool_byte)
                print('Trueを送信')
                self.client_socket_s.close()
        

def main():
    rclpy.init()
    lidar_sub_node = LiDAR_Subscriber()
    socket_obstacle_node = Send_Obstacle_Info()
    socket_state_node = Send_State()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(lidar_sub_node)
    executor.add_node(socket_obstacle_node)
    executor.add_node(socket_state_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        lidar_sub_node.destroy_node()
        socket_obstacle_node.destroy_node()
        socket_state_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
