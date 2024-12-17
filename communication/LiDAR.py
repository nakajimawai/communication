import rclpy
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
import warnings
from std_msgs.msg import Int32, Float32
import time

mode_flag = True # ユーザ操縦モードか介助者操縦モードかを保持するフラグ(True：ユーザ、False：介助者) 

class LiDAR_Subscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.sub = self.create_subscription(PointCloud2, 'rslidar_points', self.callback, 10)
        self.pub = self.create_publisher(BoolMultiArray, 'obstacle_info', 10)
        self.obstacle_info = BoolMultiArray()

        self.detection_range_sub = self.create_subscription(Float32, 'detection_range', self.detection_range_callback, 10) # 検知範囲用のサブスクライバ
        self.detection_range = Float32()
        self.detection_range.data = 0.0

        warnings.filterwarnings("ignore")# 警告を無効にする

        self.lidar_processing_times = [] # 処理時間を保存するリスト
            
    '''LiDARデータのコールバック関数'''
    def callback(self, lidar_msg):
        #self.start_time = time.time()  # コールバック開始時刻

        xyz_points = point_cloud2.read_points(lidar_msg,field_names=('x','y','z','intensity'))
        self.Filter_Points(xyz_points)
        self.obstacle_info.data = [False]*10
        self.Obstacle_Detection()

    '''検知範囲のコールバック関数'''
    def detection_range_callback(self, range_msg):
        self.detection_range = range_msg
        print(f"検知範囲={self.detection_range.data}")

        
    def Filter_Points(self, xyz_points):
        self.obstacle_list = [] # 障害物の座標データを格納するリスト
        self.forward_obstacle_list = [] # 前方停止範囲における障害物の座標データを格納するリスト（クラスタリングするため別で用意）
        x = []
        y = []
        z = []
        for data in xyz_points:
            if -1.6 <= data[2] <= 0.0: # nan以外のデータを取り出す
                x = data[0]*1000  
                y = data[1]*1000
                z = data[2]*1000 + 1550 # 床かの高さを0mmとする
                intensity = data[3]

                direction = math.degrees(math.atan2(y, x))
                x = x - 500 # 電動車いすに乗せた際のオフセット
                distance = math.sqrt((x**2)+(y**2))
                x = x + 500 # 角度の算出をするのでxをもとの値に戻す

                if distance <= 2700:
                    if (-54 < direction < 54) and (1100 <= x): # ユーザ、PCを除く前方の距離データは専用のリストに格納、エッジ効果による外れ値除去のため
                        self.forward_obstacle_list.append((x, y, z, intensity, distance)) 
                    else:
                        if 20 < z: 
                            self.obstacle_list.append((x, y, z, intensity, distance, direction))

    '''障害物の方向を算出、通信用リストに追加'''
    def Obstacle_Detection(self):
        if self.detection_range.data == 0.0:
            #print("検知範囲受信待機中")
            pass
        else:
            noise_removal_cnt = [0]*10

            '''前の監視'''
            if self.forward_obstacle_list:
                near_floor_list = []# 床の外れ値を除くためのクラスタリング用リスト
                above_floor_list = [] # 床より上    のデータ
                for data in self.forward_obstacle_list:
                    if (data[4] <= 900 * self.detection_range.data) and (1100 <= data[0]): # 監視範囲かつユーザとノートPC以外の場合
                        if -50 <= data[2] <= 100: # 床付近を監視
                            near_floor_list.append(data)

                        elif 100 < data[2]: # 床より上を監視
                            above_floor_list.append(data)

                if 2 <= len(near_floor_list): #データが2つ以上ないとクラスタリングはできない
                    filtered_obstacle_list = self.remove_outlier(near_floor_list) # エッジ効果で生じる外れ値を除去
                    for data in filtered_obstacle_list:
                        if 20 <= data[2]: # 検知物体が20mm以上の高さを持つ場合
                            direction= math.degrees(math.atan2(data[1], data[0]))

                            if (18 <= direction < 54): #左斜め前移動方向に検知
                                noise_removal_cnt[0]+=1
                                #print(data[0], data[1], data[2])

                            if (-20 <= direction < 20): #前進方向に検知
                                noise_removal_cnt[1]+=1
                                if data[4] <= 600:
                                    
                                    noise_removal_cnt[0]+=1 # 左斜め前移動中にぶつかる可能性があるため、同時に監視
                                    noise_removal_cnt[2]+=1 # 右斜め前移動中にぶつかる可能性があるため、同時に監視

                            if (-54 <= direction < -18): #右斜め前移動方向に検知
                                noise_removal_cnt[2]+=1

                if above_floor_list:# エッジ効果起きないとこで障害物が検知された場合
                    for data in above_floor_list:
                            direction= math.degrees(math.atan2(data[1], data[0]))

                            if (18 <= direction < 54): #左斜め前移動方向に検知
                                noise_removal_cnt[0]+=1
                                #print(data[0], data[1], data[2])

                            if (-20 <= direction < 20): #前進方向に検知
                                noise_removal_cnt[1]+=1
                                print(data[0], data[1], data[2], data[3])
                                if data[4] <= 600:
                                    #print(data[0], data[1], data[2], data[3])
                                    noise_removal_cnt[0]+=1 # 左斜め前移動中にぶつかる可能性があるため、同時に監視
                                    noise_removal_cnt[2]+=1 # 右斜め前移動中にぶつかる可能性があるため、同時に監視

                            if (-54 <= direction < -18): #右斜め前移動方向に検知
                                noise_removal_cnt[2]+=1

            for i in range(3):
                if (3 <= noise_removal_cnt[i]):   # データ数が少ない場合、ノイズとみなす
                    if i == 1:
                        angle = self.angle_calculation(self.forward_obstacle_list) # 検知した物体の傾斜角度を算出
                        if 8.0 <= angle:
                            self.obstacle_info.data[1] = True # 角度が8°以上の場合、スロープではなく障害物とみなす

                    else:
                        self.obstacle_info.data[i] = True

            if self.obstacle_list:   # 障害物が検出された場合
                XYZ_list = []
                for x, y, z, intensity, distance, direction in self.obstacle_list:
                    if (160 <= abs(direction) <= 180): # 後退方向も一応スロープの識別を行うために個別で監視
                        XYZ_list.append((x, y, z))
                        if (abs(x) <= 800): # 他の範囲より広めに監視
                            self.obstacle_info.data[6] = True
                        #'''
                        if (abs(x) <= 600): # 左斜め後移動中にぶつかる可能性があるので同時に監視
                            self.obstacle_info.data[5] = True 
                        if (abs(x) <= 600): # 右斜め後移動中にぶつかる可能性があるので同時に監視
                            self.obstacle_info.data[7] = True     
                        #'''                      
                    if (-162 <= direction < -126): #映像から見て左斜め後移動方向に検出
                        if (abs(x) <= 600): # 他の範囲より広めに監視
                            #noise_removal_cnt[5]+=1 
                            self.obstacle_info.data[5] = True
                                    
                        
                    if (126 <= direction < 162): #映像から見て右斜め後移動方向に検出
                        if (abs(x) <= 600): # 他の範囲より広めに監視
                            #noise_removal_cnt[8]+=1
                            self.obstacle_info.data[7] = True
        
                    else:
                        if (distance <= 900 * self.detection_range.data):
                            if (-74 <= direction < -54): #前方走行時のcw旋回方向に検出
                                #noise_removal_cnt[3]+=1  
                                self.obstacle_info.data[3] = True
                                if distance <= 600:
                                    self.obstacle_info.data[7] = True
                            '''
                            if (-126 <= direction < -90): #後方走行時のccw旋回方向に検出
                                #noise_removal_cnt[4]+=1
                                self.obstacle_info.data[9] = True
                                    
                            if (90 <= direction < 126): #後方走行時のcw旋回方向に検出
                                #noise_removal_cnt[9]+=1
                                self.obstacle_info.data[3] = True 
                            '''
                            if (54 <= direction < 74): #前方走行時のccw旋回方向に検出
                                #noise_removal_cnt[10]+=1 
                                self.obstacle_info.data[9] = True
                                if distance <= 600:
                                    self.obstacle_info.data[5] = True

                if self.obstacle_info.data[6]:
                    angle = self.angle_calculation(XYZ_list) # 検知した物体の傾斜角度を算出
                    if angle < 8.0: # 8° 下回るなら障害物でなくスロープとみなす
                        self.obstacle_info.data[6] = False

                global mode_flag
                if mode_flag : # ユーザ操縦モードの場合、障害物情報をそのまま配信
                    self.pub.publish(self.obstacle_info)
                else:          # 介助者操縦モードの場合、周囲に障害物がないことにして配信（衝突防止動作を無効に）
                    self.obstacle_info.data = [False]*10
                    self.pub.publish(self.obstacle_info)

            else:
                #self.get_logger().info("No obstacles detected.")
                self.pub.publish(self.obstacle_info)

            '''
            self.end_time = time.time() # 障害物検知終了時刻
            processing_time = (self.end_time - self.start_time) * 1000 # ミリ秒に変換
            self.lidar_processing_times.append(processing_time) # 処理時間をリストに追加

            # 100回分データが集まったら平均を算出しファイルに出力
            if len(self.lidar_processing_times) == 100:
                self.caluculate_average_and_save("/home/ubuntu/dev_ws/src/communication/communication/r=1500mm_detection_time.txt", self.lidar_processing_times)
            '''
                
    def caluculate_average_and_save(self, filename, times_list):
        average_time  = sum(times_list) / len(times_list) # 平均値計算

        #テキストファイルにデータを保存
        with open(filename, 'w') as f:
            f.write("処理時間（ms）:\n")
            for time_value in times_list:
                f.write(f"{time_value:.3f}\n")
            f.write(f"\n平均処理時間: {average_time:.3f} ms\n")
        self.get_logger().info(f"{filename}にデータを保存")

    '''k-平均法による外れ値除去（床）'''
    def remove_outlier(self, near_floor_list):
        z = [data[2] for data in near_floor_list]

        Z = np.array(z).reshape(-1, 1)
        kmeans = KMeans(n_clusters=2, random_state=0, init='k-means++', verbose=0)
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
    
    '''傾斜角度の算出'''
    def angle_calculation(self, obstacle_list):
        angle_list = []
        for data in obstacle_list:
            x = abs(data[0]) - 1000 # 傾斜角度算出のためのオフセット
            y = data[1]
            z = data[2]
            if (abs(y) <= 400): # 幅800mm以内のデータで角度算出 
                angle = math.degrees(math.atan2(z, x))
                angle_list.append(angle)
        if 2 <= len(angle_list): # データは2個以上ないとクラスタリングできない
            filtered_angle_list = self.remove_angle_outlier(angle_list) # 傾斜角度の外れ値除去

            filtered_angle_ave = sum(filtered_angle_list) / len(filtered_angle_list) 

            return filtered_angle_ave
        else: # 算出した傾斜角度が1つならそのまま返す
            return angle

    '''k-平均法による外れ値除去（傾斜角度）'''
    def remove_angle_outlier(self, angle_list):

        ANGLE = np.array(angle_list).reshape(-1, 1)
        kmeans = KMeans(n_clusters=2, random_state=0, init='k-means++', verbose=0)
        kmeans.fit(ANGLE)
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
        normal_values = np.array(angle_list)[normal_indices]
        mu = np.mean(normal_values)
        sigma = np.std(normal_values)

        # 平均値から±3σ離れているz座標を最終的な外れ値とする
        lower_bound = mu - 2 * sigma
        upper_bound = mu + 2 * sigma

        # 最終的な外れ値となったz座標を含むnear_floor_listの行は削除して新たなリストを取得する
        final_near_floor_list = [data for i, data in enumerate(angle_list) if (angle_list[i] >= lower_bound and angle_list[i] <= upper_bound)]

        return final_near_floor_list


'''障害物情報を購読してノートPC1に送信するノード'''         
class Send_Obstacle_Info(Node):
    def  __init__(self):
        super().__init__('send_obstacle_info')
        self.sub = self.create_subscription(BoolMultiArray, 'obstacle_info', self.obstacle_info_callback, 10) 

        self.pub_led_color = self.create_publisher(Int32, 'led_color', 10) # パトライト用パブリッシャ
        self.led_msg = Int32()

        # 障害物情報を送信するサーバーソケットを作成
        server_address_obstacle = ('192.168.1.102', 50000)
        self.server_socket_o = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket_o.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # TIME WAIT状態にならないよう設定
        self.server_socket_o.bind(server_address_obstacle)
        self.server_socket_o.listen(1)

        self.cnt = 0

    def obstacle_info_callback(self, obstacle_msg):
        try:
            if self.cnt <= 10: # 開始から1秒は無条件で障害物情報を送信
                self.client_socket_o, self.client_address_o = self.server_socket_o.accept()   # クライアントからの接続を待機
                data_o = pickle.dumps(obstacle_msg.data)    # ブール値の配列をバイト列に変換
                self.client_socket_o.send(data_o) 
                self.client_socket_o.close()

                self.cnt += 1
                self.str_obstacle_info = obstacle_msg # 前回の障害物情報を保持
            else:
                for i, bool in enumerate(obstacle_msg.data):
                    if bool != self.str_obstacle_info.data[i]: # 前回と障害物情報がどこかしら変わってたら送信
                        self.client_socket_o, self.client_address_o = self.server_socket_o.accept()   # クライアントからの接続を待機
                        data_o = pickle.dumps(obstacle_msg.data)    # ブール値の配列をバイト列に変換
                        self.client_socket_o.send(data_o) 
                        self.client_socket_o.close()
                        
                        break
                    else:
                        continue

                self.str_obstacle_info = obstacle_msg
        except Exception as e:
            print(e)
            print("クライアントが接続を切断もしくは、障害物情報を送信するサーバでエラー")
            self.client_socket_o.close()
            self.server_socket_o.close()

            self.led_msg.data = 1
            self.pub_led_color.publish(self.led_msg) # LEDを赤点灯
            exit()
        #'''
               

'''socket_topicの文字を購読して電動車いすの状態をノートPC1に送信するノード'''
class Send_State(Node):
    def  __init__(self):
        super().__init__('send_state_info')
        self.sub_socket = self.create_subscription(String, 'socket_topic', self.state_info_callback, 10)
        
        server_address_state = ('192.168.1.102', 50010)
        self.server_socket_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket_s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # TIME WAIT状態にならないよう設定
        self.server_socket_s.bind(server_address_state)
        self.server_socket_s.listen(1)
        
        self.state = 'stop'

    def state_info_callback(self, state_msg):
        print("状態を購読", state_msg)
        
        global mode_flag

        if state_msg.data == 'f':
            self.client_socket_s.close()
            self.server_socket_s.close()
            print("切断")
            exit()

        elif state_msg.data == 'helper':
            print("介助者操縦モード、衝突防止動作を無効に")
            mode_flag = False

        elif state_msg.data == 'user':
            print("ユーザ操縦モード、衝突防止動作を有効に")
            mode_flag = True

        elif state_msg.data == 'EG_stop':
            self.get_logger().info('緊急停止！！！！！！！！！！！！！')
            self.state = state_msg.data
            self.client_socket_s, self.client_address_s = self.server_socket_s.accept()   # クライアントからの接続を待機
            self.client_socket_s.send(self.state.encode('utf-8'))
            self.client_socket_s.close()

        else:
            self.client_socket_s, self.client_address_s = self.server_socket_s.accept()   # クライアントからの接続を待機

            if state_msg.data == 's' or state_msg.data == 'EG_stop_R':
                self.state = 'stop'
                self.client_socket_s.send(self.state.encode('utf-8'))
                self.get_logger().info('stopを送信')
                self.client_socket_s.close()

            else:
                self.state = 'move'
                self.client_socket_s.send(self.state.encode("utf-8"))
                self.get_logger().info('moveを送信')
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
    