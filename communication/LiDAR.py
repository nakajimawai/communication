import rclpy
import math
import socket
import pickle
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 #Point Cloud2を読み取るためのモジュール
from ros2_whill.msg import BoolMultiArray
from std_msgs.msg import String
from sklearn.cluster import KMeans
import warnings
from std_msgs.msg import Int32, Float32

mode_flag = True # ユーザ操縦モードか介助者操縦モードかを保持するフラグ(True：ユーザ、False：介助者) 

class LiDAR_Subscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.sub = self.create_subscription(PointCloud2, 'rslidar_points', self.callback, 10)
        self.pub = self.create_publisher(BoolMultiArray, 'obstacle_info', 10)

        self.detection_result = BoolMultiArray() # 検知結果

        self.detection_range_sub = self.create_subscription(Float32, 'detection_range', self.detection_range_callback, 10) # 検知範囲用のサブスクライバ
        self.detection_range = Float32()
        self.detection_range.data = 1.0

        warnings.filterwarnings("ignore")# 警告を無効にする

    '''LiDARデータのコールバック関数'''
    def callback(self, lidar_msg):
        #self.start_time = time.time()  # コールバック開始時刻

        xyz_points = point_cloud2.read_points(lidar_msg,field_names=('x','y','z','intensity'))
        self.Filter_Points(xyz_points)

        self.detection_result.data = [False] * 10

        self.Detection()

    '''検知範囲のコールバック関数'''
    def detection_range_callback(self, range_msg):
        self.detection_range = range_msg
        print(f"検知範囲={self.detection_range.data}")

    '''必要なデータのみを取得、仕分けする関数'''
    def Filter_Points(self, xyz_points):
        self.left_diagonal_F_list = []  # 左斜め前　 のデータを格納するリスト
        self.F_list = []                # 前進方向　 のデータを格納するリスト
        self.right_diagonal_F_list = [] # 左斜め前　 のデータを格納するリスト
        self.cw_list = []               # cw旋回方向 のデータを格納するリスト
        self.ccw_list = []              # ccw旋回方向のデータを格納するリスト
        self.left_diagonal_B_list = []  # 左斜め後ろ のデータを格納するリスト（映像から見て）
        self.B_list = []                # 後退方向　 のデータを格納するリスト
        self.right_diagonal_B_list = [] # 左斜め後ろ のデータを格納するリスト（映像から見て）

        self.forward_list = []          # 前方向(-56°〜56°)　　 のデータを格納するリスト

        x = []
        y = []
        z = []
        for data in xyz_points:
            if -4.0 <= data[2] <= 0.0: # nan以外のデータを取り出す
                x = data[0]*1000  
                y = data[1]*1000
                z = data[2]*1000 + 1550 # 床の高さを0mmとする

                direction = math.degrees(math.atan2(y, x)) # 物体の方向算出

                x = x - 500 # 電動車いすに乗せた際のオフセット
                distance = math.sqrt((x**2)+(y**2)) # 物体まで、電動車いす中心からの距離になるよう算出
                x = x + 500 # xをもとの値に戻す

                if -56 < direction < 56:
                    if (1000 <= x) and (distance <= 1800): #ユーザ、PCを除く前方の距離データは専用のリストに格納、エッジ効果による外れ値除去のため
                        self.forward_list.append((x, y, z, distance, direction))

                else:
                    if distance <= 1800:
                        if (-74 <= direction < -54): # cw旋回方向
                            self.cw_list.append((x, y, z, distance)) # 格納

                        elif (54 <= direction < 74): # ccw旋回方向
                            self.ccw_list.append((x, y, z, distance)) # 格納  

                        elif (-162 <= direction < -126): # 映像から見て左斜め後ろ
                            self.left_diagonal_B_list.append((x, y, z, distance)) # 格納  
                            
                        elif (160 <= abs(direction) < 180): # 後退方向
                            self.B_list.append((x, y, z, distance)) # 格納  

                        elif (126 <= direction < 162): # 映像から見て右斜め後ろ
                            self.right_diagonal_B_list.append((x, y, z, distance)) # 格納  

    '''検知結果を更新する関数'''
    def Detection(self):
        if self.detection_range.data == 0.0:
            #print("検知範囲受信待機中")
            pass
        else:
            noise_removal_cnt_obstacle = [0] * 10
            noise_removal_cnt_lower_than_floor = [0] * 10

            if self.forward_list: # 前方のデータがある場合
                near_floor_list = []# 床の外れ値を除くためのクラスタリング用データリスト
                above_floor_list = [] # 高さのある物体のデータリスト
                lower_floor_list_forward = [] # 床より下のデータリスト

                for x, y, z, distance, direction in self.forward_list:
                    if (-40 <= z <= 100) and (-30 <= direction <= 30): # 床付近のデータ
                        near_floor_list.append((x, y, z, distance, direction))

                    elif 100 < z: # 高さのある物体のデータ
                        above_floor_list.append((x, y, z, distance, direction))

                    elif z < 0: #床より下のデータの場合
                        if (18 <= direction < 56): #左斜め前
                            if z < -160:
                                self.detection_result.data[0] = True # 下り階段があると判定

                        elif (-20 <= direction <= 20): # 前進方向
                            if z < -160:
                                self.detection_result.data[1] = True # 下り階段があると判定
                            else:
                                lower_floor_list_forward.append((x, y, z))

                        elif (-56 <= direction < -18): # 右斜め前
                            if z < -160:
                                self.detection_result.data[2] = True # 下り階段があると判定


                if 2 <= len(near_floor_list): # データ2つ以上無いとクラスタリングできない
                    filtered_obstacle_list = self.remove_outlier(near_floor_list) # エッジ効果で生じる外れ値を除去
                    
                    for x, y, z, distance, direction in filtered_obstacle_list:
                        if (18 <= direction < 54): #左斜め前に検知
                            self.left_diagonal_F_list.append((x, y, z, distance))

                        elif (-20 <= direction < 20): #前進方向に検知
                            self.F_list.append((x, y, z, distance))
                            
                        elif (-54 <= direction < -18): #右斜め前に検知
                            self.right_diagonal_F_list.append((x, y, z, distance))


                if above_floor_list: # 高さのある物体のデータがある場合
                    for x, y, z, distance, direction in above_floor_list:

                        if (18 <= direction < 54): #左斜め前に検知
                            self.left_diagonal_F_list.append((x, y, z, distance))

                        elif (-20 <= direction < 20): #前進方向に検知
                            self.F_list.append((x, y, z, distance))
                            
                            if distance <= 600:
                                self.left_diagonal_F_list.append((x, y, z, distance))
                                self.right_diagonal_F_list.append((x, y, z, distance))
                            
                        elif (-54 <= direction < -18): #右斜め前に検知
                            self.right_diagonal_F_list.append((x, y, z, distance))

                if lower_floor_list_forward: # 前進方向に床より下のデータがある場合
                    angle = self.angle_calculation_lower_floor_list(lower_floor_list_forward)
                    if 8.0 <= angle: # 8°以上なら下り階段とみなす
                        self.detection_result.data[1] = True 

            #----------------------------------------------------------------------------
            if not self.left_diagonal_F_list: # 左斜め前方向のデータを取得できてない場合
                self.detection_result.data[0] = True # 下り階段があると判定

            else:
                for x, y, z, distance in self.left_diagonal_F_list:
                    if (50 < z) and (distance < 900): # 床より上のデータの場合
                        noise_removal_cnt_obstacle[0] += 1
                        self.detection_result.data[0] = True
                        
            #----------------------------------------------------------------------------
            if not self.F_list: # 前進方向のデータを取得できてない場合
                self.detection_result.data[1] = True # 下り階段があると判定

            else:
                for x, y, z, distance in self.F_list:
                    if (50 < z) and (distance < 900): # 床より上のデータの場合
                        noise_removal_cnt_obstacle[1] += 1
                        self.detection_result.data[1] = True

            #----------------------------------------------------------------------------
            if not self.right_diagonal_F_list: # 右斜め前方向のデータを取得できてない場合
                self.detection_result.data[2] = True # 下り階段があると判定

            else:
                for x, y, z, distance in self.right_diagonal_F_list:
                    if (50 < z) and (distance < 900): # 床より上のデータの場合
                        noise_removal_cnt_obstacle[2] += 1
                        self.detection_result.data[2] = True

            #----------------------------------------------------------------------------
            if not self.cw_list: # cw方向のデータを取得できてない場合
                self.detection_result.data[3] = True # 下り階段があると判定

            else:
                for x, y, z, distance in self.cw_list:
                    if (50 < z) and (distance < 900): # 床より上のデータの場合
                        self.detection_result.data[3] = True # 障害物があると判定
                        if distance <= 600:
                            self.detection_result.data[7] = True # 左後輪側も検知
                        
                    elif z < -160: # 床より下のデータの場合
                        self.detection_result.data[3] = True # 下り階段があると判定
            #----------------------------------------------------------------------------

            if not self.ccw_list: # ccw方向のデータを取得できてない場合
                self.detection_result.data[9] = True # 下り階段があると判定

            else:
                for x, y, z, distance in self.ccw_list:
                    if (50 < z) and (distance < 900): # 床より上のデータの場合
                        self.detection_result.data[9] = True # 障害物があると判定
                        if distance <= 600:
                            self.detection_result.data[5] = True # 右後輪側も検知
                        
                    elif z < -160: # 床より下のデータの場合
                        self.detection_result.data[9] = True # 下り階段があると判定
            #----------------------------------------------------------------------------

            if not self.left_diagonal_B_list: # #映像から見て左斜め後移動方向のデータを取得できてない場合
                self.detection_result.data[5] = True # 下り階段があると判定

            else:
                for x, y, z, distance in self.left_diagonal_B_list:
                    if (50 < z) and (distance < 900): # 床より上のデータの場合
                        self.detection_result.data[5] = True # 障害物があると判定

                    elif z < -160: # 床より下のデータの場合
                        self.detection_result.data[5] = True # 下り階段があると判定
                        
            #----------------------------------------------------------------------------

            lower_floor_list_back = [] # 後ろ方向_床付近のデータリスト

            if not self.B_list: # 後方向のデータを取得できてない場合
                self.detection_result.data[6] = True # 下り階段があると判定

            else:
                for x, y, z, distance in self.B_list:
                    if (50 < z) and (distance < 900) and (abs(x) < 800): # 床より上のデータの場合
                        self.detection_result.data[6] = True # 障害物があると判定

                    elif (-160 <= z < -50): # 床付近のデータの場合
                        lower_floor_list_back.append((x, y, z))

                    elif z < -160: # 床より下のデータの場合
                        self.detection_result.data[6] = True # 下り階段があると判定
                        lower_floor_list_back = [] # スロープとの区別は行わない
                        break

            if lower_floor_list_back:# 後退方向に床より下のデータがある場合
                angle = self.angle_calculation_lower_floor_list(lower_floor_list_back)
                if 8.0 <= angle: # 8°以上なら下り階段とみなす
                    self.detection_result.data[6] = True 
                        
            #----------------------------------------------------------------------------

            if not self.right_diagonal_B_list: # #映像から見て右斜め後移動方向のデータを取得できてない場合
                self.detection_result.data[7] = True # 下り階段があると判定

            else:
                for x, y, z, distance in self.right_diagonal_B_list:
                    if (50 < z) and (distance < 900): # 床より上のデータの場合
                        self.detection_result.data[7] = True # 障害物があると判定

                    elif z < -160: # 床より下のデータの場合
                        self.detection_result.data[7] = True # 下り階段があると判定
                        
            #----------------------------------------------------------------------------

            #for i in range(3):
            #    if (noise_removal_cnt_obstacle[i] <= 3):   # データ数が少ない場合、ノイズとみなす
            #        self.detection_result.data[i] = False

            global mode_flag
            if mode_flag : # ユーザ操縦モードの場合、障害物情報をそのまま配信
                self.pub.publish(self.detection_result)
            else:          # 介助者操縦モードの場合、周囲に障害物がないことにして配信（衝突防止動作を無効に）
                self.detection_result.data = [False]*10
                self.pub.publish(self.detection_result)

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

    '''床より下データの角度算出'''
    def angle_calculation_lower_floor_list(self, lower_floor_list):
        angle_list = []
        for data in lower_floor_list:
            x = data[0] -750
            y = data[1]
            #z = abs(data[2] - 1550) # LiDARの高さを0mmとする
            z = abs(data[2])
            angle = math.degrees(math.atan2(z, x))
            angle_list.append(angle)

        ave_angle = sum(angle_list) / len(angle_list)

        print("角度：", ave_angle)
        print("データ数：", len(angle_list))

        return ave_angle


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
    