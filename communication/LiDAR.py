import rclpy
import time
import csv
import math
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 #used to read points 
from ros2_whill.msg import BoolMultiArray
n=0

class lidar_Subscriber(Node):

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
        
        '''
        global n
        data_len = self.Filter_Points(xyz_points)
        with open('output2.csv', mode='a', newline='') as file:   
            writer = csv.writer(file)
            writer.writerow(data_len)
            n+=1
            if n== 100:
                file.close()
                print("クローズ")
                time.sleep(5)   
        '''             
        
        
    def Filter_Points(self, xyz_points):
        self.obstacle = []
        #print(xyz_points)
        for data in xyz_points:
            if data[2] == 0:   # z=0のデータのみを取り出す
                x = data[0] - 0.5
                y = data[1] 
                if math.sqrt((x**2)+(y**2)) <= 0.9:   # 
                    self.obstacle.append((data[0], data[1]))              

    '''障害物の方向を特定'''                    
    def Obstacle_Detection(self):
        if self.obstacle:   # 障害物が検出された場合
        
            for x, y in self.obstacle:
                direction = math.degrees(math.atan2(y, x))
                #self.get_logger().info(f"Obstacle detected at direction: {direction}°")
                if (0 <= direction <=45) or (-45 <= direction <0):   #前方に検出した場合
                    #y = y - 0.5   #電動車いすに載せた際のオフセット
                    #if math.sqrt((x**2)+(y**2)) <= 0.9:
                        self.obstacle_info.data[0] = True 

                elif (135 <= direction <=180) or (-180 < direction <=-135):   #後方に検出した場合
                    #y = y - 0.9   #電動車いすに載せた際のオフセット
                    #if math.sqrt((x**2)+(y**2)) <= 0.4:
                        self.obstacle_info.data[3] = True
                
                elif (45 < direction <135):   #左方向に検出した場合
                    #y = y - 0.5   #電動車いすに載せた際のオフセット
                    #if math.sqrt((x**2)+(y**2)) <= 1.2:
                        self.obstacle_info.data[2] = True 

                elif (-135 < direction <-45):   #右方向に検出した場合
                    #y = y - 0.5   #電動車いすに載せた際のオフセット
                    #if math.sqrt((x**2)+(y**2)) <= 1.2:
                        self.obstacle_info.data[1] = True 

            print(self.obstacle_info)
            self.pub.publish(self.obstacle_info)

        else:
            #self.get_logger().info("No obstacles detected.")
            self.pub.publish(self.obstacle_info)   

        '''
        self.filtered_points = [point for point in xyz_points if point[2] == 0.0]# z座標が0の点を選別
       
        for xy in filtered_points:
            print(f"X: {xy[0]}, Y:{xy[1]}, Z: {xy[2]}, INTENSITY: {xy[3]}")
            
            
        data = [len(filtered_points)]

        return data
        '''
            
    
def main():
    rclpy.init()
    lidar_sub_node = lidar_Subscriber()
    try:
        rclpy.spin(lidar_sub_node)
    except KeyboardInterrupt:
        pass
    lidar_sub_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
