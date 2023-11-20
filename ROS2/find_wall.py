from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from find_wall.srv import FindWall
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
from geometry_msgs.msg import Quaternion
class wallFinderServer(Node):
    def __init__(self):
        super().__init__("wall_finder_service")
        self.group1 =  MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()
        self.move = Twist() 
        self.done = False
        self.index = 0
        self.server = self.create_service(srv_name= "find_wall", srv_type=FindWall,callback= self.wall_finder,callback_group= self.group3)
        self.odomSub = self.create_subscription(
                            Odometry,
                            "odom",
                            self.findOdom,
                            10,
                            callback_group=self.group1
                        )

        self.laserSub = self.create_subscription(
                                LaserScan,
                                "scan",
                                self.findLaser,
                                10,
                                callback_group=self.group2
                            )
        self.movePub = self.create_publisher(Twist,"cmd_vel",10)
        self.state = 0
        self.called = False

    def findLaser(self,msg):
        self.scan = msg.ranges

        for i in range(0,360):
            if(self.scan[i] == min(self.scan)):
                self.index = i
        
        
            
       

    def findOdom(self,msg):
        self.odom =  msg
        self.orientation = self.quaternion_to_euler(msg.pose.pose.orientation)
        
        
        
    
    def wall_finder(self,request,response):
        self.get_logger().info("Server called")
        while(not self.done):
            self.movePub.publish(self.move)
            if(self.state == 0):
                self.move.angular.z = 0.2
                if(self.index in range(355,360) or self.index in range(0,5)):
                    self.get_logger().info("State one finished")
                    self.state = 1       
            elif(self.state == 1):
                self.get_logger().info("Inside state two")
                self.move.angular.z = 0.0
                self.move.linear.x = 0.1
                if(self.scan[0] < 0.2):
                    self.get_logger().info("Inside short")
                    self.state = 2
            elif(self.state == 2):
                self.get_logger().info("Inside state Three")
                self.move.angular.z = 0.2
                self.move.linear.x = 0.0
                if(self.orientation[2]>3.10):
                    self.move.angular.z = 0.0
                    self.move.linear.x = 0.0
                    self.done= True
                    self.movePub.publish(self.move)

            
            
        response.wallfound = True
        return response

    def quaternion_to_euler(self,quaternion: Quaternion):
            import math
            qx = quaternion.x
            qy = quaternion.y
            qz = quaternion.z
            qw = quaternion.w

            roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
            pitch = math.asin(2.0 * (qw * qy - qx * qz))
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

            return [roll, pitch, yaw]
        

def main(args=None):
    rclpy.init(args=args)
    follower_node = wallFinderServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(follower_node)
    executor.spin()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()   

