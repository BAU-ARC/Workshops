import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
from find_wall.action import OdomRecord
from rclpy.action import ActionClient
from find_wall.srv import FindWall
class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')
        self.scan = None
        request =  FindWall.Request()
        self.group = ReentrantCallbackGroup()
        self.findWallServ = self.create_client(FindWall,"find_wall")
        self.recordOdom = ActionClient(self,OdomRecord,"record_odom",callback_group=self.group)
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.get_laser,
            10,
            callback_group= self.group
        )
        self.move_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move = Twist()
        self.state = 0
        while not self.findWallServ.wait_for_service(timeout_sec=1.0):
           self.get_logger().info("Service not available, waiting...")
        self.findWall(request)
        
        
    def sendGoal(self):
        self.goal = OdomRecord.Goal()
        self.future_record = self.recordOdom.send_goal_async(self.goal,feedback_callback=self.feedback_callback)
        self.future_record.add_done_callback(self.checkFuture)



    def checkFuture(self,future):
        goal_handle = future.result()
        if(goal_handle.accepted):
            self.get_logger().info("Goal accepted")
            self.future_result = goal_handle.get_result_async()
            self.future_result.add_done_callback(self.get_result)

        else:
            self.get_logger().info("Goal not accepted, canceling call")
            return


    def get_result(self,future):
        result = future.result().result
        self.get_logger().info(f"Odometry recorded is: {result}")


    def feedback_callback(self,feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Current distance is: {feedback}")


    def findWall(self,request):
         self.future = self.findWallServ.call_async(request)


    def get_laser(self, msg):
        self.scan = msg.ranges
        self.follow_wall()

    def follow_wall(self):
        if(self.state == 0):
            if(self.future.done()):
                self.get_logger().info("Service finished")
                self.state = 1
                self.sendGoal()
            else:
                self.get_logger().info("Finding wall")

        elif(self.state ==1):
            self.move.angular.z= 0.0
            self.move.linear.x = 0.1
            minimum = 1
            for i in range (270,310):
                if(self.scan[i]<minimum):
                    minimum = self.scan[i]
            self.get_logger().info(f"Distance is : {minimum}")
            if(self.scan[0]< 0.5):
                self.get_logger().info("Wall ahead turn left")
                self.move.linear.x = 0.01
                self.move.angular.z = 0.3
            elif(minimum < 0.15):
                self.move.linear.x = 0.05
                self.move.angular.z = 0.1
                self.get_logger().info("Too close to the wall shifting left")
            elif(minimum > 0.2):
                self.move.linear.x = 0.05
                self.move.angular.z = -0.1
                self.get_logger().info("Too far from the wall shifting right")
            self.move_pub.publish(self.move)
            time.sleep(0.1)





def main(args=None):
    rclpy.init(args=args)
    follower_node = WallFollower()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(follower_node)
    executor.spin()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()