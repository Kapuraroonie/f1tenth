import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sig_stack.utils import yaw_from_quaternion

class Control(Node):
    def __init__(self):
        super().__init__('control')

        # subscribes to odom published from tech stack
        self.subscription_dest = self.create_subscription(Odometry, 'drive_to', self.drive_to_callback, 10)
        self.subscription_pos = self.create_subscription(Odometry, 'odom', self.pose_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        #current position
        self.currX = None
        self.currY = None
        #current yaw or orientation
        self.currYaw = None
        #current velocity of speed
        self.currV = None

    # gets the new yaw for the car- driving to waypoint
    def get_turn_angle(self, x, y):
        # x is destination x, y is destination y
        #this is the new direction- but nothing to do with how much we have to turn to get there
        target = math.atan2(y - self.currY, x - self.currX)
        #this calcuates how much it takes to turn there. We dont want to turn the long way around
        error = target - self.currYaw                                                                                                                 
        return (error + math.pi) % (2 * math.pi) - math.pi

    def pose_callback(self, msg):
        #setting all current variables
        self.currX = msg.pose.pose.position.x
        self.currY = msg.pose.pose.position.y
        self.currYaw = yaw_from_quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        self.currV = msg.twist.twist.linear.x

    
    #this fucntion should work at an interval. This means that every waypoint should be a time t apart. 
    # NOTE: not yet complete — global line does not include velocity, so target_v is a placeholder
    def get_new_speed(self, target_v):
        return target_v

    def drive_to_callback(self, msg):
        #set destenation state varaibles
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = msg.twist.twist.linear.x

        #functions to find and set the yaw and speed the car should be at to reach the next waypoint
        new_yaw = self.get_turn_angle(x, y)
        new_speed = self.get_new_speed(v)

        msg = AckermannDriveStamped()
        msg.drive.speed = new_speed
        msg.drive.steering_angle = new_yaw

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    control = Control()
    rclpy.spin(control)
    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()