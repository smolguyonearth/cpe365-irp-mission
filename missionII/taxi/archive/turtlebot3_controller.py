# Authors : Nopphakorn Subs. Niwatchai Wang. Supasate Wor. Narith Tha. Tawan Thaep. Napatharak Muan.
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math

class Turtlebot3Controller(Node):

    def __init__(self):
        super().__init__('turtlebot3_controller')   # node name
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.scanSubscriber = self.create_subscription(LaserScan, 'scan', self.scanCallback, qos_profile=qos_profile_sensor_data)
        self.batteryStateSubscriber = self.create_subscription(BatteryState, 'battery_state', self.batteryStateCallback, 1)
        self.odomSubscriber = self.create_subscription(Odometry, 'odom', self.odomCallback, 1)

        self.valueLaserRaw = {
            'range_min':0.0,
            'range_max':0.0,
            'ranges':[0.0]*360,
        }
        self.valueBatteryState = None
        self.valueOdometry = {
            'position':None,
            'orientation':None,
            'linearVelocity':None,
            'angularVelocity':None,
        }
        
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

        self.x_offset = None
        self.y_offset = None
        self.yaw_offset = None

        self.timer = self.create_timer(0.1,self.timerCallback)

    def publishVelocityCommand(self, linearVelocity, angularVelocity):
        msg = Twist()
        msg.linear.x = linearVelocity
        msg.angular.z = angularVelocity
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Publishing cmd_vel: linear={linearVelocity:.2f}, angular={angularVelocity:.2f}')

    def scanCallback(self, msg):
        self.valueLaserRaw = {
            'range_min':msg.range_min,
            'range_max':msg.range_max,
            'ranges':list(msg.ranges),
        }

    def batteryStateCallback(self, msg):
        self.valueBatteryState = msg

    def odomCallback(self, msg):
        self.valueOdometry = {
            'position':msg.pose.pose.position,
            'orientation':msg.pose.pose.orientation,
            'linearVelocity':msg.twist.twist.linear,
            'angularVelocity':msg.twist.twist.angular,
        }
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.x_offset is None and self.y_offset is None and self.yaw_offset is None:
            self.x_offset = msg.pose.pose.position.x
            self.y_offset = msg.pose.pose.position.y
            self.yaw_offset = yaw
            self.get_logger().info("Odometry reset: (0,0,0) set.")

        self.current_x = msg.pose.pose.position.x - self.x_offset
        self.current_y = msg.pose.pose.position.y - self.y_offset
        self.current_yaw = yaw - self.yaw_offset

    def timerCallback(self):
        pass

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Robot stopped.")

    def TurnTo(self, tenths_of_degrees ,angular_speed = 0.6):
        tenths_of_degrees += 15
        start_yaw = self.current_yaw
        target_angle = start_yaw + math.radians(tenths_of_degrees)

        twist = Twist()
        angle_diff = target_angle - self.current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # normalize

        if angle_diff > 0:
            twist.angular.z = abs(angular_speed)
            direction = "CCW (left)"
        else:
            twist.angular.z = -abs(angular_speed)
            direction = "CW (right)"

        self.get_logger().info(
            f"Start turn: from {math.degrees(start_yaw):.1f}° "
            f"to {math.degrees(target_angle):.1f}° ({direction})"
        )

        while abs(angle_diff) > 0.05:  # ~3°
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
            angle_diff = target_angle - self.current_yaw
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        self.stop_robot()
        self.get_logger().info("Turn complete ")

    def GoTo(self, tenths_of_cm):
        distance = tenths_of_cm / 100.0
        start_x, start_y = self.current_x, self.current_y
        twist = Twist()

        speed = 0.1 if abs(distance) > 0.2 else 0.1

        if distance > 0:
            twist.linear.x = speed   # forward
        else:
            twist.linear.x = -speed  # backward

        while True:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

            dx = self.current_x - start_x
            dy = self.current_y - start_y
            traveled = math.sqrt(dx*dx + dy*dy)

            self.get_logger().info(
                f"Moving... target={distance:.2f}m, traveled={traveled:.2f}m, "
                f"pos=({self.current_x:.2f},{self.current_y:.2f})"
            )

            if abs(traveled - abs(distance)) <= 0.01 or traveled >= abs(distance):
                break 

        self.stop_robot()
        rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3Controller()
    node.get_logger().info("Running Position Controller")

    node.GoTo(300)        # 30cm forward
    node.TurnTo(-1210)     # turn 270° CCW (but shortest = 90° CW)
   
    node.GoTo(250)       
    node.TurnTo(270)     

    node.GoTo(50)       
    node.TurnTo(-450)     

    node.GoTo(50)      
    node.TurnTo(-650)     

    node.GoTo(100) 
    node.TurnTo(850) 
    
    node.GoTo(100) 
    node.TurnTo(-500) 
    
    node.GoTo(120)

    # node.TurnTo(90)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()