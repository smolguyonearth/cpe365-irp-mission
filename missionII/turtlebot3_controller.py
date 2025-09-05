# Authors : Nopphakorn Subs. Niwatchai Wang. Supasate Wor. Narith Tha. Tawan Thaep. Napatharak Muan.
from dis import dis
from socket import TIPC_SUBSCR_TIMEOUT
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math

from time import sleep
#from std_msgs.msg import String

class Turtlebot3Controller(Node):

    def __init__(self):
        super().__init__('turtlebot3_controller')   #node name
        self.cmdVelPublisher = self.create_publisher(Twist, 'cmd_vel', 1)
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
            'position':None,        #Datatype: geometry_msg/Point   (x,y,z)
            'orientation':None,     #Datatype: geometry_msg/Quaternion (x,y,z,w)
            'linearVelocity':None,  #Datatype: geometry_msg/Vector3 (x,y,z)
            'angularVelocity':None, #Datatype: geometry_msg/Vector3 (x,y,z)
        }

        self.sequence = [
            ('go', 300),
            ('turn', -1210),
            ('go', 250),
            ('turn', 270),
            ('go', 50),
            ('turn', -450),
            ('go', 50),
            ('turn', -650),
            ('go', 100),
            ('turn', -850),
            ('go', 100),
            ('turn', -500),
            ('go', 120)
        ]

        #sequencing
        self.seq_index = 0 
        self.seq_in_progress = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.target_angle = 0.0

        #added
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

        self.x_offset = None
        self.y_offset = None
        self.yaw_offset = None
        
        self.timer = self.create_timer(0.1,self.timerCallback)

    def publishVelocityCommand(self, linearVelocity, angularVelocity):
        msg = Twist()
        msg.linear.x = linearVelocity * 1.0
        msg.angular.z = angularVelocity * 1.0
        self.cmdVelPublisher.publish(msg)

        #added
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

        #added
        #too yaw euler angles
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # reset coordinate
        if self.x_offset is None and self.y_offset is None and self.yaw_offset is None:
            self.x_offset = msg.pose.pose.position.x
            self.y_offset = msg.pose.pose.position.y
            self.yaw_offset = yaw
            self.get_logger().info("Odometry reset: (0,0,0) set.")

        self.current_x = msg.pose.pose.position.x - self.x_offset
        self.current_y = msg.pose.pose.position.y - self.y_offset
        self.current_yaw = yaw - self.yaw_offset
    
    # def TurnTo(self, tenths_of_degrees ,angular_speed = 1.0):
    #     start_yaw = self.current_yaw
    #     target_angle = start_yaw + math.radians(tenths_of_degrees)

    #     twist = Twist()
    #     angle_diff = target_angle - self.current_yaw
    #     angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # normalize

    #     if angle_diff > 0:
    #         twist.angular.z = abs(angular_speed)
    #         direction = "CCW (left)"
    #     else:
    #         twist.angular.z = -abs(angular_speed)
    #         direction = "CW (right)"

    #     self.get_logger().info(
    #         f"Start turn: from {math.degrees(start_yaw):.1f}° "
    #         f"to {math.degrees(target_angle):.1f}° ({direction})"
    #     )

    #     while abs(angle_diff) > 0.05:  # ~3°
    #         # self.cmd_vel_pub.publish(twist)
    #         self.cmdVelPublisher.publish(twist)

    #         rclpy.spin_once(self, timeout_sec=0.1)
    #         angle_diff = target_angle - self.current_yaw
    #         angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

    #     # self.stop_robot()
    #     self.publishVelocityCommand(0.0, 0.0)

    #     self.get_logger().info("Turn complete ")
    def TurnTo_step(self, target_angle, angular_speed=0.5):
        angle_diff = target_angle - self.current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        if abs(angle_diff) > 0.05:
            twist = Twist()
            twist.angular.z = angular_speed if angle_diff > 0 else -angular_speed
            self.cmdVelPublisher.publish(twist)
            return False
        else:
            self.publishVelocityCommand(0.0, 0.0)
            return True

    # def GoTo(self, tenths_of_cm):
    #     distance = tenths_of_cm / 1000.0
    #     start_x, start_y = self.current_x, self.current_y
    #     twist = Twist()

    #     speed = 0.5 if abs(distance) > 0.2 else 0.2

    #     if distance > 0:
    #         twist.linear.x = speed   # forward
    #     else:
    #         twist.linear.x = -speed  # backward

    #     while True:
    #         # self.cmd_vel_pub.publish(twist)
    #         self.cmdVelPublisher.publish(twist)

    #         rclpy.spin_once(self, timeout_sec=0.1)

    #         dx = self.current_x - start_x
    #         dy = self.current_y - start_y
    #         traveled = math.sqrt(dx*dx + dy*dy)

    #         self.get_logger().info(
    #             f"Moving... target={distance:.2f}m, traveled={traveled:.2f}m, "
    #             f"pos=({self.current_x:.2f},{self.current_y:.2f})"
    #         )

    #         if abs(traveled - abs(distance)) <= 0.01 or traveled >= abs(distance):
    #             break 

    #     # self.stop_robot()
    #     self.publishVelocityCommand(0.0, 0.0)
    #     rclpy.spin_once(self, timeout_sec=0.1)

    def GoTo_step(self, distance_m, speed=0.3):
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        traveled = math.sqrt(dx*dx + dy*dy)
        if traveled < distance_m:
            twist = Twist()
            twist.linear.x = speed
            self.cmdVelPublisher.publish(twist)
            return False
        else:
            self.publishVelocityCommand(0.0, 0.0)
            return True



    def timerCallback(self):
        print("-----------------------------------------------------------")
        print('timer triggered')

        if self.seq_index >= len(self.sequence):
            self.publishVelocityCommand(0.0, 0.0)
            return  # sequence finished

        command, value = self.sequence[self.seq_index]

        if command == 'turn':
            if not self.seq_in_progress:
                self.target_angle = self.current_yaw + math.radians(value/10.0)
                self.seq_in_progress = True

            done = self.TurnTo_step(self.target_angle)
            if done:
                self.seq_index += 1
                self.seq_in_progress = False

        elif command == 'go':
            if not self.seq_in_progress:
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.target_distance = value / 1000.0  # convert tenths of cm → meters
                self.seq_in_progress = True

            done = self.GoTo_step(self.target_distance)
            if done:
                self.seq_index += 1
                self.seq_in_progress = False

        print("Basecode!")

def robotStop():
    node = rclpy.create_node('tb3Stop')
    publisher = node.create_publisher(Twist, 'cmd_vel', 1)
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    publisher.publish(msg)
            
def main(args=None):
    rclpy.init(args=args)
    tb3ControllerNode = Turtlebot3Controller()
    print('tb3ControllerNode created')
    try:
        rclpy.spin(tb3ControllerNode)
    except:
        KeyboardInterrupt
    print('Done')
    
    tb3ControllerNode.publishVelocityCommand(0.0,0.0)
    tb3ControllerNode.destroy_node()
    robotStop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
