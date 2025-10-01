# Authors : Nopphakorn Subs. Niwatchai Wang. Supasate Wor. Narith Tha. Tawan Thaep. Napatharak Muan.
from dis import dis
from socket import TIPC_SUBSCR_TIMEOUT
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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
        
        self.state = 0
        #Use this timer for the job that should be looping until interrupted
        self.timer = self.create_timer(0.1,self.timerCallback)

        # For distance tracking
        self.last_position = None
        self.total_distance = 0.0

    def publishVelocityCommand(self, linearVelocity, angularVelocity):
        msg = Twist()
        msg.linear.x = linearVelocity * 1.0
        msg.angular.z = angularVelocity * 1.0
        self.cmdVelPublisher.publish(msg)
        #self.get_logger().info('Publishing cmd_vel: "%s", "%s"' % linearVelocity, angularVelocity)

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

    def whatdoisee(self):
        threshold = 0.2
        window = 20
        directions = [0, 270, 180, 90]
        pattern = []

        for angle in directions:
            start = (angle - window) % 360
            end   = (angle + window) % 360

            if start < end:
                sector = self.valueLaserRaw['ranges'][start:end+1]
            else:
                sector = self.valueLaserRaw['ranges'][start:] + self.valueLaserRaw['ranges'][:end+1]

            valid = [r for r in sector if r != 0.0]

            if valid and min(valid) <= threshold:
                pattern.append(1)
            else:
                pattern.append(0)

        print(" ".join(map(str, pattern)))

        if (pattern[0] == 1):
            print("ahead")
        if(pattern[1] == 1):
            print("right")
        if(pattern[2] == 1):
            print("back")
        if(pattern[3] == 1):
            print("left")
        
        
    def timerCallback(self):
        print("-----------------------------------------------------------")
        print('timer triggered')


        ranges = self.valueLaserRaw['ranges']
        rmax = self.valueLaserRaw['range_max']

        # Define sectors
        front_sector = ranges[0:10] + ranges[-10:]  # front ±10°
        front_right_sector = ranges[315:]  # front-right
        front_left_sector  = ranges[0:45]     # front-left
        right_sector = ranges[0:90]
        left_sector  = ranges[270:]

        # Distances
        front  = min([r for r in front_sector if r != 0] or [rmax])
        front_right = min([r for r in front_right_sector if r != 0] or [rmax])
        front_left  = min([r for r in front_left_sector if r != 0] or [rmax])
        right  = min([r for r in right_sector if r != 0] or [rmax])
        left   = min([r for r in left_sector if r != 0] or [rmax])

        # Print parameters
        print(f"Front: {front:.3f} m | Front-Right: {front_right:.3f} m | Front-Left: {front_left:.3f} m")
        print(f"Right: {right:.3f} m | Left: {left:.3f}a m | Range max: {rmax:.3f} m")

        # Emergency stop if front too close
        if front < 0.15:
            print("FRONT OBSTACLE TOO CLOSE! STOPPING.")
            self.publishVelocityCommand(0.0, 0.0)
            return

        # Slight turn if front-left or front-right too close
        if front_right < 0.325:
            print("Front-right too close, turning slightly left")
            self.publishVelocityCommand(0.05, 0.25)  # small left turn
            return
        if front_left < 0.325:
            print("Front-left too close, turning slightly right")
            self.publishVelocityCommand(0.05, -0.25)  # small right turn
            return

        # Wall-following logic
        if right < left:  # follow right wall
            if front < 0.2:
                print("Obstacle ahead, turn left")
                self.publishVelocityCommand(0.0, 0.3)
            else:
                print("Wall on right, go forward")
                self.publishVelocityCommand(0.05, 0.0)
        else:  # follow left wall
            if front < 0.2:
                print("Obstacle ahead, turn right")
                self.publishVelocityCommand(0.0, -0.3)
            else:
                print("Wall on left, go forward")
                self.publishVelocityCommand(0.05, 0.0)

        pos = self.valueOdometry['position']
        if pos is not None:
            if self.last_position is not None:
                dx = pos.x - self.last_position.x
                dy = pos.y - self.last_position.y
                self.total_distance += (dx**2 + dy**2)**0.5
            self.last_position = pos

        print(f"Total distance traveled: {self.total_distance:.3f} m")



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
