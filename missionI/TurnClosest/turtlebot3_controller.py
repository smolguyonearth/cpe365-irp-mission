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

    def timerCallback(self):
        print("-----------------------------------------------------------")
        print('timer triggered')
        # self.publishVelocityCommand(linearVelocity,angularVelocity)
        # self.publishVelocityCommand(0.0, 0.08)

        print("Mission1 TurnClosest!")
        print(self.valueLaserRaw['ranges'][0])
        print(self.valueLaserRaw['ranges'][90])
        print(self.valueLaserRaw['ranges'][180])
        print(self.valueLaserRaw['ranges'][270])

        # frontsensor = min(self.valueLaserRaw['ranges'][0:15] + self.valueLaserRaw['ranges'][345:])
        # frontsensor = min(self.valueLaserRaw['ranges'][0:5] + self.valueLaserRaw['ranges'][355:])
        rightsensor = self.valueLaserRaw['ranges'][0:180]
        leftsensor = self.valueLaserRaw['ranges'][180:359]
        # rearsensor = self.valueLaserRaw['ranges'][155:205]

        valid_left  = [r for r in leftsensor if r != 0]
        valid_right = [r for r in rightsensor if r != 0]

        left_min  = min(valid_left) if valid_left else float('inf')
        right_min = min(valid_right) if valid_right else float('inf')

        # out_range = min(rightsensor + leftsensor + rearsensor)
        # combined = rightsensor + leftsensor + rearsensor

        # valid_readings = [r for r in combined if r != 0]

        # if valid_readings:
        #     out_range = min(valid_readings)
        # else:
        #     out_range = float('inf')
            
        if left_min <= 0.2 and right_min > 0.2:  
            self.publishVelocityCommand(0.0, -1.0)

        elif right_min <= 0.2 and left_min > 0.2:
            self.publishVelocityCommand(0.0, 1.0)

        elif left_min <= 0.2 and right_min <= 0.2:
            self.publishVelocityCommand(0.0, 0.0)

        else:
            self.publishVelocityCommand(0.0, 0.0)

        # if frontsensor <= 0.2 and frontsensor != 0:
            # self.publishVelocityCommand(0.0, 0.0)
        # elif out_range <= 0.2 and out_range != 0:
        #     self.publishVelocityCommand(0.0, 1.0)
        #     print(f"Out_range detected: {out_range}")
        # else:
        #     self.publishVelocityCommand(0.0, 0.2)


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
