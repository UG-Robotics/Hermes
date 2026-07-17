import math
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu

class State(Enum):
    INIT     = auto()
    SETTLE   = auto()
    STRAIGHT = auto()
    TURNING  = auto()
    FINISHED = auto()

class WROController(Node):

    # Original Stable Constants
    KP = 0.70        
    KI = 0.02
    KD = 0.20
    INTEGRAL_CLAMP = 0.30

    CRUISE_SPEED = -0.30     
    TURN_SPEED   = -0.22     
    MAX_STEER    = 0.50     

    TURN_TRIGGER_DIST  = 0.65   
    TURN_COMPLETE_DEG  = 78.0   
    TOTAL_TURNS = 12   
    DT = 0.05

    def __init__(self):
        super().__init__('wro_controller')

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(LaserScan, '/tof_left',  self._cb_left,  10)
        self.create_subscription(LaserScan, '/tof_right', self._cb_right, 10)
        self.create_subscription(LaserScan, '/tof_front', self._cb_front, 10)
        self.create_subscription(Imu,       '/imu',       self._cb_imu,   10)

        self._left_dist, self._right_dist, self._front_dist = 0.5, 0.5, 2.0
        self._yaw = 0.0
        self._ready_flags = {'left': False, 'right': False, 'front': False, 'imu': False}

        # Hard reset of all memory variables
        self._state = State.INIT
        self._settle_start_time = None
        self._turn_count = 0
        self._yaw_accumulated = 0.0
        self._prev_yaw = 0.0
        self._integral = 0.0
        self._prev_error = 0.0

        self.declare_parameter('turn_direction', 1)
        self._turn_dir = int(self.get_parameter('turn_direction').value)

        self.create_timer(self.DT, self._control_loop)
        self.get_logger().info('Hermes v5.5: Stability Logic Engaged')

    @staticmethod
    def _filter_range(msg, fallback):
        valid = [r for r in msg.ranges if math.isfinite(r) and 0.02 < r < 4.0]
        return min(valid) if valid else fallback

    def _cb_left(self, msg): 
        self._left_dist = self._filter_range(msg, 0.5)
        self._ready_flags['left'] = True
    def _cb_right(self, msg): 
        self._right_dist = self._filter_range(msg, 0.5)
        self._ready_flags['right'] = True
    def _cb_front(self, msg): 
        self._front_dist = self._filter_range(msg, 2.0)
        self._ready_flags['front'] = True
    def _cb_imu(self, msg):
        q = msg.orientation
        self._yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self._ready_flags['imu'] = True

    def _control_loop(self):
        if self._state == State.INIT:
            if all(self._ready_flags.values()):
                self._state = State.SETTLE
                self._settle_start_time = self.get_clock().now()
                self.get_logger().info('Sensors Live. Settling physics...')
            self._publish(0.0, 0.0)

        elif self._state == State.SETTLE:
            # Wait 2 seconds for the robot to stop "bouncing"
            elapsed = (self.get_clock().now() - self._settle_start_time).nanoseconds / 1e9
            if elapsed > 2.0:
                self._state = State.STRAIGHT
                self._prev_yaw = self._yaw
                self._integral = 0.0
                self._prev_error = 0.0
                self.get_logger().info('Physics Stable. Launching Run.')
            self._publish(0.0, 0.0)

        elif self._state == State.STRAIGHT:
            if self._front_dist < self.TURN_TRIGGER_DIST:
                self._state, self._yaw_accumulated, self._prev_yaw = State.TURNING, 0.0, self._yaw
                self._integral, self._prev_error = 0.0, 0.0
                return

            error = self._right_dist - self._left_dist
            self._integral = max(-self.INTEGRAL_CLAMP, min(self.INTEGRAL_CLAMP, self._integral + (error * self.DT)))
            derivative = (error - self._prev_error) / self.DT
            self._prev_error = error

            steer = (self.KP * error) + (self.KI * self._integral) + (self.KD * derivative)
            self._publish(self.CRUISE_SPEED, max(-self.MAX_STEER, min(self.MAX_STEER, steer)))

        elif self._state == State.TURNING:
            delta = self._yaw - self._prev_yaw
            while delta >  math.pi: delta -= 2 * math.pi
            while delta < -math.pi: delta += 2 * math.pi
            self._yaw_accumulated += abs(delta)
            self._prev_yaw = self._yaw

            self._publish(self.TURN_SPEED, self.MAX_STEER * self._turn_dir)

            if math.degrees(self._yaw_accumulated) >= self.TURN_COMPLETE_DEG:
                self._turn_count += 1
                self._integral, self._prev_error = 0.0, 0.0 
                self._state = State.FINISHED if self._turn_count >= self.TOTAL_TURNS else State.STRAIGHT

        elif self._state == State.FINISHED:
            self._publish(0.0, 0.0)

    def _publish(self, linear, angular):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self._pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WROController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()