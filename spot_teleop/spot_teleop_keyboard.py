import rclpy
from rclpy.node import Node

from spot_msgs.msg import GaitInput

import sys, select, termios, tty


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        i    
   j    k    l
        ,    
anything else : stop
q/z : increase/decrease angular speed and step length by 10%
w/x : increase/decrease only step length by 10%
e/c : increase/decrease only angular speed by 10%
1/2 : x
3/4 : y
5/6 : z
a/s : roll
d/f : pitch
g/h : yaw
CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'j': (0.1, 0, 0, 1),
    'l': (0.1, 0, 0, -1),
    ',': (-1, 0, 0, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

rollBindings = {
    'a': 0.1,
    's': -0.1,
}

pitchBindings = {
    'd': 0.1,
    'f': -0.1,
}
yawBindings = {
    'g': 0.1,
    'h': -0.1,
}

xyzBindings = {
    '1': (0.01, 0, 0),
    '2': (-0.01, 0, 0),
    '3': (0, 0.01, 0),
    '4': (0, -0.01, 0),
    '5': (0, 0, 0.01),
    '6': (0, 0, -0.01),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def status_def(status):
    if (status == 14):
        print(msg)
    return (status + 1) % 15     


class TeleopClass(Node):
    def __init__(self, 
            settings, 
            x = 0.0,
            y = 0.0,
            z = 0.1,
            roll = 0.0,
            pitch = 0.0,
            yaw = 0.0,
            StepLength = 0.024,
            LateralFraction = 0.0,
            YawRate = 0.0,
            StepVelocity = 0.01,
            ClearanceHeight = 0.024,
            PenetrationDepth = 0.003,
            SwingPeriod = 0.2,
            YawControl = 0.0,
            YawControlOn = 0.0,
            status = 0,
        ):

        super().__init__('teleop_twist_keyboard')
        spot_name = "Spot"

        self.pub = self.create_publisher(GaitInput, '/' + spot_name + '/inverse_gait_input', 10)
        self.create_timer(0.1, self.mytimercallback)

        self.settings = settings

        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.StepLength = StepLength
        self.StepDirection = 0
        self.LateralFraction = LateralFraction
        self.YawRate = YawRate
        self.StepVelocity = StepVelocity
        self.ClearanceHeight = ClearanceHeight
        self.PenetrationDepth = PenetrationDepth
        self.SwingPeriod = SwingPeriod
        self.YawControl = YawControl
        self.YawControlOn = YawControlOn
        self.status = status

        spot_name = "Spot"
        self.pub = self.create_publisher(GaitInput, '/' + spot_name + '/inverse_gait_input', 10)
        self.gait_msg = GaitInput()

    def mytimercallback(self):

        speed = 1.0
        turn = 0.5

        print(msg)
        print(vels(speed, turn))

        key = getKey(self.settings)
        if key in moveBindings.keys():
            self.StepDirection = moveBindings[key][0]
            self.YawRate = moveBindings[key][3]
        elif key in speedBindings.keys():
            speed += speed * speedBindings[key][0]
            turn += turn * speedBindings[key][1]
            print(vels(speed, turn))
            self.status = status_def(self.status)
        elif key in rollBindings.keys():
            self.roll += rollBindings[key]
            print("currently:\troll %s" % (self.roll))
            self.status = status_def(self.status)
        elif key in pitchBindings.keys():
            self.pitch += pitchBindings[key]
            print("currently:\tpitch %s" % (self.pitch))
            self.status = status_def(self.status)
        elif key in yawBindings.keys():
            self.yaw += yawBindings[key]
            print("currently:\tyaw %s" % (self.yaw))
            self.status = status_def(self.status)
        elif key in xyzBindings.keys():
            self.x += xyzBindings[key][0]
            self.y += xyzBindings[key][1]
            self.z += xyzBindings[key][2]
            print("currently:\tx %s\ty %s\tz %s " % (self.x, self.y, self.z))
            self.status = status_def(self.status)
        else:
            # Skip updating cmd_vel if key timeout and robot already
            # stopped.
            # if key == '' and StepLength == 0 and YawRate == 0:
            #     continue
            self.x = 0.0
            self.y = 0.0
            self.z = 0.1
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0
            self.StepDirection = 0
            # self.LateralFraction = 0.0
            # self.YawRate = 0.0
            # self.StepVelocity = 0.0
            # self.ClearanceHeight = 0.0
            # self.PenetrationDepth = 0.0
            # self.SwingPeriod = 0.0
            # self.YawControl = 0.0
            # self.YawControlOn = 0.0
            if key == '\x03':
                StepLength = 0.0
                exit()
            
        self.gait_msg.x = float(self.x)
        self.gait_msg.y = float(self.y)
        self.gait_msg.z = float(self.z)
        self.gait_msg.roll = float(self.roll)
        self.gait_msg.pitch = float(self.pitch)
        self.gait_msg.yaw = float(self.yaw)
        self.gait_msg.step_length = float(self.StepLength * self.StepDirection * speed)
        self.gait_msg.lateral_fraction = float(self.LateralFraction)
        self.gait_msg.yaw_rate = float(self.YawRate * turn)
        self.gait_msg.step_velocity = float(self.StepVelocity)
        self.gait_msg.clearance_height = float(self.ClearanceHeight)
        self.gait_msg.penetration_depth = float(self.PenetrationDepth)
        self.gait_msg.swing_period = float(self.SwingPeriod)
        self.gait_msg.yaw_control = float(self.YawControl)
        self.gait_msg.yaw_control_on = float(self.YawControlOn)

        self.pub.publish(self.gait_msg)

        restoreTerminalSettings(self.settings)  


def main(args=None):
    print('Hi from spot_teleop.')

    rclpy.init()
    teleop_twist_keyboard = TeleopClass(saveTerminalSettings())
    try:
        rclpy.spin(teleop_twist_keyboard)
    except KeyboardInterrupt:
        pass
    teleop_twist_keyboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
