from __future__ import print_function

import threading

from matplotlib import rc

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
    'j': (0.5, 0, 0, 1),
    'l': (0.5, 0, 0, -1),
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

def main(args=None):
    print('Hi from spot_teleop.')
    settings = saveTerminalSettings()
    rclpy.init(args=args)

    spot_name = "Spot"

    speed = 0.05
    turn = 1.5
    repeat = 0.0
    key_timeout = 0.0
    
    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(GaitInput, '/' + spot_name + '/inverse_gait_input', 10)

    x = 0.0
    y = 0.0
    z = 0.1
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    StepLength = 0.1
    LateralFraction = 0.0
    YawRate = 0.0
    StepVelocity = 0.4
    ClearanceHeight = 0.15
    PenetrationDepth = 0.00003
    SwingPeriod = 0.3
    YawControl = 0.0
    YawControlOn = 0.0
    status = 0

    try:
        print(msg)
        print(vels(speed, turn))

        while (True):
            #rclpy.spin_once(node)
            key = getKey(settings)
            if key in moveBindings.keys():
                StepLength = moveBindings[key][0]
                YawRate = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed, turn))
                status = status_def(status)
            elif key in rollBindings.keys():
                roll = roll + rollBindings[key]
                print("currently:\troll %s" % (roll))
                status = status_def(status)
            elif key in pitchBindings.keys():
                pitch = pitch + pitchBindings[key]
                print("currently:\tpitch %s" % (pitch))
                status = status_def(status)
            elif key in yawBindings.keys():
                yaw = yaw + yawBindings[key]
                print("currently:\tyaw %s" % (yaw))
                status = status_def(status)
            elif key in xyzBindings.keys():
                x = x + xyzBindings[key][0]
                y = y + xyzBindings[key][1]
                z = z + xyzBindings[key][2]
                print("currently:\tx %s\ty %s\tz %s " % (x, y, z))
                status = status_def(status)
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                # if key == '' and StepLength == 0 and YawRate == 0:
                #     continue
                x = 0.0
                y = 0.0
                z = 0.1
                roll = 0.0
                pitch = 0.0
                yaw = 0.0
                StepLength = 0.0
                LateralFraction = 0.0
                YawRate = 0.0
                StepVelocity = 0.0
                ClearanceHeight = 0.0
                PenetrationDepth = 0.0
                SwingPeriod = 0.0
                YawControl = 0.0
                YawControlOn = 0.0
                if (key == '\x03'):
                    StepLength = 0.0
                    break
            
            gait_msg = GaitInput()
            gait_msg.x = float(x)
            gait_msg.y = float(y)
            gait_msg.z = float(z)
            gait_msg.roll = float(roll)
            gait_msg.pitch = float(pitch)
            gait_msg.yaw = float(yaw)
            gait_msg.step_length = float(StepLength * speed)
            gait_msg.lateral_fraction = float(LateralFraction)
            gait_msg.yaw_rate = float(YawRate * turn)
            gait_msg.step_velocity = float(StepVelocity)
            gait_msg.clearance_height = float(ClearanceHeight)
            gait_msg.penetration_depth = float(PenetrationDepth)
            gait_msg.swing_period = float(SwingPeriod)
            gait_msg.yaw_control = float(YawControl)
            gait_msg.yaw_control_on = float(YawControlOn)

            pub.publish(gait_msg)

    except Exception as e:
        print(e)

    finally:
        gait_msg = GaitInput()
        gait_msg.x = 0.0
        gait_msg.y = 0.0
        gait_msg.z = 0.1
        gait_msg.roll = 0.0
        gait_msg.pitch = 0.0
        gait_msg.yaw = 0.0
        gait_msg.step_length = 0.0
        gait_msg.lateral_fraction = 0.0
        gait_msg.yaw_rate = 0.0
        gait_msg.step_velocity = 0.0
        gait_msg.clearance_height = 0.0
        gait_msg.penetration_depth = 0.0
        gait_msg.swing_period = 0.0
        gait_msg.yaw_control = 0.0
        gait_msg.yaw_control_on = 0.0

        pub.publish(gait_msg)        
        restoreTerminalSettings(settings)  


if __name__ == '__main__':
    main()
