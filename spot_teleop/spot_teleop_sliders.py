#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from spot_msgs.msg import GaitInput

import tkinter as tk

params = {
    'x': 0.,
    'y': 0.,
    'z': 0.1,
    'roll': 0.,
    'pitch': 0.,
    'yaw': 0.,
    'step_length': 0.,
    'lateral_fraction': 0.,
    'yaw_rate': 0.,
    'clearance_height': 0.024,
    'penetration_depth': 0.003,
    'swing_period': 0.2,
    'yaw_control': 0.,
    'yaw_control_on': 0.
}


def create_sliders(tk_module):
    my_scale = []
    for i, text in enumerate(params):
        my_scale.append(tk.Scale(tk_module, label=text, from_=-1, to=1, resolution=0.001, length=200, orient='horizontal'))
        my_scale[i].set(params[text])
        my_scale[i].grid(row=i,column=1)
    return my_scale


class TeleopClass(Node):
    def __init__(self, tk_stuffs, sliders):

        super().__init__('teleop_twist_sliders')
        spot_name = "Spot"

        self.pub = self.create_publisher(GaitInput, '/' + spot_name + '/inverse_gait_input', 10)
        self.create_timer(0.1, self.mytimercallback)

        self.tk_stuffs = tk_stuffs
        self.sliders = sliders
        self.params = None

    def mytimercallback(self):

        self.tk_stuffs.update()
        self.params = {param: self.sliders[i].get() for i, param in enumerate(params)}

        gait_msg = GaitInput()
        gait_msg.x = self.params['x']
        gait_msg.y = self.params['y']
        gait_msg.z = self.params['z']
        gait_msg.roll = self.params['roll']
        gait_msg.pitch = self.params['pitch']
        gait_msg.yaw = self.params['yaw']
        gait_msg.step_length = self.params['step_length']
        gait_msg.lateral_fraction = self.params['lateral_fraction']
        gait_msg.yaw_rate = self.params['yaw_rate']
        gait_msg.step_velocity = 0.001
        gait_msg.clearance_height = self.params['clearance_height']
        gait_msg.penetration_depth = self.params['penetration_depth']
        gait_msg.swing_period = self.params['swing_period']
        gait_msg.yaw_control = self.params['yaw_control']
        gait_msg.yaw_control_on = self.params['yaw_control_on']

        self.pub.publish(gait_msg)


def main():
    rclpy.init()

    tk_stuffs = tk.Tk()
    tk_stuffs.title('Params')
    sliders = create_sliders(tk_stuffs)
    teleop_twist_sliders = TeleopClass(tk_stuffs, sliders)

    try:
        rclpy.spin(teleop_twist_sliders)
    except KeyboardInterrupt:
        pass
    teleop_twist_sliders.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
