#!/usr/bin/env python
import sys
# 添加生成的Python模块路径
sys.path.append('home/nf/catkin_ws/devel/lib/python3/dist-packages')

import rospy
from my_control.msg import RotorPWM
import tty
import termios
from std_srvs.srv import Empty

pwm_values = 0.0
step = 0.002

def call_service(service_name):
    rospy.wait_for_service(service_name)
    try:
        service = rospy.ServiceProxy(service_name, Empty)
        response = service()
        return response
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
def getKey():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rospy.init_node('pwm_keyboard_control')
    print("Press any key to see its output. Press 'q' to quit.")
    pwm_pub = rospy.Publisher('/airsim_node/drone_1/rotor_pwm_cmd', RotorPWM, queue_size=1)

    pwm_msg = RotorPWM()
    pwm_msg.header.stamp = rospy.Time.now()
    pwm_msg.rotorPWM0 = pwm_values
    pwm_msg.rotorPWM1 = pwm_values
    pwm_msg.rotorPWM2 = pwm_values
    pwm_msg.rotorPWM3 = pwm_values

    rate = rospy.Rate(100)  # 设置发布频率为10Hz

    while not rospy.is_shutdown():
        key = getKey()
        if key == 'q':
            break
        elif key == 'w':
            pwm_msg.rotorPWM0 = round(min(1.0, pwm_msg.rotorPWM0 + step), 3)
            pwm_msg.rotorPWM1 = round(min(1.0, pwm_msg.rotorPWM1 + step), 3)
            pwm_msg.rotorPWM2 = round(min(1.0, pwm_msg.rotorPWM2 + step), 3)
            pwm_msg.rotorPWM3 = round(min(1.0, pwm_msg.rotorPWM3 + step), 3)
        elif key == 's':
            pwm_msg.rotorPWM0 = round(max(0.0, pwm_msg.rotorPWM0 - step), 3)
            pwm_msg.rotorPWM1 = round(max(0.0, pwm_msg.rotorPWM1 - step), 3)
            pwm_msg.rotorPWM2 = round(max(0.0, pwm_msg.rotorPWM2 - step), 3)
            pwm_msg.rotorPWM3 = round(max(0.0, pwm_msg.rotorPWM3 - step), 3)
        pwm_pub.publish(pwm_msg)  # 发布消息
        print("pwm_msg: ", pwm_msg.rotorPWM0, pwm_msg.rotorPWM1, pwm_msg.rotorPWM2, pwm_msg.rotorPWM3)
        rate.sleep()  # 控制循环频率

if __name__ == "__main__":
    main()

