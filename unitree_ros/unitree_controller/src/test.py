#!/usr/bin/python
import rospy
import numpy as np
import socket
from unitree_legged_msgs.msg import MotorCmd


class UpdReciever():
    def __init__(self):
        self.UDP_IP = "127.0.0.1"
        self.UDP_PORT_RECVING = 25000
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.UDP_IP, self.UDP_PORT_RECVING))
        robot_name = "a1"
        self.servo_pub_0 =rospy.Publisher("/" + robot_name + "_gazebo/FR_hip_controller/command",MotorCmd,queue_size=1)
        self.servo_pub_1 = rospy.Publisher("/" + robot_name + "_gazebo/FR_thigh_controller/command",MotorCmd, queue_size=1)
        self.servo_pub_2 = rospy.Publisher("/" + robot_name + "_gazebo/FR_calf_controller/command",MotorCmd, queue_size=1)
        self.servo_pub_3 = rospy.Publisher("/" + robot_name + "_gazebo/FL_hip_controller/command",MotorCmd, queue_size=1)
        self.servo_pub_4 = rospy.Publisher("/" + robot_name + "_gazebo/FL_thigh_controller/command",MotorCmd, queue_size=1)
        self.servo_pub_5 = rospy.Publisher("/" + robot_name + "_gazebo/FL_calf_controller/command",MotorCmd, queue_size=1)
        self.servo_pub_6 = rospy.Publisher("/" + robot_name + "_gazebo/RR_hip_controller/command",MotorCmd, queue_size=1)
        self.servo_pub_7 = rospy.Publisher("/" + robot_name + "_gazebo/RR_thigh_controller/command",MotorCmd, queue_size=1)
        self.servo_pub_8 = rospy.Publisher("/" + robot_name + "_gazebo/RR_calf_controller/command",MotorCmd, queue_size=1)
        self.servo_pub_9 = rospy.Publisher("/" + robot_name + "_gazebo/RL_hip_controller/command",MotorCmd, queue_size=1)
        self.servo_pub_10 = rospy.Publisher("/" + robot_name + "_gazebo/RL_thigh_controller/command",MotorCmd, queue_size=1)
        self.servo_pub_11 = rospy.Publisher("/" + robot_name + "_gazebo/RL_calf_controller/command",MotorCmd, queue_size=1)
        rospy.loginfo("UDP Reciever is initialized")

    def __recv_wait(self):
        while True:
            data, addr = self.socket.recvfrom(1024) # buffer size is 1024 bytes
            package = data.split( )
            m1 = float(package[0])
            m2 = float(package[1])
            m3 = float(package[2])
            m4 = float(package[3])
            m4 = float(package[4])
            m5 = float(package[5])
            m6 = float(package[6])
            m7 = float(package[7])
            m8 = float(package[8])
            m9 = float(package[9])
            m10 = float(package[10])
            m11 = float(package[11])
            m12 = float(package[12])

            joint_data = [m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12]
            return joint_data

    def run(self):
        while not rospy.is_shutdown():
            joint_data = self.__recv_wait()
            cmd_1 = MotorCmd()
            cmd_1.Kp = 100
            cmd_1.Kd = 2
            cmd_1.mode = 0x0A 
            cmd_1.dq = 0
            cmd_1.q = joint_data[0]
            
            self.servo_pub_0.publish(cmd_1)            
if __name__ == '__main__':
    rospy.init_node("udp_receiver")
    ur = UpdReciever()
    ur.run()
    rospy.spin()