#!/usr/bin/python

__author__ = 'gdiaz'

# ROS INTERFACE

"""Provides a high level interface over ROS for bluetooth data exchange.
"""

import rospy

from threading import Thread
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from bt_receiver import btReceiver
from file_manager import fileManager
from command_parser import CommandParser
from rotations import Rotation

# Controller commands
from command_parser import HDD_ZERO_SPEED

class BTRosInterface:
    def __init__(self, test_name):
        # Only argument stuff
        self.running = False
        self.bt_receiver = btReceiver(debug = True)
        self.file_manager = fileManager(test_name, debug = True)
        self.command_parser = CommandParser()
        self.rotation = Rotation()
        self.mode = "manual"
        self.control_mode = "torque"
        self.attitude = [0,0,0]                                         #[ypr]  [DEG]
        self.speed = [HDD_ZERO_SPEED,HDD_ZERO_SPEED,HDD_ZERO_SPEED]     #[xyz]  [RPM]
        self.torque = [0,0,0]                                           #[xyz]  [Nm]
        self.voltage = [0,0,0]                                          #[xyz]  [V]
        self.iGains = [5, 10, 0]										#[PID]
        self.wGains = [15, 25, 0]										#[PID]

    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param('/rate', 5000)
        #Buetooth
        self.bt_receiver.initialize()

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True

        #subscribers
        self.command_sub = rospy.Subscriber('/controller_command', String, self.process_command)
        self.mode_sub = rospy.Subscriber('/mode', String, self.set_mode)
        self.control_mode_sub = rospy.Subscriber('/control_mode', String, self.set_control_mode)
        
        self.attitude_sub = rospy.Subscriber('/attitude', Float32MultiArray, self.set_attitude)
        
        self.speedX_sub = rospy.Subscriber('/speed_x', Float32, self.set_speed_x)
        self.speedY_sub = rospy.Subscriber('/speed_y', Float32, self.set_speed_y)
        self.speedZ_sub = rospy.Subscriber('/speed_z', Float32, self.set_speed_z)
        
        self.torqueX_sub = rospy.Subscriber('/torque_x', Float32, self.set_torque_x)
        self.torqueY_sub = rospy.Subscriber('/torque_y', Float32, self.set_torque_y)
        self.torqueZ_sub = rospy.Subscriber('/torque_z', Float32, self.set_torque_z)
        
        self.voltageX_sub = rospy.Subscriber('/voltage_x', Float32, self.set_voltage_x)
        self.voltageY_sub = rospy.Subscriber('/voltage_y', Float32, self.set_voltage_y)
        self.voltageZ_sub = rospy.Subscriber('/voltage_z', Float32, self.set_voltage_z)

        self.iGains_sub = rospy.Subscriber('/iGains', Float32MultiArray, self.set_iGains)
        self.wGains_sub = rospy.Subscriber('/wGains', Float32MultiArray, self.set_wGains)

        #publishers
        self.data1_pub = rospy.Publisher('/time1', Float32, queue_size=70)
        self.data2_pub = rospy.Publisher('/wx', Float32, queue_size=70)
        self.data3_pub = rospy.Publisher('/wy', Float32, queue_size=70)
        self.data4_pub = rospy.Publisher('/wz', Float32, queue_size=70)
        self.data5_pub = rospy.Publisher('/time2', Float32, queue_size=70)
        self.data6_pub = rospy.Publisher('/wx_raw', Float32, queue_size=70)
        self.data7_pub = rospy.Publisher('/wy_raw', Float32, queue_size=70)
        self.data8_pub = rospy.Publisher('/wz_raw', Float32, queue_size=70)

        self.cmd_yaw_pub = rospy.Publisher('/cmd_yaw', Float32, queue_size=70)

        self.cmd_speedX_pub = rospy.Publisher('/cmd_speedX', Float32, queue_size=70)
        self.cmd_torqueX_pub = rospy.Publisher('/cmd_torqueX', Float32, queue_size=70)
        self.cmd_voltageX_pub = rospy.Publisher('/cmd_voltageX', Float32, queue_size=70)

        self.qE4_pub = rospy.Publisher('/qE4', Float32, queue_size=70)
        self.qE1_pub = rospy.Publisher('/qE1', Float32, queue_size=70)
        self.qE2_pub = rospy.Publisher('/qE2', Float32, queue_size=70)
        self.qE3_pub = rospy.Publisher('/qE3', Float32, queue_size=70)

        self.qDE1_pub = rospy.Publisher('/qDE1', Float32, queue_size=70)
        self.qDE2_pub = rospy.Publisher('/qDE2', Float32, queue_size=70)
        self.qDE3_pub = rospy.Publisher('/qDE3', Float32, queue_size=70)

        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False
        
        self.command_sub.unregister()
        self.mode_sub.unregister()
        self.control_mode_sub.unregister()
        
        self.attitude_sub.unregister()

        self.speedX_sub.unregister()
        self.speedY_sub.unregister()
        self.speedZ_sub.unregister()

        self.torqueX_sub.unregister()
        self.torqueY_sub.unregister()
        self.torqueZ_sub.unregister()

        self.voltageX_sub.unregister()
        self.voltageY_sub.unregister()
        self.voltageZ_sub.unregister()

        self.iGains_sub.unregister()
        self.wGains_sub.unregister()

        self.data1_pub.unregister()
        self.data2_pub.unregister()
        self.data3_pub.unregister()
        self.data4_pub.unregister()
        self.data5_pub.unregister()
        self.data6_pub.unregister()
        self.data7_pub.unregister()
        self.data8_pub.unregister()

        self.cmd_yaw_pub.unregister()

        self.cmd_speedX_pub.unregister()
        self.cmd_torqueX_pub.unregister()
        self.cmd_voltageX_pub.unregister()

        self.qE4_pub.unregister()
        self.qE1_pub.unregister()
        self.qE2_pub.unregister()
        self.qE3_pub.unregister()

        self.qDE1_pub.unregister()
        self.qDE2_pub.unregister()
        self.qDE3_pub.unregister()

        self.bt_receiver.stop()
        self.file_manager.stop()

    def set_mode(self, msg):
        self.mode = msg.data

    def set_control_mode(self, msg):
        self.control_mode = msg.data

    def set_attitude(self, msg):
        self.attitude = msg.data

    def set_speed_x(self, msg):
        self.speed[0] = msg.data

    def set_speed_y(self, msg):
        self.speed[1] = msg.data

    def set_speed_z(self, msg):
        self.speed[2] = msg.data

    def set_torque_x(self, msg):
        self.torque[0] = msg.data

    def set_torque_y(self, msg):
        self.torque[1] = msg.data

    def set_torque_z(self, msg):
        self.torque[2] = msg.data

    def set_voltage_x(self, msg):
        self.voltage[0] = msg.data

    def set_voltage_y(self, msg):
        self.voltage[1] = msg.data

    def set_voltage_z(self, msg):
        self.voltage[2] = msg.data

    def set_iGains(self, msg):
        self.iGains = msg.data

    def set_wGains(self, msg):
        self.wGains = msg.data

    def process_command(self, msg):
        print "msg to send:"+msg.data
        if (msg.data == "end") :
            self.running = False
            return
        else:
            args = [self.mode, self.control_mode, self.attitude, self.speed, self.torque, self.voltage]
            command_code = self.command_parser.parse_command(msg.data, args, self.iGains, self.wGains)
            if (command_code!=[-1,-1,-1,-1]):
                frame = self.file_manager.encode(command_code)
                self.bt_receiver.write(frame)
            else:
                rospy.logwarn("Unknown command")
                print "Unknown command"

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            # Receive data
            if(self.bt_receiver.read()):
                packet = self.bt_receiver.packet
                if (packet[0]==1 or packet[0]==2):
                    self.bt_receiver.reset()
                    #write data to file
                    self.file_manager.save_data(packet, self.speed, self.torque, self.voltage, self.attitude)
                    #publish data
                    data = self.file_manager.decode(packet)
                    if (packet[0]==1):
                        self.data1_pub.publish(data[0])#*57.2958
                        self.data2_pub.publish(data[1])
                        self.data3_pub.publish(data[2])
                        self.data4_pub.publish(data[3])#*9.5493

                        qE = self.rotation.get_qE(self.attitude[0]*0.0174, [data[0], data[1], data[2], data[3]])
                        self.qE4_pub.publish(qE[0])
                        self.qE1_pub.publish(qE[1])
                        self.qE2_pub.publish(qE[2])
                        self.qE3_pub.publish(qE[3])

                        self.qDE1_pub.publish(qE[1]*qE[0])
                        self.qDE2_pub.publish(qE[2]*qE[0])
                        self.qDE3_pub.publish(qE[3]*qE[0])
                    elif (packet[0]==2):
                        self.data5_pub.publish(data[0])
                        self.data6_pub.publish(data[1])
                        self.data7_pub.publish(data[2])
                        self.data8_pub.publish(data[3])
                    self.cmd_yaw_pub.publish(self.attitude[0])
                    self.cmd_speedX_pub.publish(self.speed[0])
                    self.cmd_torqueX_pub.publish(self.torque[0])
                    self.cmd_voltageX_pub.publish(self.voltage[0])
            rate.sleep()

if __name__ == '__main__':
    test_name = raw_input("test_name: ")
    rospy.init_node('ros_bt_interface')
    bt_ros = BTRosInterface(test_name)
    bt_ros.initialize()
    bt_ros.start()
    rospy.spin()
    bt_ros.stop()