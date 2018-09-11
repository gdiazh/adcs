#!/usr/bin/python

__author__ = 'gdiaz'

# COMMAND PARSER
import rospy

# Controller commands
SET_VOLTAGE = 1
SET_TORQUE = 2
SET_SPEED = 3
KEEP_ATTITUDE = 4
SET_MODE = 5
STOP = 6
USE_CURRENT_SETPOINT = 7
INCREASE_SETPOINT = 8
DECREASE_SETPOINT = 9
PRINT_SPEED = 10
AUTOMATIC_MODE = 11
STOP_X = 12
STOP_Y = 13
STOP_Z = 14
SET_CONTROL_MODE = 15
CHANGE_CURRENT_GAINS = 16
CHANGE_SPEED_GAINS = 17

TORQUE_MODE = 0
SPEED_MODE = 1
POS_MODE = 2

HDD_ZERO_SPEED = 1000
MIN_VOLTAGE = 0

class CommandParser:
    def __init__(self):
        self.command = "default"

    def parse_command(self, command, args, iGains, wGains):
        mode = args[0]
        control_mode = args[1]
        attitude = args[2]
        speed =args[3]
        torque =args[4]
        voltage =args[5]
        kp_ix = iGains[0]
        ki_ix = iGains[1]
        kd_ix = iGains[2]
        kp_wx = wGains[0]
        ki_wx = wGains[1]
        kd_wx = wGains[2]
        if (command == "set-voltage"):
            self.command_code = [SET_VOLTAGE, voltage[0], voltage[1], voltage[2]]
        elif (command == "set-torque"):
            self.command_code = [SET_TORQUE, torque[0], torque[1], torque[2]]
        elif (command == "set-speed"):
            self.command_code = [SET_SPEED, speed[0], speed[1], speed[2]]
        elif (command == "set-attitude"):
            self.command_code = [KEEP_ATTITUDE, attitude[0], attitude[1], attitude[2]]
        elif (command == "set-mode"):
            self.command_code = [SET_MODE, mode, 0, 0]
        elif (command == "set-attitude-control"):
            self.command_code = [SET_CONTROL_MODE, POS_MODE, 0, 0]
        elif (command == "set-speed-control"):
            self.command_code = [SET_CONTROL_MODE, SPEED_MODE, 0, 0]
        elif (command == "set-torque-control"):
            self.command_code = [SET_CONTROL_MODE, TORQUE_MODE, 0, 0]
        elif (command == "torque-gain"):
            self.command_code = [CHANGE_CURRENT_GAINS, kp_ix, ki_ix, kd_ix]
        elif (command == "speed-gain"):
            self.command_code = [CHANGE_SPEED_GAINS, kp_wx, ki_wx, kd_wx]
        elif (command == "stop"):
            self.command_code = [STOP, MIN_VOLTAGE, MIN_VOLTAGE, MIN_VOLTAGE]
        elif (command == "use-current-setpoint"):
            self.command_code = [USE_CURRENT_SETPOINT, 0, 0, 0]
        elif (command == "increase-setpoint"):
            self.command_code = [INCREASE_SETPOINT, 0, 0, 0]
        elif (command == "decrease-setpoint"):
            self.command_code = [DECREASE_SETPOINT, 0, 0, 0]
        elif (command == "print-speed"):
            self.command_code = [PRINT_SPEED, 0, 0, 0]
        elif (command == "automatic-mode"):
            self.command_code = [AUTOMATIC_MODE, 0, 0, 0]
        elif (command == "i-pid"):
            self.command_code = [SET_CONTROL_MODE, TORQUE_MODE, 0, 0]
            self.command_code = [AUTOMATIC_MODE, 0, 0, 0]
        elif (command == "yaw-pid"):
            self.command_code = [SET_CONTROL_MODE, POS_MODE, 0, 0]
            self.command_code = [AUTOMATIC_MODE, 0, 0, 0]
        else:
            self.command_code = [-1,-1,-1,-1]
            rospy.logwarn("Unknown command")
            print "Unknown command"
        return self.command_code


if __name__ == '__main__':
    comm = CommandParser()
    args = ["manual", [0,0,0], 200]
    comm.parse_command("set-speed", args)