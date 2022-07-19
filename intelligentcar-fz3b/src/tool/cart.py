
from ctypes import *
import time

import math
import config
import struct

if config.serial_mock_debug == False:
    import serial


def byte2float(x):
    return struct.unpack('<f', struct.pack('4b', *x))[0]

def float2byte(f):
    return [i for i in struct.pack('<f', f)]

def print_cmd(bytes):
  l = [hex(int(i)) for i in bytes]
  print("Send Cmd [{}]".format(" ".join(l)))

head = bytes.fromhex('42')
tail = bytes.fromhex('10')
speed_angle_cmd = bytes.fromhex('01')
control_model_cmd = bytes.fromhex('02')
comma_trail = bytes.fromhex('10')


class Cart:
    def __init__(self):
        portx = "/dev/ttyUSB0"
        bps = 115200
        if config.serial_mock_debug == False:
            self.serial = serial.Serial(portx, int(bps), timeout=1, parity=serial.PARITY_NONE, stopbits=1)
       
        self.full_speed = config.full_speed
        self.min_speed = config.min_speed

        self.full_angle = config.full_angle
        self.min_angle = config.min_angle

        self.cur_speed = config.default_speed
        self.cur_angle = config.default_angle
        self.debug = config.car_serial_debug

    def check_speed_param(self, speed):
        if speed < self.min_speed:
            print("Warning, {} is out of range, min speed is {}.".format(speed,self.min_speed))
            speed = self.min_speed
        elif speed > self.full_speed:
            print("Warning, {} is out of range, max speed is {}.".format(speed,self.full_angle))
        return speed
    
    def check_angle_param(self, angle):
        if angle < self.min_angle:
            print("Warning, {} is out of range, min angle is {}.".format(angle,self.min_angle))
            angle = self.min_angle
        elif angle > self.full_angle:
            print("Warning, {} is out of range, max angle is {}.".format(angle,self.full_angle))
            angle = self.full_angle
        return angle

    def send_cmd(self, cmd, payload):
        all_cmd = head + cmd + payload
        check_sum = int(0)
        for i in all_cmd:
            check_sum += i
            check_sum = int(check_sum % 256)
        all_cmd = all_cmd + check_sum.to_bytes(1, byteorder='little', signed=False) + tail
        if self.debug :
            print_cmd(all_cmd)
        if config.serial_mock_debug == False:
            self.serial.write(all_cmd)

    def move_steer(self, speed, angle):
        angle = self.full_angle * angle
        payload = float2byte(speed) +  float2byte(angle)
        self.send_cmd(speed_angle_cmd,bytes(payload))

    def move(self, speed):
        print("Set speed {}".format(speed))
        self.cur_speed = self.check_speed_param(speed)
        self.move_steer(self.cur_speed,self.cur_angle)

    def move_open(self, speed):
        print("Open Set speed {}".format(speed))
        self.cur_speed = speed
        self.move_steer(self.cur_speed,self.cur_angle)

    def steer(self, angle):
        print("Set steer {}".format(angle))
        self.cur_angle = self.check_angle_param(angle)
        self.move_steer(self.cur_speed,angle)

    def stop(self):
        self.cur_angle = 0
        self.cur_speed = 0
        print("Stop .")
        self.move_steer(self.cur_speed,self.cur_angle)
 
    def reverse(self):
        
        self.cur_speed = -self.cur_speed
        print("reverse {}".format(self.cur_speed))
        self.move_steer(self.cur_speed,self.cur_angle)

    def turn_left(self):
        pass

    def turn_right(self):
        pass

    def change_control_mode(self, open_close):
        if open_close:
            print("Change to open loop control.")
            payload = bytes.fromhex('01 00 00 00 00 00 00 00') # 闭环
        else:
            print("Change to close loop control.")
            payload = bytes.fromhex('00 00 00 00 00 00 00 00') # 开环
        self.send_cmd(control_model_cmd,bytes(payload))

         

def test():
    c = Cart()

    #c.change_control_mode(True)
    c.move_open(700)
    #time.sleep(2)
    #c.stop()
    #time.sleep(1)
    #c.move(-2.0)
    #time.sleep(2)
    #c.stop()
    # c.steer(0);
    # time.sleep(10);
    # c.steer(0.2);
    # time.sleep(10);
    # c.steer(0.5);
    # time.sleep(1);
    # c.stop();

def test_serial():
    c = Cart()
    c.move_steer(12.3,3.5)

if __name__ == "__main__":
    #test_serial()
    test()
    # # testmove()
    # # test();
    # #     turntest()
    # c = Cart()
    # while True:
    #     # c.move([5,5,5,5])
    #     c.move([10, 10, 10, 10])
    #     time.sleep(3)
    #     c.move([0, 0, 0, 0])
    #     time.sleep(3)
