import os
from joystick import JoyStick
from cart import Cart
import time;
import cv2;
import threading;
import json;

cart = Cart()

class Logger:

    def __init__(self):
        self.camera = cv2.VideoCapture("/dev/video0")
        self.started = False
        self.stopped_ = False
        self.counter = 0
        self.map = {}
        self.result_dir = "train"
        if not os.path.exists(self.result_dir):
            os.makedirs(self.result_dir)

    def start(self):
        self.started = True
        cart.steer(0)

    def stop(self):
        if self.stopped_:
            return
        self.stopped_ = True
        cart.stop()
        path = "{}/result.json".format(self.result_dir)
        with open(path, 'w') as fp:
            json.dump(self.map.copy(), fp)

    def log(self, axis):
        if self.started :
            print("axis:".format(axis))
            cart.steer(axis)
            return_value, image = self.camera.read()
            path = "{}/{}.jpg".format(self.result_dir, self.counter)
            self.map[self.counter] = axis
            cv2.imwrite(path, image)
            self.counter = self.counter + 1
            
    def stopped(self):
        return self.stopped_

js = JoyStick()
logger = Logger()


def joystick_thread():
    js.open();
    while not logger.stopped():
        time, value, type_, number = js.read()
        if js.type(type_) == "button":
            print("button:{} state: {}".format(number, value))
            if number == 5 and value == 1:
                logger.start()
            if number == 4 and value == 1:
                logger.stop()
        if js.type(type_) == "axis":
            print("axis:{} state: {}".format(number, value))
            if number == 0:
                # handle_axis(time, value);
                js.x_axis = value * 1.0 / 32767

def main():
    t = threading.Thread(target=joystick_thread, args=())
    t.start()

    # logger.start();
    while not logger.stopped():
        # time.sleep(0.01);
        logger.log(js.x_axis)

    t.join()
    cart.stop()

main()
cart.stop()
