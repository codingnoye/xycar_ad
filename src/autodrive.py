#!/usr/bin/env python

import rospy, time
from linedetector import LineDetector
from motordriver import MotorDriver
from obstacledetector import ObstacleDetector

class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.now = time.time()

    def trace(self):
        raw_angle = self.line_detector.getAngle()
        dist = self.obstacle_detector.get_distance()
        angle = self.steer(raw_angle)
        speed = self.accelerate(angle, dist)
        self.driver.drive(angle + 90, speed + 90)

    def stop(self):
        self.driver.drive(90, 50)
        self.driver.drive(90, 60)
        time.sleep(0.1)
        self.driver.drive(90, 70)
        self.driver.drive(90, 80)
        time.sleep(0.1)
        self.driver.drive(90, 90)
        self.driver.drive(90, 90)
        time.sleep(10)

    def steer(self, angle):
        newangle = 0.86*angle
        if time.time()-self.now<8: return max(-10, min(10, newangle))
        return newangle

    def accelerate(self, angle, dist):
        print(time.time()-self.now)
        if dist[1]<90 and dist[1] != 0 and time.time()-self.now>72 and angle>-10 and angle<10:
            self.stop()
            return 0;
        if time.time()-self.now<8: return 50
        return 43

    def exit(self):
        print('finished')

if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)