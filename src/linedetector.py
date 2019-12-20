import rospy
import cv2
import threading, time, pickle
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LineDetector:

    def __init__(self, topic, showMode = True, camMode = True):
        self.showMode = showMode
        self.camMode = camMode
        self.img_width, self.img_height = 640, 480
        self.cfg = [(145, 170), (500, 170), (9, 244), (639, 244), 5, 61, 80]
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.detect)
        if self.camMode:
            self.recorder = cv2.VideoWriter(
                '/home/nvidia/xycar/src/auto_drive/record11.avi',
                cv2.VideoWriter_fourcc(*'MJPG'),
                30,
                (640, 480)
            )
        self.angle = 0
        self.historyScale = 0

    def __del__(self):
        if self.camMode:
            self.recorder.release()
        cv2.destroyAllWindows()

    def detect(self, data):
        cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        TL, TR, LL, LR, blur, lth, uth = self.cfg
        tTL, tTR, tLL, tLR = [0, 0], [640, 0], [0, 240], [640, 240]
        if self.camMode:
            self.recorder.write(cam_img)
        originBox = np.float32([TL, TR, LL, LR])
        transBox = np.float32([tTL, tTR, tLL, tLR])
        intOriginBox = np.int32([[TL, LL, LR, TR]])
        trans = cv2.getPerspectiveTransform(originBox, transBox)
        warped = cv2.warpPerspective(cam_img, trans, (640, 240))
        # 
        blur += blur % 2 + 1
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        blured = cv2.GaussianBlur(gray, (blur, blur), 0)
        edge = cv2.Canny(blured, lth, uth)
        lines = cv2.HoughLinesP(edge, 1, np.pi/180, 30, None, 80, 50)
        filtered = cv2.cvtColor(edge, cv2.COLOR_GRAY2BGR)
        # 
        angles = []
        if lines is not None:
            for line in lines:
                x1,y1,x2,y2 = line[0]
                if y1>y2: x1, y1, x2, y2 = x2, y2, x1, y1
                if self.showMode: cv2.line(filtered,(x1,y1),(x2,y2),(255,0,0),3)
                #angles.append((x2-x1)*(abs(y2-y1)/(y2-y1)))
                if y2-y1 == 0:
                    angles.append(0)
                else:
                    angles.append(np.degrees(np.arctan2(y2-y1,x2-x1))-90)
                
        if len(angles)>0:
            angle = np.average(angles)
            self.angle = self.angle * self.historyScale + angle * (1-self.historyScale)
        
        if self.showMode:
            origin = cv2.polylines(cam_img, [intOriginBox], True, (255, 0, 0), 2)
            cv2.imshow("origin", origin)
            cv2.imshow("view", filtered)
            cv2.waitKey(1)

    def getAngle(self):
        return self.angle