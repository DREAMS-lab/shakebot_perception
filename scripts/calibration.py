from statistics import mean
import rospy
import PySimpleGUI as sg
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
from tf import transformations as tform
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
import yaml

class calibration:
    def __init__(self):
        rospy.init_node("bed_calibrator")
        sg.theme("LightGreen")
        self.bridge = CvBridge()
        layout = [[sg.Image(key="-IMAGE-")],[sg.Button("Set Home")], [sg.Button("Reset")]]
        self.window = sg.Window("Calibrator", layout, finalize=True)
        self.detected = False
        self.write = False
        self.tagsDict = {}
        self.main()
    
    def updateImage(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img = cv.resize(img, [640, 480])
        imgbytes = cv.imencode(".png", img)[1].tobytes()
        self.window["-IMAGE-"].update(data=imgbytes)
        
    def setDetections(self, msg):
        if msg.detections:
            self.tags = msg.detections
            for i in self.tags:
                self.tagsDict.update({i.id:{"pose":{ "position":{ "x": i.pose.pose.pose.position.x, "y":i.pose.pose.pose.position.y, "z":i.pose.pose.pose.position.z }, "orientation":{ "x":i.pose.pose.pose.orientation.x, "y":i.pose.pose.pose.orientation.y, "z":i.pose.pose.pose.orientation.z, "w":i.pose.pose.pose.orientation.w }}}})
            # print(self.tagsDict)
            self.detected = True
        
    def setHomePose(self, data):
        home_t = tform.translation_matrix(np.array([mean([data[i]["pose"]["position"]["x"] for i in data]), mean([data[i]["pose"]["position"]["y"] for i in data]), mean([data[i]["pose"]["position"]["z"] for i in data])]))
        home_q = tform.translation_matrix(np.array([mean([data[i]["pose"]["orientation"]["x"] for i in data]), mean([data[i]["pose"]["orientation"]["y"] for i in data]), mean([data[i]["pose"]["orientation"]["z"] for i in data]), mean([data[i]["pose"]["orientation"]["w"] for i in data])]))
        self.home = np.dot(home_q, home_t)
        
    def main(self):
        rospy.Subscriber("/apriltag_detection/tag_detections", AprilTagDetectionArray, self.setDetections)
        rospy.Subscriber("/apriltag_detection/tag_detections_image", Image, self.updateImage)
        home_set_fl = False
        while not rospy.is_shutdown():
            event, values = self.window.read(timeout=20)
            
            if event == sg.WIN_CLOSED:
                break
            
            if event == "Reset":
                home_set_fl = False
            
            if event == "Set Home" and self.detected is True and home_set_fl is False:
                self.home_pose = self.tagsDict
                home_set_fl = True
                self.write = True
                self.setHomePose(self.home_pose)
                        
            if home_set_fl is True and self.write is True:
                with open(r'/home/yash/catkin_ws/src/shakebot_perception/config/perceptionCalib.yaml', 'w+') as file:
                    yaml.dump(self.home.tolist(), file)
                
        self.window.close()
        
if __name__=="__main__":
    # try:
    #     calibration()
    # except Exception as e:
    #     print(e)
    calibration()
