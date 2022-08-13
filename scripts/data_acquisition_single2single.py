import rospy
import time
import numpy as np
from statistics import mean
from tf import transformations as tform
from apriltag_ros.msg import AprilTagDetectionArray
import pandas as pd
import PySimpleGUI as sg
from cv_bridge import CvBridge
import cv2 as cv
from sensor_msgs.msg import Image, Imu
import message_filters as mf

class data_acquisition:
    def __init__(self):
        rospy.init_node("data_acquisition")
        self.tagsDict = {}
        self.data = {}
        self.detected = False
        self.record = False
        self.write = False
        self.initialize = False
        sg.theme("LightGreen")
        self.bridge = CvBridge()
        layout = [[sg.Image(key="-IMAGE-")],[sg.Button("Start Initialization")],[sg.Button("Start Recording"), sg.Button("Stop Recording")]]
        self.window = sg.Window("Calibrator", layout, finalize=True)
        # self.main()
        
    def updateImage(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img = cv.resize(img, [640, 480])
        imgbytes = cv.imencode(".png", img)[1].tobytes()
        self.window["-IMAGE-"].update(data=imgbytes)
    
    def setHomePose(self, tstamp, data):
        self.home = self.getCentroidPose(tstamp, data)
    
    def np2tq(self, nparr):
        t = tform.translation_from_matrix(nparr)
        q = tform.quaternion_from_matrix(nparr)
        return (t, q)
    
    def np2dictgen(self, tstamp, nparr):
        bed_ft, bed_fq = self.np2tq(nparr)
        rdict = {tstamp:{"pose":{ "position":{ "x": bed_ft[0], "y":bed_ft[1], "z":bed_ft[2] }, "orientation":{ "x":bed_fq[0], "y":bed_fq[1], "z":bed_fq[2], "w":bed_fq[3] }}}}
        return rdict
    
    def getCentroidPose(self, tstamp, tagDict):
        bed_t = tform.translation_matrix(np.array([mean([tagDict[tstamp][i]["pose"]["position"]["x"] for i in tagDict[tstamp]]), mean([tagDict[tstamp][i]["pose"]["position"]["y"] for i in tagDict[tstamp]]), mean([tagDict[tstamp][i]["pose"]["position"]["z"] for i in tagDict[tstamp]])]))
        bed_q = tform.quaternion_matrix(np.array([mean([tagDict[tstamp][i]["pose"]["orientation"]["x"] for i in tagDict[tstamp]]), mean([tagDict[tstamp][i]["pose"]["orientation"]["y"] for i in tagDict[tstamp]]), mean([tagDict[tstamp][i]["pose"]["orientation"]["z"] for i in tagDict[tstamp]]), mean([tagDict[tstamp][i]["pose"]["orientation"]["w"] for i in tagDict[tstamp]])]))
        pose = np.dot(bed_q, bed_t)
        return pose
    
    def np2dictgen(self, tstamp, nparr):
        bed_ft, bed_fq = self.np2tq(nparr)
        rdict = {tstamp:{"pose":{ "position":{ "x": bed_ft[0], "y":bed_ft[1], "z":bed_ft[2] }, "orientation":{ "x":bed_fq[0], "y":bed_fq[1], "z":bed_fq[2], "w":bed_fq[3] }}}}
        return rdict
    
    def imuMsg2dict(self, tstamp, msg):
        rdict = {tstamp:{"acceleration":{ "x": msg.linear_acceleration.x, "y":msg.linear_acceleration.y, "z":msg.linear_acceleration.z }}}
        return rdict
    
    def setBedPose(self, tag_msg, imu_msg):
        self.tagsDict={}
        
        if tag_msg.detections:
            self.tags = tag_msg.detections
            tstamp = str(tag_msg.header.stamp)
            for i in self.tags:
                self.tagsDict.update({tstamp:{i.id:{"pose":{ "position":{ "x": i.pose.pose.pose.position.x, "y":i.pose.pose.pose.position.y, "z":i.pose.pose.pose.position.z }, "orientation":{ "x":i.pose.pose.pose.orientation.x, "y":i.pose.pose.pose.orientation.y, "z":i.pose.pose.pose.orientation.z, "w":i.pose.pose.pose.orientation.w }}}}})
            # print(self.tagsDict)
            self.detected = True
            
            if self.initialize is True:
                time.sleep(1)
                self.setHomePose(tstamp, self.tagsDict)
                self.initialize = False
                print("Home position set success")
                
            if self.record is True:
                singlePosenp = self.getCentroidPose(tstamp, self.tagsDict)
                FPosenp = np.matmul(tform.inverse_matrix(self.home), singlePosenp)
                if tstamp not in self.data:
                    self.data[tstamp] = {}
                if "pose" not in self.data:
                    self.data[tstamp]["pose"] = {}
                if "acceleration" not in self.data:
                    self.data[tstamp]["acceleration"] = {}
                self.data[tstamp]["acceleration"] = self.imuMsg2dict(tstamp, imu_msg)[tstamp]["acceleration"]
                self.data[tstamp]["pose"] = self.np2dictgen(tstamp, FPosenp)[tstamp]["pose"]     
                
            if self.write is True:
                pdData = pd.DataFrame(self.data)            
                pdData.to_csv("./scripts/recorded.csv")
                self.write = False
                print("exiting")
                
    
    def main(self):
        print("inside main")
        tag_sub = mf.Subscriber("/apriltag_detection/tag_detections", AprilTagDetectionArray)
        img_sub = rospy.Subscriber("/apriltag_detection/tag_detections_image", Image, self.updateImage)
        imu_sub = mf.Subscriber("/imu_data", Imu)
        ts = mf.ApproximateTimeSynchronizer([tag_sub, imu_sub],20,5,1)
        ts.registerCallback(self.setBedPose)
        self.tic = time.perf_counter()
        while not rospy.is_shutdown():
            event, values = self.window.read(timeout=20)
            
            if event == sg.WIN_CLOSED:
                break
            
            if self.detected is True:
                if event == "Start Initialization":
                    self.initialize = True
                    
                if event == "Start Recording":
                    self.record = True
                    
                if event == "Stop Recording":
                    self.record = False
                    self.write = True

        self.window.close()
        rospy.spin()
    
if __name__=="__main__":
    try:
        s = data_acquisition()
        s.main()
    except Exception as e:
        print(e)
    # data_acquisition()
