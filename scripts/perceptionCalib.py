#!/usr/bin/python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from tf import transformations as tform
from shakebot_motion.msg import calib_msg
import time
import numpy as np
from statistics import mean
from std_msgs.msg import Float64
import os

class perceptCalib:
    def __init__(self):
        rospy.init_node("perceptionCalib")
        self.initialize = True
        self.tagsDict = {}
        self.data = {}
        self.home = {}
        self.state = ""
        self.capture = False
        self.count = 1  # defaults to 1
        self.write = False
        rospy.Subscriber("/calibration_parameters", calib_msg, self.capturePos)
        rospy.Subscriber("/apriltag_detection/tag_detections", AprilTagDetectionArray, self.setBedPose)
        self.pub = rospy.Publisher("bed_displacement", Float64, queue_size=10)
        
    def np2tq(self, nparr):
        t = tform.translation_from_matrix(nparr)
        q = tform.quaternion_from_matrix(nparr)
        return (t, q)
        
    def dict2np(self, dictionary):
        _t = tform.translation_matrix(np.array([dictionary["pose"]["position"]["x"], dictionary["pose"]["position"]["y"], dictionary["pose"]["position"]["z"]]))
        _q = tform.quaternion_matrix(np.array([dictionary["pose"]["orientation"]["x"], dictionary["pose"]["orientation"]["y"], dictionary["pose"]["orientation"]["z"], dictionary["pose"]["orientation"]["w"]]))
        pose = np.dot(_q, _t)
        return pose
    
    def getCentroidPose(self, tagDict):
        bed_t = tform.translation_matrix(np.array([mean([tagDict[i]["pose"]["position"]["x"] for i in tagDict]), mean([tagDict[i]["pose"]["position"]["y"] for i in tagDict]), mean([tagDict[i]["pose"]["position"]["z"] for i in tagDict])]))
        bed_q = tform.quaternion_matrix(np.array([mean([tagDict[i]["pose"]["orientation"]["x"] for i in tagDict]), mean([tagDict[i]["pose"]["orientation"]["y"] for i in tagDict]), mean([tagDict[i]["pose"]["orientation"]["z"] for i in tagDict]), mean([tagDict[i]["pose"]["orientation"]["w"] for i in tagDict])]))
        pose = np.dot(bed_q, bed_t)
        return pose
    
    def np2dictspec(self,  id, nparr):
        bed_ft, bed_fq = self.np2tq(nparr)
        rdict = {id:{"pose":{ "position":{ "x": bed_ft[0], "y":bed_ft[1], "z":bed_ft[2] }, "orientation":{ "x":bed_fq[0], "y":bed_fq[1], "z":bed_fq[2], "w":bed_fq[3] }}}}
        return rdict
    
    def np2dictgen(self, tstamp, nparr):
        bed_ft, bed_fq = self.np2tq(nparr)
        rdict = {tstamp:{"pose":{ "position":{ "x": bed_ft[0], "y":bed_ft[1], "z":bed_ft[2] }, "orientation":{ "x":bed_fq[0], "y":bed_fq[1], "z":bed_fq[2], "w":bed_fq[3] }}}}
        return rdict
    
    def setHomePose(self, data):
        self.home.update(data)
    
    def setBedPose(self, tag_msg):
        if self.write is False:
            self.tagsDict={}
            if tag_msg.detections:
                self.tags = tag_msg.detections
                tstamp = str(tag_msg.header.stamp)
                for i in self.tags:
                    self.tagsDict.update({i.id:{"pose":{ "position":{ "x": i.pose.pose.pose.position.x, "y":i.pose.pose.pose.position.y, "z":i.pose.pose.pose.position.z }, "orientation":{ "x":i.pose.pose.pose.orientation.x, "y":i.pose.pose.pose.orientation.y, "z":i.pose.pose.pose.orientation.z, "w":i.pose.pose.pose.orientation.w }}}})
                
                if self.initialize is True:
                    time.sleep(1)
                    self.setHomePose(self.tagsDict)
                    self.initialize = False
                    rospy.loginfo("Home position set success!")
                if self.capture is True:
                    FPose = {}
                    for i in self.home:
                        if i in self.tagsDict:
                            FPose.update(self.np2dictspec(i, np.matmul(tform.inverse_matrix(self.dict2np(self.home[i])), self.dict2np(self.tagsDict[i]))))
                        
                    singlePosenp = self.getCentroidPose(FPose)
                    if self.state not in self.data:
                        self.data[self.state] = {}
                    self.data[self.state]["pose"] = self.np2dictgen(tstamp, singlePosenp)[tstamp]["pose"]
                    self.data[self.state]["bed_length"] = self.bedlength
                    self.capture = False
                    rospy.loginfo(f"Captured {self.state} state")
                    if self.state == "last":
                        self.startPublish()

        else:
            pass
        
    def calculateAndSave(self):
        rospy.loginfo("calculating and writing parameter")
        M = np.matrix([[abs(self.data["middle"]["pose"]["position"]["x"] - self.data["left"]["pose"]["position"]["x"]), abs(self.data["right"]["pose"]["position"]["x"] - self.data["left"]["pose"]["position"]["x"])]])
        D = np.matrix([[self.data["middle"]["bed_length"], self.data["right"]["bed_length"]]])
        self.alpha = np.matmul(np.matmul(D,M.transpose()),tform.inverse_matrix(np.matmul(M,M.transpose())))
        outf = "/home/"+os.environ.get("USERNAME")+"/catkin_ws/src/shakebot_perception/config/perceptionCalib.yaml"
        with open(outf,"w+") as f:
            f.write(str(self.alpha))
            f.close()
        self.write = False
        self.count +=1
        
    def setParams(self, state, bedLength):
        self.state = state
        self.capture = True
        self.bedlength = bedLength
        self.count+=1
        
    def startPublish(self):
        rospy.loginfo("starting publishing")
        pub_msg = Float64()
        pub_msg = (self.alpha[0][0] * abs(self.data["last"]["pose"]["position"]["x"] - self.data["initial"]["pose"]["position"]["x"]))/10
        # pub_msg = (393.72134561 * abs(self.data["last"]["pose"]["position"]["x"] - self.data["initial"]["pose"]["position"]["x"]))/10
        self.pub.publish(pub_msg)
        rospy.spin()
    
    def capturePos(self, msg):
        if msg.left_ls and self.count == 1:
            self.setParams("left", 0)
        elif msg.right_ls and self.count == 2:
            self.setParams("right", msg.bed_length)
        elif msg.bed_position and self.count == 3:
            self.setParams("middle", msg.bed_position)
        elif self.count == 4 and len(self.data) == 3:
            self.write = True
            self.calculateAndSave()
        elif self.count == 5 and msg.pgd_calib_trigger == True:
            self.setParams("initial", msg.bed_length)
        elif self.count == 6 and msg.pgd_calib_trigger == False:
            self.setParams("last", msg.bed_length)
            
    def main(self):
        rospy.spin()
     
if __name__=="__main__":
    try:
        s = perceptCalib()
        s.main()
    except Exception as e:
        print(e)
        s = perceptCalib()
    # s.main()
    # s = perceptCalib()
    # s.main()