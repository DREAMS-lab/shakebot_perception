import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from tf import transformations as tform
from shakebot_perception.msg import calib_msg
import time
import numpy as np
from data_acquisition_new import data_acquisition

class perceptCalib:
    def __init__(self):
        rospy.init_node("perceptionCalib")
        self.dict2np = data_acquisition.dict2np
        self.np2dictspec = data_acquisition.np2dictspec
        self.np2dictgen = data_acquisition.np2dictgen
        self.getCentroidPose = data_acquisition.getCentroidPose
        self.initialize = True
        self.tagsDict = {}
        self.data = {}
        self.home = {}
        self.state = ""
        self.capture = False
        self.count = 1
        self.write = False
    
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
        else:
            pass
        
    def calculateAndSave(self):
        M = np.matrix([[self.data.left.pose.position.x - self.data.middle.pose.position.x, self.data.left.pose.position.x - self.data.right.pose.position.x]])
        D = np.matrix([[self.data.middle.bed_length, self.data.right.bed_length]])
        alpha = np.matmul(np.matmul(D,M.transpose),tform.inverse_matrix(np.matmul(M,M.transpose)))
        with open("/home/yash/catkin_ws/src/shakebot_perception/config/perceptionCalib.yaml","w") as f:
            f.write(alpha)
            f.close()
        self.write = False
        
    def setParams(self, state, bedLength):
        self.state = state
        self.capture = True
        self.count+=1
        self.bedlength = bedLength
        
    def capturePos(self, msg):
        if msg.left_ls and self.count == 1:
            self.setParams("left", 0)
        elif msg.right_ls and self.count == 2:
            self.setParams("right", msg.bed_length)
        elif msg.bed_position and self.count == 3:
            self.setParams("right", msg.bed_position)
        elif self.count==4 and len(self.data)==3:
            self.write = True
            self.calculateAndSave()
   
    def main(self):
        rospy.Subscriber("calibration_parameters", calib_msg, self.capturePos)
        rospy.Subscriber("/apriltag_detection/tag_detections", AprilTagDetectionArray, self.setBedPose)
        rospy.spin()
     
if __name__=="__main__":
    try:
        s = perceptCalib()
        s.main()
    except Exception as e:
        print(e)
        
    # s = perceptCalib()
    # s.main()