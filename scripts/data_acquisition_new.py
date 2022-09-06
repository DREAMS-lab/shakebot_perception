#!/usr/bin/python3
import rospy
import time
import os
import numpy as np
from statistics import mean
from tf import transformations as tform
from apriltag_ros.msg import AprilTagDetectionArray
import pandas as pd
from sensor_msgs.msg import Image, Imu
import message_filters as mf
import actionlib
from shakebot_perception.msg import recorder_automationResult, recorder_automationAction
import json
from velocity_calc import velocity_calc

class data_acquisition:
    def __init__(self):
        rospy.init_node("data_acquisition")
        self.tagsDict = {}
        self.data = {}
        self.home = {}
        self.detected = False
        self.record = False
        self.write = False
        self.initialize = False
        self.a_server = actionlib.SimpleActionServer("shakebot_recorder_as",recorder_automationAction, execute_cb=self.execute_cb, auto_start=False)
        self.a_server.start()
        self.result = recorder_automationResult()
    
    def setHomePose(self, data):
        for i in data:
            self.home.update(data)
        
    def dict2np(self, dictionary):
        _t = tform.translation_matrix(np.array([dictionary["pose"]["position"]["x"], dictionary["pose"]["position"]["y"], dictionary["pose"]["position"]["z"]]))
        _q = tform.quaternion_matrix(np.array([dictionary["pose"]["orientation"]["x"], dictionary["pose"]["orientation"]["y"], dictionary["pose"]["orientation"]["z"], dictionary["pose"]["orientation"]["w"]]))
        pose = np.dot(_q, _t)
        return pose
    
    def np2tq(self, nparr):
        t = tform.translation_from_matrix(nparr)
        q = tform.quaternion_from_matrix(nparr)
        return (t, q)
    
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
    
    def imuMsg2dict(self, msg):
        rdict = {"acceleration":{ "x": msg.linear_acceleration.x, "y":msg.linear_acceleration.y, "z":msg.linear_acceleration.z }}
        return rdict
    
    def fetchImu(self, imu_msg):
        if self.write is False:
            if self.record is True:
                tstamp = str(imu_msg.header.stamp)
                if tstamp not in self.data:
                    self.data[tstamp] = {}
                self.data[tstamp]["acceleration"]=self.imuMsg2dict(imu_msg)["acceleration"]
        else:
            pass
    
    def setBedPose(self, tag_msg):
        if self.write is False:
            self.tagsDict={}
            
            if tag_msg.detections:
                self.tags = tag_msg.detections
                tstamp = str(tag_msg.header.stamp)
                for i in self.tags:
                    self.tagsDict.update({i.id:{"pose":{ "position":{ "x": i.pose.pose.pose.position.x, "y":i.pose.pose.pose.position.y, "z":i.pose.pose.pose.position.z }, "orientation":{ "x":i.pose.pose.pose.orientation.x, "y":i.pose.pose.pose.orientation.y, "z":i.pose.pose.pose.orientation.z, "w":i.pose.pose.pose.orientation.w }}}})
                self.detected = True
                
                if self.initialize is True:
                    time.sleep(1)
                    self.setHomePose(self.tagsDict)
                    self.initialize = False
                    rospy.loginfo("Home position set success!")
                
                if self.record is True:
                    FPose = {}
                    for i in self.home:
                        if i in self.tagsDict:
                            FPose.update(self.np2dictspec(i, np.matmul(tform.inverse_matrix(self.dict2np(self.home[i])), self.dict2np(self.tagsDict[i]))))
                        
                    singlePosenp = self.getCentroidPose(FPose)
                    if tstamp not in self.data:
                        self.data[tstamp] = {}
                    self.data[tstamp]["pose"] = self.np2dictgen(tstamp, singlePosenp)[tstamp]["pose"]

                # if self.write is True:
                #     pdData = pd.DataFrame(self.data)
                #     rospy.loginfo("saving file.....")
                #     pdData.to_csv("~/catkin_ws/src/shakebot_perception/scripts/recorded.csv")
                #     self.write = False
                #     rospy.loginfo("exiting")
        else:
            pass
    
    def execute_cb(self, goal):
        success=True
        published = True
        self.result.recorder_result = False
        self.goal_recorder_state = goal.recorder_state
        while published:
            if self.a_server.is_preempt_requested():
                success = False
                break
            
            if success:
                if self.goal_recorder_state is True:
                    rospy.loginfo("started recording")
                    self.initialize = True
                    self.record = True
                    self.result.recorder_result = True
                    self.a_server.set_succeeded(self.result)
                    published = False
                    
                if self.goal_recorder_state is False:
                    rospy.loginfo("stopped recording")
                    self.record = False
                    self.write = True
                    if self.write is True:
                        rospy.loginfo("saving file.....")
                        fname = time.strftime("recorded_%m_%d_%y_%H_%M_%S", time.localtime())+".json"
                        outfile = "/home/"+os.environ.get("USERNAME")+"/catkin_ws/src/shakebot_perception/scripts/" + fname
                        # print(self.data)
                        with open(outfile, "w") as f:
                            json_object = json.dumps(self.data, indent=4)
                            f.write(json_object)
                            f.close()
                            
                        # pdData.to_csv("~/catkin_ws/src/shakebot_perception/scripts/"+fname)
                        self.write = False
                        self.data = {}
                        rospy.loginfo("save complete.")
                    self.result.recorder_result = True
                    self.a_server.set_succeeded(self.result)
                    published = False
                    rospy.loginfo("Plotting recorded data.")
                    s = velocity_calc()
                    s.main()
                    rospy.loginfo("Publish goal once again to start recording.")
    
    def main(self):
        rospy.Subscriber("/apriltag_detection/tag_detections", AprilTagDetectionArray, self.setBedPose)
        rospy.Subscriber("/imu_data", Imu, self.fetchImu)
        # tag_sub = mf.Subscriber("/apriltag_detection/tag_detections", AprilTagDetectionArray)
        # imu_sub = mf.Subscriber("/imu_data", Imu)
        # ts = mf.ApproximateTimeSynchronizer([tag_sub, imu_sub],20,5,1)
        # ts.registerCallback(self.setBedPose)
        rospy.loginfo("publish goal to start recording!!")
        rospy.spin()
        
    
if __name__=="__main__":
    # try:
    #     s = data_acquisition()
    #     s.main()
    # except Exception as e:
    #     print(e)
    s = data_acquisition()
    s.main()
    # print(time.strftime("recorded_%m_%d_%y_%H_%M_%S", time.localtime())+".csv")
