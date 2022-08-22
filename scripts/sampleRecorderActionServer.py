#!/usr/bin/python3
import rospy
import actionlib
from shakebot_perception.msg import recorder_automationResult, recorder_automationAction

class ActionServer():
    def __init__(self):
        self.a_server = actionlib.SimpleActionServer("shakebot_recorder_as",recorder_automationAction, execute_cb=self.execute_cb, auto_start=False)
        self.a_server.start()
        self.result = recorder_automationResult()
        
    def execute_cb(self, goal):
        success=True
        if goal.recorder_state is True:
            self.result.recorder_result = True
            
        if success:
            self.a_server.set_succeeded(self.result)
            
if __name__ == "__main__":
    rospy.init_node("action_server")
    s = ActionServer()
    rospy.spin()