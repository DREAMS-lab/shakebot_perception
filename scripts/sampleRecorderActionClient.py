#!/usr/bin/python3
import rospy
import actionlib
from shakebot_perception.msg import recorder_automationAction, recorder_automationGoal

def call_server():
    client = actionlib.SimpleActionClient("shakebot_recorder_as", recorder_automationAction)
    client.wait_for_server()
    goal = recorder_automationGoal()
    goal.recorder_state = True
    client.send_goal(goal)
    client.wait_for_result()
    result=client.get_result()
    return result

if __name__ == "__main__":
    rospy.init_node("shakebot_recorder_action_client")
    result = call_server()
    print(f"the result is {result}")