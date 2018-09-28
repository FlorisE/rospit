"""
Script that launches useful nodes and services for executing Physical Integration Tests
"""
import rospy
from rospit.ros import ROSTestRunnerNode


if __name__ == '__main__':
    ROSTestRunnerNode()
    rospy.spin()
