#!/usr/bin/env python

import moveit_commander
import rospy

from jsk_recognition_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import PoseStamped


class APC2016RobotInterface(object):

    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.init_robot()
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)
        self.update_shelf()

    def init_robot(self):
        self.robot.arm.remember_joint_values(
            'reset_pose',
            [1.320, 1.400, -0.200, 1.720, 0, 1.660, 0])

    def _shelf_cb(self, boxes_msg):
        self.shelf_box = boxes_msg.boxes[0]

    def update_shelf(self):
        self.shelf_box = None
        sub = rospy.Subscriber(
            'transformable_shelf/output/boxes',
            BoundingBoxArray, self._shelf_cb)

        rospy.loginfo('Waiting for shelf box.')
        while self.shelf_box is None:
            rospy.sleep(1)

        pose = PoseStamped(header=self.shelf_box.header,
                           pose=self.shelf_box.pose)
        self.scene.add_box(
            'shelf', pose,
            (self.shelf_box.dimensions.x,
             self.shelf_box.dimensions.y,
             self.shelf_box.dimensions.z))
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('look_around_bin_main')
    ri = APC2016RobotInterface()
