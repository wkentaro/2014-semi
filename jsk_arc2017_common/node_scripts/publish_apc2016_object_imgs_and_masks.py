#!/usr/bin/env python

import glob
import os.path as osp
import skimage.io
import rospy
import yaml
from sensor_msgs.msg import Image

import cv_bridge


rospy.init_node('publish_apc2016_object_imgs_and_masks')

templates_dir = '/home/wkentaro/Projects/mvtk/mvtk/experiments/fewshot_classification_arc2017/data/templates'

pub_img = rospy.Publisher('~output/image', Image, queue_size=1)
pub_mask = rospy.Publisher('~output/mask', Image, queue_size=1)

bridge = cv_bridge.CvBridge()

target_names = osp.join(templates_dir, 'target_names.yaml')
target_names = yaml.load(open(target_names))
for cls_name in target_names:
    cls_dir = osp.join(templates_dir, cls_name)
    for img_file in glob.glob(osp.join(cls_dir, '*.png')):
        dirname, basename = osp.split(img_file)
        mask_file = osp.join(dirname, 'masks', basename)
        img = skimage.io.imread(img_file)
        mask = skimage.io.imread(mask_file)

        now = rospy.Time.now()
        imgmsg = bridge.cv2_to_imgmsg(img, encoding='rgb8')
        mask_msg = bridge.cv2_to_imgmsg(mask, encoding='mono8')
        imgmsg.header.frame_id = basename
        imgmsg.header.stamp = now
        mask_msg.header.frame_id = basename
        mask_msg.header.stamp = now

        pub_img.publish(imgmsg)
        pub_mask.publish(mask_msg)
        rospy.sleep(2)
