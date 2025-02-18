#!/usr/bin/env python

from jsk_arc2017_common.msg import WorkOrder
from jsk_arc2017_common.msg import WorkOrderArray
import json
import operator
import os.path as osp
import rospy


class WorkOrderPublisher(object):

    abandon_items = [
        'measuring_spoons',
        'mesh_cup'
        ]

    def __init__(self):
        json_dir = rospy.get_param('~json_dir', None)
        self.rate = rospy.get_param('~rate', 1.0)
        if json_dir is None:
            rospy.logerr('must set json dir path to ~json_dir')
            return
        location_path = osp.join(json_dir, 'item_location_file.json')
        with open(location_path) as location_f:
            bins = json.load(location_f)['bins']
        order_path = osp.join(json_dir, 'order_file.json')
        with open(order_path) as order_f:
            orders = json.load(order_f)['orders']
        box_path = osp.join(json_dir, 'box_sizes.json')
        with open(box_path) as box_f:
            boxes = json.load(box_f)['boxes']

        self.item_location = {}
        for bin_ in bins:
            bin_id = bin_['bin_id']
            for item_name in bin_['contents']:
                self.item_location[item_name] = bin_id

        box_sizes = {}
        for box in boxes:
            size_id = box['size_id']
            box_sizes[size_id] = reduce(operator.mul, box['dimensions'])
        size_ids = [order['size_id'] for order in orders]
        sorted_size_ids = sorted(size_ids, key=lambda x: box_sizes[x])

        self.cardboard_ids = {}
        for i, size_id in enumerate(sorted_size_ids):
            self.cardboard_ids[size_id] = 'ABC'[i]

        larm_orders = orders[:2]
        rarm_orders = orders[2:3]
        self.larm_msg = self._generate_msg(larm_orders)
        self.rarm_msg = self._generate_msg(rarm_orders)
        self.larm_pub = rospy.Publisher(
            '~left_hand', WorkOrderArray, queue_size=1)
        self.rarm_pub = rospy.Publisher(
            '~right_hand', WorkOrderArray, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self._publish_msg)

    def _generate_msg(self, orders):
        order_msgs = []
        for order in orders:
            size_id = order['size_id']
            for target_item in order['contents']:
                if target_item in self.abandon_items:
                    continue
                order_msg = WorkOrder()
                order_msg.bin = self.item_location[target_item]
                order_msg.item = target_item
                order_msg.box = self.cardboard_ids[size_id]
                order_msgs.append(order_msg)
        order_array_msg = WorkOrderArray()
        order_array_msg.orders = order_msgs
        return order_array_msg

    def _publish_msg(self, event):
        self.larm_pub.publish(self.larm_msg)
        self.rarm_pub.publish(self.rarm_msg)

if __name__ == '__main__':
    rospy.init_node('work_order_publisher')
    work_order_publisher = WorkOrderPublisher()
    rospy.spin()
