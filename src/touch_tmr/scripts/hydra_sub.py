#!/usr/bin/env python
# //==============================================================================
#     \author    <hzhou6@wpi.edu>
#     \author    Haoying Zhou
#     \version   1.0
# */
# //==============================================================================
import numpy as np
from razer_hydra.msg import Hydra
from touch_tmr.msg import hydraData
import rospy
import time
from scipy.spatial.transform import Rotation as Rot

class razer_Device:
    def __init__(self, name='hydra_calib'):
        self.pose_topic_name = name
        self._active = False
        self._scale = [1.0, 1.0, 1.0]
        self.jaw_scale = 1.0
        self.clutch_left = 1
        self.clutch_right = 1
        self.base_mtx_left = np.eye(3)
        self.base_mtx_right = np.eye(3)
        self.base_pos_left = [0.0,0.0,0.0]
        self.base_pos_right = [0.0,0.0,0.0]
        self.pose_left = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.pose_right = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.jaw_left = 0.0
        self.jaw_right = 0.0

        self.reset_button = False
        rospy.init_node('TouchTmr_hydra', anonymous=True)
        self.pub_topic= rospy.Publisher('hydra_data', hydraData, queue_size=1)
        self._pose_sub = rospy.Subscriber(self.pose_topic_name, Hydra, self.pose_cb, queue_size=1)

    def set_base_frame(self, msg):
        if msg.paddles[0].buttons[0] == True:
            mtx_left = Rot.from_quat([msg.paddles[0].transform.rotation.x,
                                                msg.paddles[0].transform.rotation.y,
                                                msg.paddles[0].transform.rotation.z,
                                                msg.paddles[0].transform.rotation.w])
            self.base_mtx_left = mtx_left.as_matrix()
            self.base_pos_left = [msg.paddles[0].transform.translation.x,
                                  msg.paddles[0].transform.translation.y,
                                  msg.paddles[0].transform.translation.z]
            self.reset_button = True

        if msg.paddles[1].buttons[0] == True:
            mtx_right = Rot.from_quat([msg.paddles[1].transform.rotation.x,
                                      msg.paddles[1].transform.rotation.y,
                                      msg.paddles[1].transform.rotation.z,
                                      msg.paddles[1].transform.rotation.w])
            self.base_mtx_right = mtx_right.as_matrix()
            self.base_pos_right = [msg.paddles[1].transform.translation.x,
                                  msg.paddles[1].transform.translation.y,
                                  msg.paddles[1].transform.translation.z]
            self.reset_button = True

    def get_clutch(self, msg):
        if msg.paddles[0].buttons[5] == True:
            time.sleep(0.1)
            if msg.paddles[0].buttons[5] == True:
                time.sleep(0.08)
                if msg.paddles[0].buttons[5] == True:
                    self.clutch_left = -1*self.clutch_left

        if msg.paddles[1].buttons[5] == True:
            time.sleep(0.1)
            if msg.paddles[1].buttons[5] == True:
                time.sleep(0.08)
                if msg.paddles[1].buttons[5] == True:
                    self.clutch_right = -1*self.clutch_right


    def set_scale(self, scale):
        self._scale = scale

    def get_scale(self):
        return self._scale

    def hydra_msg_read(self, msg_hydra):
        self.set_base_frame(msg_hydra)
        msg_left = msg_hydra.paddles[0].transform
        msg_right = msg_hydra.paddles[1].transform
        pos_left = [msg_left.translation.x,
                    msg_left.translation.y,
                    msg_left.translation.z]
        pos_right = [msg_right.translation.x,
                    msg_right.translation.y,
                    msg_right.translation.z]
        rot_left = Rot.from_quat([msg_left.rotation.x,
                             msg_left.rotation.y,
                             msg_left.rotation.z,
                             msg_left.rotation.w])
        rot_right = Rot.from_quat([msg_right.rotation.x,
                                  msg_right.rotation.y,
                                  msg_right.rotation.z,
                                  msg_right.rotation.w])
        return pos_left, pos_right, rot_left.as_matrix(), rot_right.as_matrix()

    def pose_cb(self, msg):
        self._active = True
        pose_left = []
        pose_right = []
        self.get_clutch(msg)
        pos_left, pos_right, rot_left, rot_right = self.hydra_msg_read(msg)
        for i_index in range(3):
            pos_left[i_index] = self._scale[i_index] * (pos_left[i_index] - self.base_pos_left[i_index])
            pos_right[i_index] = self._scale[i_index] * (pos_right[i_index] - self.base_pos_right[i_index])
            pose_left.append(pos_left[i_index])
            pose_right.append(pos_right[i_index])

        mtx_left = Rot.from_matrix(np.dot(np.transpose(self.base_mtx_left),rot_left))
        mtx_right = Rot.from_matrix(np.dot(np.transpose(self.base_mtx_right), rot_right))

        ori_left = mtx_left.as_euler('xyz',degrees=False)
        ori_right = mtx_right.as_euler('xyz',degrees=False)

        for j_index in range(3):
            pose_left.append(ori_left[j_index])
            pose_right.append(ori_right[j_index])

        

        self.pose_left = pose_left
        self.pose_right = pose_right
        self.jaw_left = self.jaw_scale * msg.paddles[0].trigger
        self.jaw_right = self.jaw_scale * msg.paddles[1].trigger

    def get_hydra_data(self):
        return self.pose_left, self.pose_right, self.jaw_left, self.jaw_right, self.clutch_left, self.clutch_right

    def sub_pub(self, feq):
        rate = rospy.Rate(feq)
        while not rospy.is_shutdown():
            pose_left, pose_right, jaw_left, jaw_right,clutch_left, clutch_right = self.get_hydra_data()
            pub_data = hydraData()
            pub_data.poseLeft = pose_left
            pub_data.poseRight = pose_right
            pub_data.jawLeft = jaw_left
            pub_data.jawRight = jaw_right
            pub_data.clutchLeft = clutch_left
            pub_data.clutchRight = clutch_right
            self.pub_topic.publish(pub_data)
            rate.sleep()

if __name__ == '__main__':
    dev = razer_Device()
    dev.sub_pub(200)
