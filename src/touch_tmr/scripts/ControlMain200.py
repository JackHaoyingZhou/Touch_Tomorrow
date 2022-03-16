#!/usr/bin/env python3
#Vxmax = 0.002387
#Vymax = 0.001933
#Vzmax = 0.0026818898

from __future__ import print_function
from touch_tmr.msg import hydraData
import PyKDL as pk
from math import *
import dvrk
import sys
import rospy
import numpy as np
import argparse
#from ambf_client import Client
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R
import time

# print with node id
def print_id(message):
    print('%s -> %s' % (rospy.get_caller_id(), message))

class ArmSideControl:

    # initialize collector flag and parameters
    # These parameters are essential for increasing the running rate
    V_flag = True
    V_start = pk.Frame()
    V_end = pk.Frame()
    V_data = pk.Frame()
    Count_V = 1

    E_flag = True
    E_start = 0
    E_end = 0
    E_data = 0
    Count_E = 1
    vtec_flag = False
    def get_data(self,data):
        self.E_data = float(data.jawRight)
        self.V_data.p = pk.Vector(float(data.poseRight[1])*0.38,
                                  float(data.poseRight[0])*0.4537,
                                  float(data.poseRight[2])*0.25)
        self.V_data.M = pk.Rotation.EulerZYX(float(data.poseRight[5]),
                                             float(data.poseRight[3]),  
                                             float(data.poseRight[4]))
        #self.clutch = int(data.data)


    #the configure function as all other tests
    def configure(self, robot_name, expected_interval):
        print_id('configuring dvrk_psm_test for %s' % robot_name)
        self.expected_interval = expected_interval
        self.arm = dvrk.psm(arm_name = robot_name,
                            expected_interval = expected_interval)
        

    def home(self):
        print_id('starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print_id('starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')
        # get current joints just to set size
        print_id('move to starting position')
        goal = np.copy(self.arm.setpoint_jp())
        # go to zero position, make sure 3rd joint is past cannula
        goal.fill(0)
        goal[2] = 0.12
        self.arm.move_jp(goal).wait()
        self.V_start = self.arm.measured_cp()
        self.V_ori = self.V_start
        self.eulori = np.array(list(self.V_ori.M.GetEulerZYX()))
        self.arm.jaw.open(0.8).wait()
        self.E_start = app.arm.jaw.setpoint_js()[2]
        x = np.array([0.0, 1.0])
        y = np.array([0.8, 0])
        self.coefficient_pos = np.polyfit(x, y, 1)       

    def move_jaw(self,effort):

        goal = np.copy(self.arm.jaw.setpoint_jp())
        delta_jaw_jp =  self.coefficient_pos[0] * effort + self.coefficient_pos[1]
        goal[0] = delta_jaw_jp
        self.arm.jaw.servo_jp(goal)

    def run(self):
        if self.clutch < 0:
            # The tracking data at 100hz,
            # should have two waypoint to fit 300 hz
            # Use flag and count to determine wether should update data or move to next waypoint
            #ts = time.time()
            goal_jaw = self.E_data - self.E_start
            goal_jaw = self.E_start + (self.step_E * self.Count_E)
            self.move_jaw(goal_jaw)
            self.step_p = (self.V_data.p + self.V_ori.p - self.V_start.p)
            if abs(self.step_p[0]) > 0.002:
                self.step_p[0] = abs(self.step_p[0])/self.step_p[0] * 0.002
            if abs(self.step_p[1]) > 0.0019:
                self.step_p[1] = abs(self.step_p[0])/self.step_p[0] * 0.0019
            if abs(self.step_p[2]) > 0.002:
                self.step_p[2] = abs(self.step_p[0])/self.step_p[0] * 0.002
            print(self.step_p)
            eulend = np.array(list(app.V_data.M.GetEulerZYX()))
            temp = eulend[1]
            eulend[1] = -eulend[2]
            eulend[2] = temp
            self.eulstart = np.array(list(self.V_start.M.GetEulerZYX()))
            if self.eulstart[2] <0:
                self.eulstart[2] = self.eulstart[2] + 2 * 3.1415926
            self.step_m = (eulend + self.eulori - self.eulstart)
            for step in self.step_m:
                if step < 0:
                    flag = -1
                else:
                    flag = 1
                if abs(step) > 0.17:
                    step = flag * 0.17


            self.goal_arm = self.V_start
            self.goal_arm.p = self.goal_arm.p + (self.step_p * self.Count_V)
            self.goal_arm.p = pk.Vector(round(self.goal_arm.p[0],6),round(self.goal_arm.p[1],6),round(self.goal_arm.p[2],6))
            g = self.eulstart + (self.step_m*self.Count_V)
            self.goal_arm.M = pk.Rotation.EulerZYX(g[0],g[1],g[2])
            self.arm.servo_cp(self.goal_arm)
            self.V_start = self.arm.measured_cp()
            #te = time.time()
            
        else:
            pass

if __name__ == '__main__':
    app = ArmSideControl()
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rospy.init_node('dvrk_arm_test')
    # strip ros arguments
    argv = rospy.myargv(argv=sys.argv)
    # Starting Subscriber loop
    rospy.Subscriber("/hydra_data", hydraData, app.get_data)
    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.005,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name
    
    app.configure(args.arm, args.interval)
    app.home()

    rate = rospy.Rate(200) # 200hz

    while not rospy.is_shutdown():
        try:
            app.run()
        except KeyboardInterrupt:
            break
        rate.sleep()

