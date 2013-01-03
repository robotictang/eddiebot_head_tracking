#!/usr/bin/env python
# Software License Agreement (BSD License)
# Copyright (c) 2010, Willow Garage Inc. 
# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
"""
    head_track_node.py - Version 1.0 2010-12-28
"""

import roslib; roslib.load_manifest('pi_head_tracking_tutorial')
import rospy
from sensor_msgs.msg import JointState, RegionOfInterest, CameraInfo
from math import radians

class head_track():
    def __init__(self):
        rospy.init_node("head_track")
        rospy.on_shutdown(self.shutdown)
        
        """ Publish the movement commands on the /cmd_joints topic using the 
            JointState message type. """
        self.head_pub = rospy.Publisher("/cmd_joints", JointState)
        
        self.rate = rospy.get_param("~rate", 10)
        
        """ The pan/tilt thresholds indicate how many pixels the ROI needs to be off-center
            before we make a movement. """
        self.pan_threshold = int(rospy.get_param("~pan_threshold", 5))
        self.tilt_threshold = int(rospy.get_param("~tilt_threshold", 5))
        
        """ The k_pan and k_tilt parameter determine how responsive the servo movements are.
            If these are set too high, oscillation can result. """
        self.k_pan = rospy.get_param("~k_pan", 7.0)
        self.k_tilt = rospy.get_param("~k_tilt", 5.0)
        
        self.max_pan = rospy.get_param("~max_pan", radians(145))
        self.min_pan = rospy.get_param("~min_pan", radians(-145))
        self.max_tilt = rospy.get_param("~max_tilt", radians(90))
        self.min_tilt = rospy.get_param("~min_tilt", radians(-90))
        
        r = rospy.Rate(self.rate) 
        
        self.head_cmd = JointState()
        self.joints = ["head_pan_joint", "head_tilt_joint"]
        self.head_cmd.name = self.joints
        self.head_cmd.position = [0, 0]
        self.head_cmd.velocity = [1, 1]
        self.head_cmd.header.stamp = rospy.Time.now()
        self.head_cmd.header.frame_id = 'head_pan_joint'
    
        """ Center the head and pan servos at the start. """
        for i in range(3):
            self.head_pub.publish(self.head_cmd)
            rospy.sleep(1)
        
        self.tracking_seq = 0
        self.last_tracking_seq = -1
        
        rospy.Subscriber('roi', RegionOfInterest, self.setPanTiltSpeeds)
        rospy.Subscriber('camera_info', CameraInfo, self.getCameraInfo)
        
        while not rospy.is_shutdown():
            """ Publish the pan/tilt movement commands. """
            self.head_cmd.header.stamp = rospy.Time.now()
            self.head_cmd.header.frame_id = 'head_pan_joint'
            if self.last_tracking_seq == self.tracking_seq:
                self.head_cmd.velocity = [0.0001, 0.0001]
            else:
                self.last_tracking_seq = self.tracking_seq
            self.head_pub.publish(self.head_cmd)
            r.sleep()
    
    def setPanTiltSpeeds(self, msg):
        """ When OpenCV loses the ROI, the message stops updating.  Use this counter to
            determine when it stops. """
        self.tracking_seq += 1
        
        """ Check to see if we have lost the ROI. """
        if msg.width == 0 or msg.height == 0 or msg.width > self.image_width / 2 or \
                msg.height > self.image_height / 2:
            self.head_cmd.velocity = [0, 0]
            return

        """ Compute the center of the ROI """
        COG_x = msg.x_offset + msg.width / 2 - self.image_width / 2
        COG_y = msg.y_offset + msg.height / 2 - self.image_height / 2
          
        """ Pan the camera only if the displacement of the COG exceeds the threshold. """
        if abs(COG_x) > self.pan_threshold:
            """ Set the pan speed proportion to the displacement of the horizontal displacement
                of the target. """
            self.head_cmd.velocity[0] = self.k_pan * abs(COG_x) / float(self.image_width)
               
            """ Set the target position to one of the min or max positions--we'll never
                get there since we are tracking using speed. """
            if COG_x > 0:
                self.head_cmd.position[0] = self.min_pan
            else:
                self.head_cmd.position[0] = self.max_pan
        else:
            self.head_cmd.velocity[0] = 0.0001
        
        """ Tilt the camera only if the displacement of the COG exceeds the threshold. """
        if abs(COG_y) > self.tilt_threshold:
            """ Set the tilt speed proportion to the displacement of the vertical displacement
                of the target. """
            self.head_cmd.velocity[1] = self.k_tilt * abs(COG_y) / float(self.image_height)
            
            """ Set the target position to one of the min or max positions--we'll never
                get there since we are tracking using speed. """
            if COG_y < 0:
                self.head_cmd.position[1] = self.min_tilt
            else:
                self.head_cmd.position[1] = self.max_tilt
        else:
            self.head_cmd.velocity[1] = 0.0001
            
    def getCameraInfo(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
        
    def shutdown(self):
        rospy.loginfo("Shutting down head tracking node...")         
                   
if __name__ == '__main__':
    try:
        head_track()
    except rospy.ROSInterruptException:
        rospy.loginfo("Head tracking node is shut down.")




