#!/usr/bin/env python
# Software License Agreement (BSD License)
#
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
""" vision_node.py - Version 1.0 2010-12-28

    Modification of the ROS OpenCV Camshift example using cv_bridge and publishing the ROI
    coordinates to the /roi topic.   
"""

import roslib
roslib.load_manifest('pi_head_tracking_tutorial')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class vision_node:
    def __init__(self):
        rospy.init_node('vision_node')
                
        self.ROI = rospy.Publisher("roi", RegionOfInterest)
        
        """ Give the camera driver a moment to come up. """
        rospy.sleep(3)

        """ Create the display window """
        self.cv_window_name = "OpenCV Image"
        cv.NamedWindow(self.cv_window_name, 0)
        
        """ Create the cv_bridge object """
        self.bridge = CvBridge()
        
        """ Subscribe to the raw camera image topic """
        self.image_sub = rospy.Subscriber("input", Image, self.callback)
        
        """ Set up a smaller window to display the CamShift histogram. """
        cv.NamedWindow("Histogram", 0)
        cv.MoveWindow("Histogram", 700, 10)
        cv.SetMouseCallback(self.cv_window_name, self.on_mouse)

        self.drag_start = None      # Set to (x,y) when mouse starts dragtime
        self.track_window = None    # Set to rect when the mouse drag finishes

        self.hist = cv.CreateHist([180], cv.CV_HIST_ARRAY, [(0,180)], 1 )
        self.backproject_mode = False

    def callback(self, data):
        """ Convert the raw image to OpenCV format using the convert_image() helper function """
        cv_image = self.convert_image(data)
        
        """ Apply the CamShift algorithm using the do_camshift() helper function """
        cv_image = self.do_camshift(cv_image)
        
        """ Refresh the displayed image """
        cv.ShowImage(self.cv_window_name, cv_image)
        
        """ Toggle between the normal and back projected image if user hits the 'b' key """
        c = cv.WaitKey(7) % 0x100
        if c == 27:
            return
        elif c == ord("b"):
            self.backproject_mode = not self.backproject_mode
          
    def convert_image(self, ros_image):
        try:
          cv_image = self.bridge.imgmsg_to_cv(ros_image, "bgr8")
          return cv_image
        except CvBridgeError, e:
          print e

    def do_camshift(self, cv_image):
        """ Get the image size """
        image_size = cv.GetSize(cv_image)
        image_width = image_size[0]
        image_height = image_size[1]
        
        """ Convert to HSV and keep the hue """
        hsv = cv.CreateImage(image_size, 8, 3)
        cv.CvtColor(cv_image, hsv, cv.CV_BGR2HSV)
        self.hue = cv.CreateImage(image_size, 8, 1)
        cv.Split(hsv, self.hue, None, None, None)

        """ Compute back projection """
        backproject = cv.CreateImage(image_size, 8, 1)

        """ Run the cam-shift algorithm """
        cv.CalcArrBackProject( [self.hue], backproject, self.hist )
        cv.ShowImage("Backproject", backproject)
        if self.track_window and is_rect_nonzero(self.track_window):
            crit = ( cv.CV_TERMCRIT_EPS | cv.CV_TERMCRIT_ITER, 10, 1)
            (iters, (area, value, rect), track_box) = cv.CamShift(backproject, self.track_window, crit)
            self.track_window = rect

        """ If mouse is pressed, highlight the current selected rectangle
            and recompute the histogram """

        if self.drag_start and is_rect_nonzero(self.selection):
            sub = cv.GetSubRect(cv_image, self.selection)
            save = cv.CloneMat(sub)
            cv.ConvertScale(cv_image, cv_image, 0.5)
            cv.Copy(save, sub)
            x,y,w,h = self.selection
            cv.Rectangle(cv_image, (x,y), (x+w,y+h), (255,255,255))

            sel = cv.GetSubRect(self.hue, self.selection )
            cv.CalcArrHist( [sel], self.hist, 0)
            (_, max_val, _, _) = cv.GetMinMaxHistValue(self.hist)
            if max_val != 0:
                cv.ConvertScale(self.hist.bins, self.hist.bins, 255. / max_val)
        elif self.track_window and is_rect_nonzero(self.track_window):
            """ The width and height on the returned track_box seem to be exchanged--a bug in CamShift? """
            ((x, y), (w, h), angle) = track_box
            track_box = ((x, y), (h, w), angle)
            cv.EllipseBox(cv_image, track_box, cv.CV_RGB(255,0,0), 3, cv.CV_AA, 0)
            
            roi = RegionOfInterest()
            roi.x_offset = int(min(image_width, max(0, track_box[0][0] - track_box[1][0] / 2)))
            roi.y_offset = int(min(image_height, max(0, track_box[0][1] - track_box[1][1] / 2)))
            roi.width = int(track_box[1][0])
            roi.height = int(track_box[1][1])
            self.ROI.publish(roi)

        cv.ShowImage("Histogram", self.hue_histogram_as_image(self.hist))
        
        if not self.backproject_mode:
            return cv_image
        else:
            return backproject
        

    def hue_histogram_as_image(self, hist):
            """ Returns a nice representation of a hue histogram """
    
            histimg_hsv = cv.CreateImage( (320,200), 8, 3)
            
            mybins = cv.CloneMatND(hist.bins)
            cv.Log(mybins, mybins)
            (_, hi, _, _) = cv.MinMaxLoc(mybins)
            cv.ConvertScale(mybins, mybins, 255. / hi)
    
            w,h = cv.GetSize(histimg_hsv)
            hdims = cv.GetDims(mybins)[0]
            for x in range(w):
                xh = (180 * x) / (w - 1)  # hue sweeps from 0-180 across the image
                val = int(mybins[int(hdims * x / w)] * h / 255)
                cv.Rectangle( histimg_hsv, (x, 0), (x, h-val), (xh,255,64), -1)
                cv.Rectangle( histimg_hsv, (x, h-val), (x, h), (xh,255,255), -1)
    
            histimg = cv.CreateImage( (320,200), 8, 3)
            cv.CvtColor(histimg_hsv, histimg, cv.CV_HSV2BGR)
            return histimg

    def on_mouse(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
        if event == cv.CV_EVENT_LBUTTONUP:
            self.drag_start = None
            self.track_window = self.selection
        if self.drag_start:
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax - xmin, ymax - ymin)

def is_rect_nonzero(r):
    (_,_,w,h) = r
    return (w > 0) and (h > 0)           

def main(args):
      vn = vision_node()
      try:
        rospy.spin()
      except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
