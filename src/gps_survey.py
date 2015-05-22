#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, EJ Kreinar
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
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
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

import rospy
import roslib
import sys
import collections

from sensor_msgs.msg import NavSatFix

# syntax to start the node: 
#   rosrun cutter_survey gps_survey.py _filename:=filenamehere

class Surveyor:
    def __init__(self):
        # Start the node
        rospy.init_node('gps_survey')
        topic_in  = rospy.get_param('~topic_in','/gps_fix')
        filename = rospy.get_param('~filename','survey.txt')
        self.window_size = rospy.get_param('~window_size',10) # magic number 10 would be (GPScallbackHz)*(windowlengthinseconds), assuming 10Hz and 1-second window
        self.file = roslib.packages.get_pkg_dir('snowmower_bot_drivers')+'/survey/'+filename
        
        rospy.loginfo('Subscribing to: '+topic_in)
        rospy.loginfo('Output file will be: '+ self.file)
        
        # Initialize
        self.coords = (0.0, 0.0, 0.0)
        self.average = (0.0, 0.0, 0.0)
        self.history = collections.deque(maxlen=self.window_size)  
        self.i = 0 # start a counter

        # Start the ROS stuff
        rospy.Subscriber(topic_in, NavSatFix, self.GPSCallback)
        rospy.spin()

        
    def GPSCallback(self, data):
        self.i = self.i +1
        # GPS Calback: Called when a new gps point arrives
        self.coords = (data.latitude, data.longitude, data.altitude)
        self.history.append(self.coords)
        self.average = (mean(map(lambda x: x[0], self.history)), mean(map(lambda y: y[1], self.history)), mean(map(lambda z: z[2], self.history))) 
        sys.stdout.write("Current GPS point: (%f, %f, %f) \t\t Average point: (%f, %f, %f)\r" \
                    % (data.latitude, data.longitude, data.altitude,  \
                self.average[0], self.average[1], self.average[2])) 
        sys.stdout.flush()

        if self.i >= self.window_size:
            rospy.loginfo("Surveying... AveragePoint: %s ", ('%f, %f, %f' % self.average) )
            with open(self.file,'a') as f:
            #write_data = f.write(str('%f, %f, %f\n' % self.coords))
                write_data = f.write(str('  Lat: %.10f\n  Lon: %.10f\n  Alt: %.10f\n' % self.average))
            rospy.signal_shutdown("Enough GPS points recieved.")

def mean(l):
    return float(sum(l))/len(l) if len(l) > 0 else float('nan')

if __name__ == "__main__":
    try:
        Surveyor()
    except rospy.ROSInterruptException: pass

