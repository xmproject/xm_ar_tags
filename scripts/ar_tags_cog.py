#!/usr/bin/env python
'''
/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Created for the Pi Robot Project: http://www.pirobot.org
 *  Copyright (c) 2013 Patrick Goebel.  All rights reserved.
 *  Created for the XM Robot Project: http://www.github/xmproject
 *  Copyright (c) 2015 The XM Robot Team. All rights reserved
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of XM Robot Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
'''

# Description: Find the COG of AR tags that are detected in the field of view and 
# publish the result as a PoseStamped message on the /target_pose topic.


import rospy
from geometry_msgs.msg import Point, PoseStamped
from ar_track_alvar.msg import AlvarMarkers

class TagsCOG():
    def __init__(self):
        rospy.init_node("ar_tags_cog")
        
        # Read in an optional list of valid tag ids
        self.tag_ids = rospy.get_param('~tag_ids', None)
        
        # Publish the COG on the /target_pose topic as a PoseStamped message
        self.tag_pub = rospy.Publisher("target_pose", PoseStamped)

        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.get_tags)
        
        rospy.loginfo("Publishing combined tag COG on topic /target_pose...")
                
    def get_tags(self, msg):
        # Initialize the COG as a PoseStamped message
        tag_cog = PoseStamped()
        
        # Get the number of markers
        n = len(msg.markers)
        
        # If no markers detected, just return
        if n == 0:
            return

        # Iterate through the tags and sum the x, y and z coordinates            
        for tag in msg.markers:
            
            # Skip any tags that are not in our list
            if self.tag_ids is not None and not tag.id in self.tag_ids:
                continue
            
            # Sum up the x, y and z position coordinates of all tags
            tag_cog.pose.position.x += tag.pose.pose.position.x
            tag_cog.pose.position.y += tag.pose.pose.position.y
            tag_cog.pose.position.z += tag.pose.pose.position.z
            
             # Compute the COG
            tag_cog.pose.position.x /= n
            tag_cog.pose.position.y /= n
            tag_cog.pose.position.z /= n
            
            # Give the tag a unit orientation
            tag_cog.pose.orientation.w = 1

            # Add a time stamp and frame_id
            tag_cog.header.stamp = rospy.Time.now()
            tag_cog.header.frame_id = msg.markers[0].header.frame_id

            # Publish the COG
            self.tag_pub.publish(tag_cog)      
  
if __name__ == '__main__':
    try:
        TagsCOG()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AR Tag Tracker node terminated.")
