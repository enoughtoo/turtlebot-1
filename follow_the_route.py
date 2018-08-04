#!/usr/bin/env python

import rospy
import yaml

import sys

from take_photo import TakePhoto
from go_to_specific_point_on_map import GoToPose

from std_msgs.msg import Bool
from std_msgs.msg import UInt32
from std_msgs.msg import String

if __name__ == '__main__':

    # Read information from yaml file
    with open("/home/robot/git/turtlebot_actions/turtlebot/route.yaml", 'r') as stream:
        dataMap = yaml.load(stream)

    try:
        rospy.init_node('follow_route', anonymous=False)
        
        start_session_pub = rospy.Publisher('photo/start_session', Bool, queue_size=10)
        session_state_msg = Bool()
        
        session_num_pub = rospy.Publisher('photo/session_num', UInt32, queue_size=10)
        session_num_msg = UInt32()
        
        rack_id_pub = rospy.Publisher('photo/rack_id', UInt32, queue_size=10)
        rack_id_msg = UInt32()

        navigator = GoToPose()
        camera = TakePhoto()

        for obj in dataMap:

            if rospy.is_shutdown():
                break

            name = obj['filename']
            
            session_num_msg = obj['session']
            rack_id_msg = obj['rackid']
            session_state_msg = True

            rospy.loginfo("Go to %s pose of rack %s", name[:-4], rack_id_msg)

            start_session_pub.publish(session_state_msg)
            session_num_pub.publish(session_num_msg)
            rack_id_pub.publish(rack_id_msg)

            success = navigator.goto(obj['position'], obj['quaternion'])
            
            if not success:
                rospy.loginfo("Failed to reach %s pose", name[:-4])
                continue

            rospy.loginfo("Reached %s pose", name[:-4])

            # Take a photo from kinnect
            if camera.take_picture(name):
                rospy.loginfo("Saved image " + name)
            else:
                rospy.loginfo("No images received")
                
            session_state_msg = False
            start_session_pub.publish(session_state_msg)
            
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
