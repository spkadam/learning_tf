#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
import turtlesim.msg

def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    '''The handler function for the turtle pose message broadcasts this turtle's 
    translation and rotation, and publishes it as a transform from frame "world" to frame "turtleX". '''
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")

if __name__ == '__main__':

    rospy.init_node('turtle_tf_broadcaster')
    '''This node takes a single parameter "turtle", which specifies a turtle name, e.g "turtle 1" or "turtle 2"'''
    turtlename = rospy.get_param('~turtle') 
    '''The node subscribes to topic "turtleX/pose" and runs function handle_turtle_pose on every incoming message.'''
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
