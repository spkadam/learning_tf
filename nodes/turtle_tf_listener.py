#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    '''TransformListener to help make the task of receiving transforms easier. 
    Here, we create a tf.TransformListener object. Once the listener is created,
    it starts receiving tf transformations over the wire, and buffers them for up to 10 seconds. '''
    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    '''Here, the real work is done, we query the listener for a specific transformation by lookupTransform. 
    Let's take a look at the arguments:
    We want the transform from the '/turtle1' frame ...
    ... to the '/turtle2' frame.
    The time at which we want to transform. Providing rospy.Time(0) will just get us the latest available transform. 
    This function returns two lists. The first is the (x, y, z) linear transformation of the child frame relative to the parent,
     and the second is the (x, y, z, w) quaternion required to rotate from the parent orientation to the child orientation.
    All this is wrapped in a try-except block to catch possible exceptions. '''
    rate = rospy.Rate(10.0)

    '''The waitForTransform() takes four arguments:

    1.Wait for the transform from this frame...
    2.... to this frame,
    3.at this time, and
    4.timeout: don't wait for longer than this maximum duration '''
    
    listener.waitForTransform("/turtle2", "/turtle1", rospy.Time(), rospy.Duration(4.0))

    while not rospy.is_shutdown():
        # try:
        #      (trans,rot) = listener.lookupTransform('/turtle2', '/carrot1', rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue

        #  try:
        #       now = rospy.Time.now()
        #       listener.waitForTransform("/turtle2", "/carrot1", now, rospy.Duration(4.0))
        #      (trans,rot) = listener.lookupTransform("/turtle2", "/carrot1", now)
        #  except (tf.LookupException, tf.ConnectivityException):

        #  try:
        #      now = rospy.Time.now() - rospy.Duration(5.0)
        #      listener.waitForTransform("/turtle2", "/turtle1", now, rospy.Duration(1.0))
        #      (trans, rot) = listener.lookupTransform("/turtle2", "/turtle1", now)
        #  except (tf.Exception, tf.LookupException, tf.ConnectivityException):
        '''The advanced API for lookupTransform() takes six arguments:

        1.Give the transform from this frame,
        2. at this time ...
        3.... to this frame,
        4.at this time.
        5.Specify the frame that does not change over time, in this case the "/world" frame, and
        6.the variable to store the result in. 

        Notice that waitForTransform() also has a basic and and advanced API, just like lookupTransform().
        time_travel.png'''     
        try:
            now = rospy.Time.now()
            past = now - rospy.Duration(5.0)
            listener.waitForTransformFull("/turtle2", now,
                                      "/turtle1", past,
                                      "/world", rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransformFull("/turtle2", now,
                                      "/turtle1", past,
                                      "/world")

             
        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()