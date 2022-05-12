#!/usr/bin/env python
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from time import time
from distributed_functions import checkingAvailableRobot
#############################################################
odom_list     = []
distance_list = []

#############################################################
def odomCallback(data, args):
    global odom_list
    # args are listed as follows: 
    # 0 - number of robots, 1 - index of robot, 2 - listener, 3 - global_frame
    # check whether it is the first time running or not
    if len(odom_list) == 0:
        for i in range(0, args[0]):
            odom_list.append({'current_pos': [], 'distance': 0.0, 'first_run':True})
    # check for the transform of the robot
    cond    = 0
    attempt = 0
    while cond == 0 and attempt < 10:
        try:
            (trans, rot) = args[2].lookupTransform(args[3], data.header.frame_id, rospy.Time(0.5))
            cond = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            cond == 0
        attempt += 1
    # now performing data checking 
    if attempt > 10 and cond == 0:
        # if havent success in retrieving the data
        print("~~~ not getting any message from the TF. skipping data and not updating the list")
    else:
        # if success in retreiving the data
        print("performed the data transformed for the data")
        if odom_list[args[1]]['first_run'] == True:
            odom_list[args[1]]['current_pos']  = [trans[0], trans[1]]

        x = trans[0]
        y = trans[1]

        d_increment = np.sqrt(((x - odom_list[args[1]]['current_pos'][0])*(x - odom_list[args[1]]['current_pos'][0])) + \
             ((y - odom_list[args[1]]['current_pos'][1])*(y - odom_list[args[1]]['current_pos'][1])))
        odom_list[args[1]]['distance'] = odom_list[args[1]]['distance'] + d_increment
        odom_list[args[1]]['first_run'] = False
        odom_list[args[1]]['current_pos']  = [x, y]

def odomCallback2(data, args):
    global odom_list
    # args are listed as follows: 
    # 0 - number of robots, 1 - index of robot, 2 - listener, 3 - global_frame
    # check whether it is the first time running or not
    if len(odom_list) == 0:
        for i in range(0, args[0]):
            odom_list.append({'current_pos': [], 'distance': 0.0, 'first_run':True})

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    if odom_list[args[1]]['first_run'] == True:
        odom_list[args[1]]['current_pos']  = [x, y]


    d_increment = np.sqrt(((x - odom_list[args[1]]['current_pos'][0])*(x - odom_list[args[1]]['current_pos'][0])) + \
            ((y - odom_list[args[1]]['current_pos'][1])*(y - odom_list[args[1]]['current_pos'][1])))
    odom_list[args[1]]['distance'] = odom_list[args[1]]['distance'] + d_increment
    odom_list[args[1]]['first_run'] = False
    odom_list[args[1]]['current_pos']  = [x, y]


#############################################################
def node():
    global odom_list
    rospy.init_node('odom_tracking', anonymous=False)

    # define the rosparam
    global_frame      = rospy.get_param('~global_frame', 'map')   # the global frame topic going to subscribe to
    robot_frame       = rospy.get_param('~robot_frame' , 'base_footprint')        # the robot frame robot is tracking for
    name_prefix       = rospy.get_param('~name_prefix' , 'tb3')        # the robot frame robot is tracking for
    odom_topic        = rospy.get_param('~odom_topic' , 'Odom')       # odom topic for the node for subscription
    display_interval  = rospy.get_param('~display_interval' , 4.0)     # time interval for displayign the message to the user
    frequency         = rospy.get_param('~frequency', 10.0)   # frequency for deciding how frequency the ros node running in the background

    rate = rospy.Rate(frequency)

    # get the basic robot information for the subscribing of the topic
    robots_list             = checkingAvailableRobot(name_prefix, [odom_topic]) # now must make sure the robot has all the name

    # now perform the initialization / post processing
    listener_list = []
    for i in range(0, len(robots_list)):
        listener_list.append([])

    # now subscribe to the robot
    subs = []
    print(robots_list)
    for i in range(0, len(robots_list)):
    # 0 - number of robots, 1 - index of robot, 2 - listener, 3 - global_frame
        subs.append(rospy.Subscriber(robots_list[i] + "/odom", Odometry, \
            callback=odomCallback2, callback_args=[len(robots_list), i, listener_list[i], global_frame]))

    # while loop for displaying the odom message
    start_time        = rospy.get_rostime().secs
    nextTimeReport    = start_time + display_interval
    while not rospy.is_shutdown():
        # now start performing the odom calculating for the data given the to the list of the robot.
        if nextTimeReport <= rospy.get_rostime().secs:
            outputStr = ''
            print(odom_list)
            if len(odom_list) > 0:
                for q in range(0, len(robots_list)):
                    formatted_float = "{:.3f}".format(odom_list[q]['distance'])
                    outputStr += robots_list[q] + '-' + formatted_float + 'm'
                    if q != (len(robots_list) - 1):
                        outputStr += ';  '
                print('--------------++++++++++++++++++++++++++--------------')
                print(">> Distance list for robot: " + outputStr)
                print('--------------++++++++++++++++++++++++++--------------')
            nextTimeReport = rospy.get_rostime().secs + display_interval

#############################################################
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 #############################################################