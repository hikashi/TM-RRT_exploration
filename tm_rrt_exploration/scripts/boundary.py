#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from copy import copy
from geometry_msgs.msg import Point, PolygonStamped, Polygon, PointStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Bool
from time import time

#############################################################
boundary_Pts       = []
recordedPoints     = []
startExploration   = True 
resetExploration   = False
odomPts            = [0.0, 0.0]

#############################################################
def boundaryMapInfoCallback(data):
    global boundary_Pts
    # data = map
    map_resolution = data.info.resolution
    map_width      = data.info.height
    map_height     = data.info.width 
    pos_origin_x   = data.info.origin.position.x
    pos_origin_y   = data.info.origin.position.y
    # check the boundary pts
    if len(boundary_Pts) < 4:
        boundary_Pts.append(np.array([((map_width*map_resolution)*-1.0)-pos_origin_x+0.01,((map_height*map_resolution)*1.0)+pos_origin_y-0.01]))
        boundary_Pts.append(np.array([((map_width*map_resolution)*-1.0)-pos_origin_x+0.01,((map_height*map_resolution)*-1.0)-pos_origin_y+0.01]))
        boundary_Pts.append(np.array([((map_width*map_resolution)*1.0)+pos_origin_x-0.01,((map_height*map_resolution)*-1.0)-pos_origin_y+0.01]))
        boundary_Pts.append(np.array([((map_width*map_resolution)*1.0)+pos_origin_x-0.01,((map_height*map_resolution)*1.0)+pos_origin_y-0.01]))

#############################################################
def recordedPointsCallback(data):
    global recordedPoints, dataLength, explorationPoints
    if len(recordedPoints) < 4:
        tempPoints = Point()
        tempPoints.x = data.point.x
        tempPoints.y = data.point.y
        tempPoints.z = data.point.z
        recordedPoints.append(tempPoints)
        dataLength += 1

#############################################################
def listenStartExplorationCallback(data):
    global resetExploration, startExploration
    if data.data == True and startExploration == False:
        startExploration  =  True
    elif data.data == False and startExploration == True:
        startExploration  =  False
    if data.data == True and startExploration == True:
        startExploration  =  True      
            
#############################################################
def listenResetExplorationCallback(data):
    global resetExploration, startExploration
    if data.data == True:
        startExploration  =  False
        resetExploration  =  True
    
            
#############################################################
def listenOdomCallback(data, args):
    global odomPts # args are listed as follows: 
    # 0 - listener, 1 - global_frame
    cond = 0
    while cond == 0:
        try:
            (trans, rot) = args[0].lookupTransform(args[1], data.header.frame_id, rospy.Time(0))
            cond = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.sleep(0.11)
            cond == 0
            rospy.sleep(0.5) # sleep to combat the spam request to ros network
    # now making odomPts 
    odomPts = trans
    # if no need odom
    # odomPts = [data.pose.pose.position.x, data.pose.pose.position.y]
#############################################################
def node():
    # decide the global parameter for the data and workign on the necessary things for the data
    global recordedPoints, dataLength, startExploration, resetExploration, boundary_Pts, odomPts
    rospy.init_node('exploration_boundary', anonymous=False)

    # define the rosparam
    mapFrame      = rospy.get_param('~map_frame', 'map')   # the name frame boundary going to be published on
    n_point       = rospy.get_param('~n_point', 4) # the number of the boundary point going to be created
    topicInput    = rospy.get_param('~topicInput', '/clicked_point') # the number of the bXXoundary point going to be created
    startTopic    = rospy.get_param('~startTopic', '/start_signal') # the number of the boundary point going to be created
    resetTopic    = rospy.get_param('~resetTopic', '/reset_signal') # the number of the boundary point going to be created
    controlOutput = rospy.get_param('~controlOutput', '/explore_start') # the number of the boundary point going to be created
    restartOutput = rospy.get_param('~restartOutput', '/explore_reset') # the number of the boundary point going to be created
    topicOutput   = rospy.get_param('~topicOutput', '/exploration_boundary') # specify the topic which the node is going to published into the list
    frequency     = rospy.get_param('~frequency', 2.0)     # number of the rate listening to the ros param
    timeInterval  = rospy.get_param('~timeInterval', 5.0)   # time interval for broadcasting the information on odometry and time elapsed
    autoInputPoint= rospy.get_param('~AutoInputPoint', "")   # the auto input point instead of clicked point. (points separated by semicolon; and axis are separated by comma,)
    diagDistance  = rospy.get_param('~diagonal_distance', 7.50)   # the auto input point instead of clicked point. (points separated by semicolon; and axis are separated by comma,)
    mapTopic      = rospy.get_param('~mapTopic', '/map')   # the  name of the topic map subscribe to.
    initialPoint  = rospy.get_param('~initialStartPts', '1.9,3.19')   # the initial start pts if manually assign start pts
    odom_topic    = rospy.get_param('~odom_topic_startpts', '/tb3_1/odom')   # odom topic for defining pts
    robot_frame   = rospy.get_param('~robot_frame', '/tb3_1/base_footprint')   # footprint for converting tf
    # define the odom capture time
    #############################################################
    # preprocess some of the data
    rate       = rospy.Rate(frequency)
    dataLength = n_point
    rospy.Subscriber(topicInput, PointStamped, recordedPointsCallback)
    rospy.Subscriber(startTopic, Bool, listenStartExplorationCallback)
    rospy.Subscriber(resetTopic, Bool, listenResetExplorationCallback)
    rospy.Subscriber(mapTopic, OccupancyGrid, boundaryMapInfoCallback)
    listener = tf.TransformListener()
    # listener.waitForTransform(mapFrame, robot_frame, rospy.Time(0.0), rospy.Duration(10.0))
    rospy.Subscriber(odom_topic, Odometry, listenOdomCallback, callback_args=[listener, mapFrame])
    #############################################################
    output_start_control  = rospy.Publisher(controlOutput, Bool, queue_size=1)
    restart_control       = rospy.Publisher(restartOutput, Bool, queue_size=1)
    clicked_point_pub     = rospy.Publisher(topicInput, PointStamped, queue_size = 100)


    valid_entry = False
    control_msg = Bool()
    control_msg.data = startExploration
    output_start_control.publish(control_msg)   

    reset_msg = Bool()
    reset_msg.data = resetExploration
    restart_control.publish(reset_msg)   

    #############################################################
    # start the main function
    while not rospy.is_shutdown():
        # setting up the publishing option
        # check for the start exploration.
        if not resetExploration:
            if startExploration:
                boundary_record_point = []
                if not valid_entry:
                    coordinate_list = autoInputPoint.split(";")
                    # start working on the autoInputPoint
                    # if len(coordinate_list) == 0:
                    while len(boundary_Pts) < 4:
                        rospy.sleep(1.0)
                        print(boundary_Pts)
                        print('waiting for the boundary points to be calculated')
                        pass
                    # now publishing the clicked point.
                    rospy.sleep(2.0)
                    for point in boundary_Pts:
                        pub_point1 = PointStamped()
                        pub_point1.header.frame_id = mapFrame
                        pub_point1.point.x = float(point[0])
                        pub_point1.point.y = float(point[1])
                        pub_point1.point.z = 0.0
                        clicked_point_pub.publish(pub_point1)
                        rospy.sleep(1.5)

                    # now waiting for the user input.
                    # while len(recordedPoints) < (n_point):
                    #     print(recordedPoints)
                    #     print(boundary_Pts)
                    #     print('Please enter the exploration start point....') 
                    #     rospy.sleep(1.0)
                    #     pass

                    tempStr = initialPoint.split(",")
                    pub_point2 = PointStamped()
                    pub_point2.header.frame_id = mapFrame
                    pub_point2.point.x = float(tempStr[0])
                    pub_point2.point.y = float(tempStr[1])
                    # pub_point2.point.x = float(odomPts[0])
                    # pub_point2.point.y = float(odomPts[1])
                    pub_point2.point.z = 0.0
                    clicked_point_pub.publish(pub_point2)
                    rospy.sleep(0.7)


                    # convert to the points for boundary]
                    valid_entry = True
                    rospy.loginfo('boundary polygon: received all ' + str(n_point) + ' points from the user')
                    
                
                control_msg = Bool()
                control_msg.data = startExploration
                output_start_control.publish(control_msg)                     
                reset_msg = Bool()
                reset_msg.data = resetExploration
                restart_control.publish(reset_msg)
                
                rate.sleep()

                if valid_entry:
                    # check only 4 points input by the user.
                    boundaryPolygon  = rospy.Publisher(topicOutput, PolygonStamped, queue_size=10)
                    start_time = rospy.get_rostime().secs
                    nextTimeReport = start_time + timeInterval
                    rospy.loginfo('--- >>> the exploration starts at time: %.2f ' %(start_time))
                    while startExploration:
                        # now start workign on publishing theboudnary geometry point
                        header = Header()
                        boundaryList = PolygonStamped()
                        boundaryPoints = Polygon()
                        
                        boundaryPoints.points = copy(recordedPoints)
                        boundaryList.polygon = boundaryPoints

                        header.frame_id = mapFrame
                        header.stamp = rospy.Time.now()
                        boundaryList.header = header

                        boundaryPolygon.publish(boundaryList)
                        
                        if nextTimeReport <= rospy.get_rostime().secs:
                            rospy.loginfo('--- >>> current time: %.2f    time elapsed: %.2f' %(rospy.get_rostime().secs, rospy.get_rostime().secs-start_time))
                            nextTimeReport = rospy.get_rostime().secs + timeInterval
                        
                        control_msg = Bool()
                        control_msg.data = startExploration
                        output_start_control.publish(control_msg)          
                        reset_msg = Bool()
                        reset_msg.data = resetExploration
                        restart_control.publish(reset_msg)   
                        rate.sleep()
            else:   
                control_msg = Bool()
                control_msg.data = startExploration
                output_start_control.publish(control_msg)                     
                reset_msg = Bool()
                reset_msg.data = False
                restart_control.publish(reset_msg)   
                # print('pending for exploration input')
                rospy.sleep(2.0)
        else:           
            boundary_Pts = []       
            boundary_record_point = []  
            control_msg = Bool()
            control_msg.data = startExploration
            output_start_control.publish(control_msg)     
            reset_msg = Bool()
            reset_msg.data = resetExploration
            restart_control.publish(reset_msg)    
            # --------------------------------------------------
            rospy.sleep(2.0)
            reset_msg.data = False
            restart_control.publish(reset_msg) 
            resetExploration = False
            # --------------------------------------------------
            if len(recordedPoints) > 0:
                recordedPoints     = [] 
                valid_entry        = False
            # --------------------------------------------------
            
        rate.sleep()

#############################################################
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 

 
                # elif len(coordinate_list) == 1:
                #     point1             = coordinate_list[0].split(",")
                #     clicked_point_pub  = rospy.Publisher(topicInput, PointStamped, queue_size = 100)

                #     pub_point1 = PointStamped()
                #     pub_point1.header.frame_id = mapFrame
                #     pub_point1.point.x = float(point1[0]) + diagDistance
                #     pub_point1.point.y = float(point1[1]) + diagDistance
                #     pub_point1.point.z = float(point1[2])
                #     clicked_point_pub.publish(pub_point1)
                #     rospy.sleep(1.3)

                #     pub_point2 = PointStamped()
                #     pub_point2.header.frame_id = mapFrame
                #     pub_point2.point.x = float(point1[0]) - diagDistance
                #     pub_point2.point.y = float(point1[1]) + diagDistance
                #     pub_point2.point.z = float(point1[2])
                #     clicked_point_pub.publish(pub_point2)
                #     rospy.sleep(1.3)                

                #     pub_point3 = PointStamped()
                #     pub_point3.header.frame_id = mapFrame
                #     pub_point3.point.x = float(point1[0]) - diagDistance
                #     pub_point3.point.y = float(point1[1]) - diagDistance
                #     pub_point3.point.z = float(point1[2])
                #     clicked_point_pub.publish(pub_point3)
                #     rospy.sleep(1.3)
                    
                #     pub_point4 = PointStamped()
                #     pub_point4.header.frame_id = mapFrame
                #     pub_point4.point.x = float(point1[0]) + diagDistance
                #     pub_point4.point.y = float(point1[1]) - diagDistance
                #     pub_point4.point.z = float(point1[2])
                #     clicked_point_pub.publish(pub_point4)
                #     rospy.sleep(1.3)

                #     pub_point5 = PointStamped()
                #     pub_point5.header.frame_id = mapFrame
                #     pub_point5.point.x = float(point1[0]) 
                #     pub_point5.point.y = float(point1[1])
                #     pub_point5.point.z = float(point1[2])
                #     clicked_point_pub.publish(pub_point5)
                #     rospy.sleep(1.3)


                #     valid_entry = True
                # else:
                    
                #     for coordinate in coordinate_list:
                #       point = coordinate.split(",")
                #       tempPoints = Point()
                #       tempPoints.x = float(point[0])
                #       tempPoints.y = float(point[1])
                #       tempPoints.z = float(point[2])
                #       recordedPoints.append(tempPoints)
                #     # publish the point to the clicked point
                #     clicked_point_pub  = rospy.Publisher(topicInput, PointStamped, queue_size=100)
                #     clicked_point_pub.publish(recordedPoints)


                    # now check whether the recorded points are more
                # if len(recordedPoints) < 5 or len(recordedPoints) > 5: # can be only 5 points, 4 for boundary and 1 for initial starting point
                #         rospy.loginfo("Insufficient point to start the exploration")
                #         startExploration = False
                # else:
                #     valid_entry = True
