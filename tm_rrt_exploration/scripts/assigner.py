#!/usr/bin/env python3

#--------Include modules---------------
import rospy
import random
import time
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, Bool
from copy import copy
from numpy import array
from numpy.linalg import norm
# import math
from tmrrt_exploration.msg import PointArray, invalidArray
from functions import robot,informationGain,discount2,calcDynamicTimeThreshold, calculateLocationDistance, relativePositionMetric, getMaxRevenueForRobot, getHigestIndex


# Subscribers' callbacks------------------------------
mapData         = OccupancyGrid()
frontiers       = []
invalidFrontier = []
globalmaps      = []
startStopSignal = False
resetSignal     = False

def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data

def startSignalCallBack(data):
    global startStopSignal
    startStopSignal=data.data


def resetSignalCallBack(data):
    global resetSignal
    resetSignal=data.data
# Node----------------------------------------------

def node():
	global frontiers,mapData,globalmaps,invalidFrontier, startStopSignal, resetSignal
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic                = rospy.get_param('~map_topic','/map')
	info_radius              = rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier          = rospy.get_param('~info_multiplier',3.0)		
	hysteresis_radius        = rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain          = rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic          = rospy.get_param('~frontiers_topic','/filtered_points')	
	inv_points_topic         = rospy.get_param('~invalid_frontier','/invalid_points')	
	inv_frontier_topic       = rospy.get_param('~invalid_centroids','/invalid_centroids')	
	time_per_meter           = rospy.get_param('~time_per_meter',12)
	message_time_interval    = rospy.get_param('~message_time_interval',2.0)
	robot_namelist           = rospy.get_param('~robot_namelist', "robot1")
	delay_after_assignment   = rospy.get_param('~delay_after_assignment',1.0)
	invalid_distance         = rospy.get_param('~invalid_distance',0.5)
	rp_metric_distance       = rospy.get_param('~rp_metric_distance',10.0)
	non_interrupt_time       = rospy.get_param('~non_interrupt_time',1.2)
	start_delay              = rospy.get_param('~start_delay',1.0)
	rateHz                   = rospy.get_param('~rate',100)
	startSignalTopic 		 = rospy.get_param('~startSignalTopic', '/explore_start') # start stop signal for the robot
	resetSignalTopic 		 = rospy.get_param('~resetSignalTopic', '/explore_reset') # reset signal for controlling the start and end of the robot
	debugFlag1               = rospy.get_param('~debugFlag1',False)
	debugFlag2               = rospy.get_param('~debugFlag2',False)
	# services for the robot
	global_frame             = rospy.get_param('~global_frame', 'map')
	plan_service             = rospy.get_param('~plan_service', '/move_base_node/NavfnROS/make_plan')
	base_link                = rospy.get_param('~base_link', 'base_link')
	move_base_service        = rospy.get_param('~move_base_service', '/move_base')

	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(frontiers_topic, PointArray, callBack)
	rospy.Subscriber(startSignalTopic, Bool, startSignalCallBack)
	rospy.Subscriber(resetSignalTopic, Bool, resetSignalCallBack)

	#start a big while loop?
#---------------------------------------------------------------------------------------------------------------
	robot_namelist = robot_namelist.split(',')
	# print(robot_namelist)
#---------------------------------------------------------------------------------------------------------------
	start_time = rospy.get_rostime().secs
	robot_assigned_goal = []

# wait if no frontier is received yet 
	while len(frontiers)<1:
		pass
	centroids=copy(frontiers)	
#wait if map is not received yet
	while (len(mapData.data)<1):
		pass

	robots=[]
	if len(robot_namelist) > 0:
		for i in range(0,len(robot_namelist)):
			rospy.loginfo('working on initializing robot  :' + robot_namelist[i])
			robots.append(robot(name=robot_namelist[i],move_base_service=move_base_service,in_service=plan_service,global_frame=global_frame,base_link=base_link))

	for i in range(0,len(robot_namelist)):
		rospy.loginfo('setting initial position for robot: %s' %(robot_namelist[i]))	
		position0, rot0 = robots[i].getPosition(quad=True)
		robots[i].sendGoal(point=position0, quadData=rot0)
		robot_assigned_goal.append({'robot_id':i, 'goal':position0, 'startLoc':position0,
		 'time_start':start_time, 'valid':True, 'time_thres':-1, 'lastgoal':position0})


	invCenPub = rospy.Publisher(inv_frontier_topic, Marker, queue_size=10)
	invPub    = rospy.Publisher(inv_points_topic, invalidArray, queue_size=10)

	# -------------------- define the basic information for the invalid centroid to publish
	points = Marker()
	points.header.frame_id = mapData.header.frame_id
	points.header.stamp = rospy.Time.now()
	points.ns   = "invalidCentroid"
	points.id   = 0
	points.type = Marker.POINTS
	# Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	points.action = Marker.ADD
	points.pose.orientation.w = 1.0
	points.scale.x  = 0.4
	points.scale.y  = 0.4
	points.color.r  = 255.0/255.0
	points.color.g  = 0.0/255.0
	points.color.b  = 255.0/255.0
	points.color.a  = 0.5
	points.lifetime = rospy.Duration()

# ------------------------------------------------------------------------
# delay for initializing the robot
	rospy.sleep(start_delay)

#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	robot_goal_record = []
	temp_inv_array = []
	next_assign_time   = rospy.get_rostime().secs
	next_time_interval = rospy.get_rostime().secs
	time_interval_due = False
	robot_goal_cancel = []
	while not rospy.is_shutdown():
		if resetSignal:	
			print(">> ASSIGNER NODE ------------ RESET SIGNAL RECEIVED ------------")
			# reinitialize everything.
			robot_assigned_goal = []
			robots              = []			
			robot_goal_record   = []
			temp_inv_array      = []
			next_assign_time    = rospy.get_rostime().secs
			next_time_interval  = rospy.get_rostime().secs
			time_interval_due   = False
			robot_goal_cancel   = []
			if len(robot_namelist) > 0:
				for i in range(0,len(robot_namelist)):
					robots.append(robot(name=robot_namelist[i],move_base_service=move_base_service,in_service=plan_service,global_frame=global_frame,base_link=base_link))

			for i in range(0,len(robot_namelist)):
				position1, rot1 = robots[i].getPosition(quad=True)
				robots[i].sendGoal(point=position1, quadData=rot1)
				# robots[i].sendGoalTransformed(robots[i].getPosition())
				robot_assigned_goal.append({'robot_id':i, 'goal':position1, 'startLoc':position1,
				'time_start':start_time, 'valid':True, 'time_thres':-1, 'lastgoal':position1})

		if startStopSignal:
			centroids=copy(frontiers)	

	#-------------------------------------------------------------------------			
			if next_time_interval <= rospy.get_rostime().secs:
				time_interval_due = True
				next_time_interval = rospy.get_rostime().secs + message_time_interval
	#-------------------------------------------------------------------------			
			invalid_goal_array = invalidArray()
			invalid_goal_array.points = []
			
			for iz in range(0,len(invalidFrontier)):
				tempPoint = Point()
				tempPoint.z = 0.0
				tempPoint.x = invalidFrontier[iz][0]
				tempPoint.y = invalidFrontier[iz][1]
				invalid_goal_array.points.append(copy(tempPoint))


				
	#-------------------------------------------------------------------------
	# check for the status of the movebase 
			for ii in range(0, len(robot_namelist)):
					movebase_status = robots[ii].get_movebase_status()
					# check the move_base status #4 is having difficulties.
					# need liase with brandon to make sure the nubmer of retry is limited and more prone to trigger this condition based on the move base condition.
					if movebase_status >= 4 and robot_assigned_goal[ii]['time_thres'] != -1:
						currTime = rospy.get_rostime().secs

						# marks point as invalid
						distance = calculateLocationDistance(robots[ii].getPosition(), robot_assigned_goal[ii]['goal'])
						repeatInvalid = False
							# if the distance is very far then do not publish as invalid point, mostly cant reach goal within the given time limit 
						robot_assigned_goal[ii]['lastgoal']   = robot_assigned_goal[ii]['goal']
						robot_assigned_goal[ii]['goal']       = robots[ii].getPosition()
						robot_assigned_goal[ii]['time_start'] = rospy.get_rostime().secs
						robot_assigned_goal[ii]['startLoc']   = robots[ii].getPosition()
						robot_assigned_goal[ii]['time_thres'] = -1
						# cancel goal
						rospy.loginfo("\n!!!> Robot " + robot_namelist[ii] + " give up goal: " + str(robot_assigned_goal[ii]['goal'])
						+ " at time: "  + str(currTime) + " sec  -- distance: " + str(distance) + " -- robot path planning failed \n")
						robots[ii].cancelGoal()
						robot_goal_cancel.append(ii)
							
	#-------------------------------------------------------------------------			
	#get number of available/busy robots
			na=[] #available robots
			nb=[] #busy robots
			for i in range(0, len(robot_namelist)):
					if ii in robot_goal_cancel:
						na.append(i)
					else:
						if (robots[i].getState()==1):
							nb.append(i)
						else:
							na.append(i)
			# remove the content of the goal cancel list and start from the scratch again
			robot_goal_cancel = []
			

	#-------------------------------------------------------------------------			
	#Get information gain for each frontier point
			infoGain=[]
			for ip in range(0,len(centroids)):
				infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*info_multiplier)
	#------------------------------------------------------------------------- 
	#get dicount and update informationGain
			for i in nb+na:
				infoGain=discount2(mapData,robot_assigned_goal[i]['lastgoal'],centroids,infoGain,info_radius)
	#-------------------------------------------------------------------------            
			revenue_record=[]
			centroid_record=[]
			robots_position = []
			robots_goals    = []
			# initilization of the array
			for qq in range(0, len(robot_namelist)):
				if debugFlag2:
					print("robot %s initialize the robot revenue and centroid" %(robot_namelist[qq]))
				revenue_record.append([])
				centroid_record.append([])
			# get robot position and goals
			for qq in range(0, len(robot_namelist)):
				if debugFlag2:
					startTime = time.time()
				robots_position.append(robots[qq].getPosition())
				robots_goals.append(robot_assigned_goal[qq]['goal'])
				if debugFlag2:
					print("Robot %s goal and position queried used time: %fs" %(robot_namelist[qq],time.time()-startTime))

			robotListTemp2 = na + nb
			# random.shuffle(robotListTemp2)
			for ir in robotListTemp2:
				if debugFlag1:
					startTime = time.time()
				for ip in range(0,len(centroids)):
					cost=norm(robots_position[ir]-centroids[ip])	
					# if cost <= (rp_metric_distance*1.5):
					information_gain=infoGain[ip]
					if ir in nb:
						if norm(centroids[ip]-robots_position[ir])<=hysteresis_radius:
							information_gain*=hysteresis_gain

						if norm(robots_goals[ir]-centroids[ip])<=hysteresis_radius:
							information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*hysteresis_gain
					else:
						if norm(centroids[ip]-robots_position[ir])<=hysteresis_radius:
							information_gain*=hysteresis_gain

					
					rp_metric = relativePositionMetric(centroids[ip], ir, robots_goals, rp_metric_distance)
					
					if information_gain >= 0:
						information_gain = information_gain * rp_metric
					else:
						information_gain = information_gain / rp_metric

					revenue=(information_gain)- (cost)
					revenue_record[ir].append(revenue)
					centroid_record[ir].append(centroids[ip])

				if debugFlag2:
					print("Robot %s revenue and centroid calculated used %fs" %(robot_namelist[ir],time.time()-startTime))
	#-------------------------------------------------------------------------	
			if time_interval_due:
				if debugFlag1:
					rospy.loginfo("available robots: "+str([robot_namelist[xyz] for xyz in na]))	

				if len(na) >= 1:
					if debugFlag1:
						rospy.loginfo("revenue record: "+str(revenue_record))	
						rospy.loginfo("centroid record: "+str(centroid_record))	
						for ih in range(0, len(robot_namelist)):
							currTime = rospy.get_rostime().secs
							timeRemaining = (robot_assigned_goal[ih]['time_start'] + robot_assigned_goal[ih]['time_thres']) - currTime
							rospy.loginfo("robot " + robot_namelist[ih] + " - last goal:" + str(robot_assigned_goal[ih]['lastgoal']))
				else:
					rospy.loginfo("- no idling robots detected, status update -")
					for ih in range(0, len(robot_namelist)):
						currTime = rospy.get_rostime().secs
						timeRemaining = (robot_assigned_goal[ih]['time_start'] + robot_assigned_goal[ih]['time_thres']) - currTime
						rospy.loginfo("robot " + robot_namelist[ih] + " working on goal:" + str(robot_assigned_goal[ih]['goal']) 
						+ " -remaining time: " + str(float(timeRemaining)) + "s")

	#-------------------------------------------------------------------------	
			if (len(centroids)>0):
				goal_repeated   = False
				robotListTemp = na + nb
				random.shuffle(robotListTemp)
				if rospy.get_rostime().secs >= next_assign_time:
					for xxx in robotListTemp:
						'''
						the idea here allocate retry function for each of the next highest revenue frontier.
						for Available robot, there will be no condition imposed to allocate the revenue when compared to the Busy robot.
						For busy robot, the condition imposed will be based on three metrics - revenue, distance, time.
						'''
						notAssign = True
						attempt  = 0
						start = time.time()
						# condition setting up here:
							# not assigned a goal
							# attempt will not be larger than the list of centroids
							# the robot is in available state
						skipAssign = False
						if xxx in nb:
							goalDistance = calculateLocationDistance(robots[xxx].getPosition(), robot_assigned_goal[xxx]['goal'])
							timeElapsed = rospy.get_rostime().secs - robot_assigned_goal[xxx]['time_start']
							if goalDistance >= (hysteresis_radius) and goalDistance <= (rp_metric_distance*1.5):
								skipAssign = True
								if debugFlag1:
									print("\n ======== --- Robot %s skip the goal assignment!  Distance to goal: %fm" %(robot_namelist[xxx], goalDistance))
							# now check the time.
							if timeElapsed > non_interrupt_time and goalDistance <= hysteresis_radius:
							# cond_busyNear = False  	
								skipAssign = True
								if debugFlag1:
									print("\n ======== --- Robot %s skip the goal assignment!  non interrupt time: %.2fs" %(robot_namelist, (non_interrupt_time - timeElapsed)))
						if not skipAssign:
							while notAssign and attempt < len(centroids):
								winner_id = getHigestIndex(revenue_record[xxx], attempt)
								if winner_id != -1:
									# condition for the general robot	
									cond_goal       = True
									cond_history    = True
									cond_goalTaken  = True
									cond_busyNear   = True
									for xi in range(0, len(invalidFrontier)):
										if calculateLocationDistance(centroid_record[xxx][winner_id], invalidFrontier[xi]) < 0.1:
											cond_goal = False
											if debugFlag1:
												print(">> invalid goal detected. distance: %.3f" %(calculateLocationDistance(centroid_record[xxx][winner_id], invalidFrontier[xi])))
											break	
									# check if history appear in other robot history
									history = []
									for xj in range(0,len(robot_namelist)):
										history.append(robots[xj].getGoalHistory())
									for xj in range(0, len(robot_namelist)):
										if xxx != xj:
											for xh in range(0, len(history[xj])):
												distance_apart = calculateLocationDistance(centroid_record[xxx][winner_id], history[xj][xh])
												if distance_apart <= 0.3:
													cond_goalTaken = False
													if debugFlag1:
														print('Goal [%f,%f] for robot %s repeated at other robot %s history' %(centroid_record[xxx][winner_id][0],centroid_record[xxx][winner_id][1], robot_namelist[xxx],robot_namelist[xj]))
													break
										# if the goal is repeated at own history
										else:
											for xh in range(0, len(history[xj])):
												distance_apart = calculateLocationDistance(centroid_record[xxx][winner_id], history[xj][xh])
												if distance_apart <= 0.1:
													remainingTime = ((robot_assigned_goal[xj]['time_start'] + robot_assigned_goal[xj]['time_thres']) - rospy.get_rostime().secs) 
													if remainingTime < (0.0-delay_after_assignment):
														cond_history = False
														if debugFlag1:
															print('history repeat the same goal assignment in robot %s history' %(robot_namelist[xxx]))
														break
									if xxx in nb:
											distance_apart = calculateLocationDistance(centroid_record[xxx][winner_id], robot_assigned_goal[xxx]['goal'])
											if distance_apart >= (hysteresis_radius*2.0):
													cond_busyNear = False
																				
									# checking the condition for the goal assignment 
									if cond_history and cond_goal and cond_goalTaken and cond_busyNear:
										position3, rot3 = robots[xxx].getPosition(quad=True) 
										robots[xxx].sendGoal(point=centroid_record[xxx][winner_id], quadData=rot3)
										robots[xxx].setGoalHistory(centroid_record[xxx][winner_id])	
										robot_assigned_goal[xxx]['lastgoal']   = robot_assigned_goal[xxx]['goal'].copy()
										robot_assigned_goal[xxx]['goal']       = centroid_record[xxx][winner_id]
										robot_assigned_goal[xxx]['startLoc']   = position3
										robot_assigned_goal[xxx]['time_start'] = rospy.get_rostime().secs
										robot_assigned_goal[xxx]['time_thres'] = calcDynamicTimeThreshold(position3, centroid_record[xxx][winner_id], time_per_meter)
										rospy.loginfo("\n" + robot_namelist[xxx] + " has been assigned to  "+str(centroid_record[xxx][winner_id]) + 	
										" mission start at " + str(robot_assigned_goal[xxx]['time_start']) + " sec  - Limit: " + str(int(robot_assigned_goal[xxx]['time_thres'])) + "s") 
										notAssign        = False
									else:
										# if not assigned
										attempt = attempt + 1
										if debugFlag2:
											print("======== Robot: %s attempt no %d to assign goal ==========" %(robot_namelist[xxx], attempt))
								else:
									if debugFlag1:
										print("++++++== Robot: %s give up assigning goal after %d attempt ==++++++" %(robot_namelist[xxx], attempt))
								
							if debugFlag1:
								print('    Assignment of goal for robot %s used %.6f seconds \n' %(robot_namelist[xxx], (time.time()-start)))


							# check whether there is invalid goal or history after numerous attempt
							if (cond_history or cond_goalTaken) and not cond_goal:
								goal_repeated   = True
								faultyGoal      = centroid_record[xxx][winner_id]
								if goal_repeated:
									repeatInvalid = False
									for ie in range(0,len(invalidFrontier)):
										if calculateLocationDistance(invalidFrontier[ie], robot_assigned_goal[xxx]['goal']) < 0.01:
											repeatInvalid = True
									if repeatInvalid == False:
										tempInvGoal = Point()
										tempInvGoal.x = robot_assigned_goal[xxx]['goal'][0]				
										tempInvGoal.y = robot_assigned_goal[xxx]['goal'][1]	
										tempInvGoal.z = 0.0	
										# if the distance is very far then do not publish as invalid point, mostly cant reach goal within the given time limit 
										distance = calculateLocationDistance(robots[xxx].getPosition(), robot_assigned_goal[xxx]['goal'])
										invalid_goal_array.points.append(copy(tempInvGoal))
										invalidFrontier.append(robot_assigned_goal[xxx]['goal'])		
										temp_inv_array.append(robot_assigned_goal[xxx]['goal'])
									if debugFlag1:
										print('Robot: %d assigned goal is repeated in the history' %(winner_id))
									robot_assigned_goal[xxx]['lastgoal']   = faultyGoal
									robot_assigned_goal[xxx]['goal']       = robots[xxx].getPosition()
									robot_assigned_goal[xxx]['time_start'] = rospy.get_rostime().secs
									robot_assigned_goal[xxx]['startLoc']   = robots[xxx].getPosition()
									robot_assigned_goal[xxx]['time_thres'] = -1
									# cancel goal
									rospy.loginfo("\n !!!> Robot " + robot_namelist[xxx] + " give up goal: " + str(robot_assigned_goal[xxx]['goal'])
									+ " at time: "  + str(int(currTime)) + " sec  -- distance: " + str(distance) + " -- goal repeated \n")
									robots[xxx].cancelGoal()
									robot_goal_cancel.append(xxx)
						rospy.sleep(0.1)
					# add delay time after assigning robot to the location
					next_assign_time = rospy.get_rostime().secs + delay_after_assignment
					
	#-------------------------------------------------------------------------	
	# check if the robot assignment time out.
			for ix in range(0,len(robot_namelist)):
				currTime = rospy.get_rostime().secs
				if robot_assigned_goal[ix]['time_thres'] != -1:
					if (((robot_assigned_goal[ix]['time_start'] + robot_assigned_goal[ix]['time_thres']) - currTime) <= 0):
						# for for repeated invalid frontier
						repeatInvalid = False
						distance = calculateLocationDistance(robots[ix].getPosition(), robot_assigned_goal[ix]['goal'])
						for ie in range(0,len(invalidFrontier)):
							if calculateLocationDistance(invalidFrontier[ie], robot_assigned_goal[ix]['goal']) < 0.01:
								repeatInvalid = True
						if repeatInvalid == False:
							tempInvGoal = Point()
							tempInvGoal.x = robot_assigned_goal[ix]['goal'][0]
							tempInvGoal.y = robot_assigned_goal[ix]['goal'][1]
							tempInvGoal.z = 0.0

							# if the distance is very far then do not publish as invalid point, mostly cant reach goal within the given time limit
							if distance <= invalid_distance:
								invalid_goal_array.points.append(copy(tempInvGoal))
								invalidFrontier.append(robot_assigned_goal[ix]['goal'])
								temp_inv_array.append(robot_assigned_goal[ix]['goal'])

						# need to reset the assigned tasks to the robot
						robot_assigned_goal[ix]['lastgoal']   = robot_assigned_goal[ix]['goal'].copy()
						robot_assigned_goal[ix]['goal']       = robots[ix].getPosition()
						robot_assigned_goal[ix]['time_start'] = rospy.get_rostime().secs
						robot_assigned_goal[ix]['startLoc'] = robots[ix].getPosition()
						robot_assigned_goal[ix]['time_thres'] = -1
						# cancel goal
						rospy.loginfo("!!!> Robot " + robot_namelist[ix] + " give up goal: " + str(robot_assigned_goal[ix]['goal'])
						+ " at time: "  + str(currTime) + " sec  -- distance: " + str(distance) + " -- goal repeated")
						robots[ix].cancelGoal()
						robot_goal_cancel.append(ix)

	#-------------------------------------------------------------------------
			# wont print so frequent
			if time_interval_due:
				rospy.loginfo('publishing invalid goals :' + str(temp_inv_array))
				# print out the distance traveled by each of the robot
				for ii in range(0, len(robot_namelist)):
					print("Robot - %d   total distance traveled: %.2fm" %(ii, robots[ii].getDistanceTraveled()))
				
			# publish invalid location for the points
			invPub.publish(invalid_goal_array)
			points.points = copy(invalid_goal_array.points)
			invCenPub.publish(points)
			time_interval_due = False
		else:
			# cancel the robot goal if the signal is terminated
			for ji in range(0, len(robot_namelist)):
				robots[ji].cancelGoal()
				robot_assigned_goal[ji]['lastgoal']   = robot_assigned_goal[ji]['goal'].copy()
				robot_assigned_goal[ji]['goal']       = robots[ji].getPosition()
				robot_assigned_goal[ji]['time_start'] = rospy.get_rostime().secs
				robot_assigned_goal[ji]['startLoc']   = robots[ji].getPosition()
				robot_assigned_goal[ji]['time_thres'] = -1
			rospy.loginfo('-----------------+++ Assigner Module: Terminated Signal Received +++----------------- ')
			rospy.sleep(2.0)
	#------------------------------------------------------------------------- 
		rate.sleep()
	#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
