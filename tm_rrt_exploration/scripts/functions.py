import rospy
import tf
from numpy import array, number, record
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from numpy import floor
from numpy.linalg import norm
from numpy import inf
import numpy as np
# import random
# ________________________________________________________________________________


class robot:
    def __init__(self, name, move_base_service, in_service, global_frame, base_link):
        rospy.loginfo('setting up robot init for robot - ' + name)	
        self.assigned_point = []
        self.goal_history = []
        self.name = name
        #################################################################
        self.goal  = MoveBaseGoal()
        self.start = PoseStamped()
        self.end   = PoseStamped()
        #################################################################        
        self.global_frame = rospy.get_param('~global_frame', global_frame)
        self.robot_frame = rospy.get_param('~robot_frame', base_link)
        self.plan_service =  rospy.get_param('~plan_service', in_service)
        # print("plan service" + in_service)
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0.0), rospy.Duration(10.0))
        ###################################################################
        self.total_distance  = 0.0
        self.first_run       = True
        self.movebase_status = 0
        self.sub       = rospy.Subscriber(name + "/odom", Odometry, self.odom_callback)
        self.statussub = rospy.Subscriber(name + "/move_base/status", GoalStatusArray, self.movebase_status_callback)
        ###################################################################
        cond = 0
        while cond == 0:
            try:
                rospy.loginfo('Waiting for the robot transform')
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0.0))
                # print(self.name+'/'+self.robot_frame)
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.sleep(0.1)
                cond = 0
        #####################################################################
        self.position = array([trans[0], trans[1]])
        self.previous_x = 0
        self.previous_y = 0
        self.assigned_point = self.position
        # print('waiting for the move_base service - %s' %(self.name+move_base_service))
        self.client = actionlib.SimpleActionClient('/' + self.name+move_base_service, MoveBaseAction)
        self.client.wait_for_server()
        self.goal.target_pose.header.frame_id = self.global_frame
        self.goal.target_pose.header.stamp = rospy.Time.now()
        ######################################################################
        # print('waiting for the plan service -%s' %('/' + self.name+self.plan_service))
        rospy.wait_for_service(self.name+self.plan_service)
        self.make_plan = rospy.ServiceProxy(self.name+self.plan_service, GetPlan)

        self.start.header.frame_id = self.global_frame
        self.end.header.frame_id = self.global_frame
        # print('done setting up robot')
        #######################################################################


    def odom_callback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        if self.first_run == True:
            self.previous_x = x
            self.previous_y = y
        d_increment = np.sqrt(((x - self.previous_x)*(x - self.previous_x)) + ((y - self.previous_y)*(y - self.previous_y)))
        self.total_distance = self.total_distance + d_increment
        # print("Total distance traveled is %.2f" %(self.total_distance))
        self.first_run = False
        self.previous_x = x
        self.previous_y = y
    
    def movebase_status_callback(self, data):
        if len(data.status_list) > 0:
            self.movebase_status = max([status.status for status in data.status_list])
        # uint8 PENDING=0
        # uint8 ACTIVE=1
        # uint8 PREEMPTED=2
        # uint8 SUCCEEDED=3
        # uint8 ABORTED=4
        # uint8 REJECTED=5
        # uint8 PREEMPTING=6
        # uint8 RECALLING=7
        # uint8 RECALLED=8
    
    def get_movebase_status(self):
        return self.movebase_status

    def getDistanceTraveled(self):
        return self.total_distance

    def getPosition(self, quad=False):
        cond    = 0
        # attempt = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond = 0
        self.position = array([trans[0], trans[1]])

        if not quad:
            return self.position
        else:
            return self.position, rot

    def transformPointToRobotFrame(self, trans):
        while not rospy.is_shutdown():
            try:
                point = PoseStamped()
                t = self.listener.getLatestCommonTime(self.name + '/' + self.robot_frame, self.global_frame)
                point.header.frame_id = self.global_frame
                point.pose.position.x = trans[0]
                point.pose.position.y = trans[1]
                point.pose.orientation.w = 1.0
                transformed = self.listener.transformPose(self.name + '/' + self.robot_frame, point)
                return (transformed.pose.position.x, transformed.pose.position.y)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
    
    def sendGoalTransformed(self, point):
        transform_point = self.transformPointToRobotFrame(point)
        self.goal.target_pose.pose.position.x = transform_point[0]
        self.goal.target_pose.pose.position.y = transform_point[1]
        self.goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(self.goal)
        self.goal_history.append(array(point))
        self.assigned_point = array(point)
        
    def sendGoal(self, point, quadData=None):
        if quadData is None:
            self.goal.target_pose.pose.position.x = point[0]
            self.goal.target_pose.pose.position.y = point[1]
            self.goal.target_pose.pose.orientation.w = 1.0
        else:
            self.goal.target_pose.pose.position.x = point[0]
            self.goal.target_pose.pose.position.y = point[1]
            self.goal.target_pose.pose.orientation.x = quadData[0]
            self.goal.target_pose.pose.orientation.y = quadData[1]
            self.goal.target_pose.pose.orientation.z = quadData[2]
            self.goal.target_pose.pose.orientation.w = quadData[3]
        self.client.send_goal(self.goal)
        self.assigned_point = array(point)

    def cancelGoal(self, noQuadFlag=False):
        if noQuadFlag:
            point = self.getPosition()
            self.goal.target_pose.pose.position.x = point[0]
            self.goal.target_pose.pose.position.y = point[1]
            self.goal.target_pose.pose.orientation.w = 1.0
        else:
            point, quadData = self.getPosition(quad=True)
            self.goal.target_pose.pose.position.x = point[0]
            self.goal.target_pose.pose.position.y = point[1]
            self.goal.target_pose.pose.orientation.x = quadData[0]
            self.goal.target_pose.pose.orientation.y = quadData[1]
            self.goal.target_pose.pose.orientation.z = quadData[2]
            self.goal.target_pose.pose.orientation.w = quadData[3]
        # send the goal
        self.client.send_goal(self.goal)

    def getState(self):
        return self.client.get_state()

    def makePlan(self, start, end):
        self.start.pose.position.x = start[0]
        self.start.pose.position.y = start[1]
        self.end.pose.position.x = end[0]
        self.end.pose.position.y = end[1]
        start = self.listener.transformPose(self.name+'/map', self.start)
        end = self.listener.transformPose(self.name+'/map', self.end)
        plan = self.make_plan(start=start, goal=end, tolerance=0.2)
        return plan.plan.poses
    
    def setGoalHistory(self, point):
        self.goal_history.append(point)

    def getGoalHistory(self):
        return self.goal_history
            
# ________________________________________________________________________________


def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data
    index = int((floor((Xp[1]-Xstarty)/resolution) *
                  width)+(floor((Xp[0]-Xstartx)/resolution)))
    return index


def point_of_index(mapData, i):
    y = mapData.info.origin.position.y + \
        (i/mapData.info.width)*mapData.info.resolution
    x = mapData.info.origin.position.x + \
        (float(i-(int(i/mapData.info.width)*(mapData.info.width)))*mapData.info.resolution)
    # modified for certain python version might mismatch the int and float conversion
    return array([x, y])
# ________________________________________________________________________________


def informationGain(mapData, point, r):
    infoGain = 0.0
    index = index_of_point(mapData, point)
    r_region = int(r/mapData.info.resolution)
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if(mapData.data[i] == -1 and norm(array(point)-point_of_index(mapData, i)) <= r):
                    infoGain = infoGain + 1.0
    return infoGain*(mapData.info.resolution**2)
    # need to check the resolution
# ________________________________________________________________________________


def discount(mapData, assigned_pt, centroids, infoGain, r):
    index = index_of_point(mapData, assigned_pt)
    r_region = int(r/mapData.info.resolution)
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                for j in range(0, len(centroids)):
                    current_pt = centroids[j]
                    if(mapData.data[i] == -1 and norm(point_of_index(mapData, i)-current_pt) <= r and norm(point_of_index(mapData, i)-assigned_pt) <= r):
                        # this should be modified, subtract the area of a cell, not 1
                        infoGain[j] = infoGain[j] - 1.0
    return infoGain


def discount2(mapData, assigned_pt, centroids, infoGain, r):
    for j in range(0, len(infoGain)):
        temp_infoGain = 0.0
        index = index_of_point(mapData, centroids[j])
        r_region = int(r/mapData.info.resolution)
        init_index = index-r_region*(mapData.info.width+1)
        for n in range(0, 2*r_region+1):
            start = n*mapData.info.width+init_index
            end = start+2*r_region
            limit = ((start/mapData.info.width)+2)*mapData.info.width
            for i in range(start, end+1):
                if (i >= 0 and i < limit and i < len(mapData.data)):
                    if(mapData.data[i] == -1 and norm(array(centroids[j]) - point_of_index(mapData, i)) <= r and norm(point_of_index(mapData, i)-assigned_pt) <= r):
                        temp_infoGain += 1.0
        infoGain[j] -= (temp_infoGain*(mapData.info.resolution**2))
    return infoGain
# ________________________________________________________________________________
def relativePositionMetric(inputLoc, robotIndex, robots_position, distance_threshold=3.0):
    if len(robots_position) > 1:
        inputList = []
        for i in range(0, len(robots_position)):
            if i != robotIndex:
                inputList.append(robots_position[i])
        # now calculate relative position for each of the robot in the list 
        distanceList = []
        for j in inputList:
            distanceList.append(calculateLocationDistance(j, inputLoc))
        # now get the largest distance and perform calculation on the data
        minDistance = min(distanceList)
        if minDistance > distance_threshold:
            return 1.0
        else:
            temp = minDistance/distance_threshold
            if temp < 0.01:
                return 0.01
            else:
                return temp
    else:
        return 1.0

# ________________________________________________________________________________
def getMaxRevenueForRobot(revenue_record, id_record, robotID):
    extracted_index   = []
    extracted_revenue = []
    # assuming both revenue record and id_record have the same length
    for i in range(0, len(id_record)):
        if id_record[i] == robotID:
            extracted_index.append(i)
            extracted_revenue.append(revenue_record[i])
    # now extracting the max value
    if len(extracted_revenue) > 0:
        max_idx = extracted_revenue.index(max(extracted_revenue))
        return extracted_index[max_idx]
    else:
        return -1 # denote skip
    # given max_idx
# ________________________________________________________________________________
def calculateLocationDistance(input_loc, dest_loc):
    return(np.sqrt(np.power(input_loc[0]-dest_loc[0],2) + np.power(input_loc[1]-dest_loc[1],2)))

# ________________________________________________________________________________
def calcDynamicTimeThreshold(curr_pos, goal_pos, time_per_meter):
    # this function use to dynamic determine the mission time threshold for the robot
    dist1 = np.sqrt(np.power(curr_pos[0]-goal_pos[0],2) + np.power(curr_pos[1]-goal_pos[1],2))
    if dist1 < 1.0:
        return time_per_meter
    elif dist1 >= 1.0 and dist1 <= 10.0:
        return int(time_per_meter * np.abs(dist1))
    else:
        return int(time_per_meter * 10.0)
    

# ________________________________________________________________________________
def pathCost(path):
    if (len(path) > 0):
        i = len(path)/2
        p1 = array([path[i-1].pose.position.x, path[i-1].pose.position.y])
        p2 = array([path[i].pose.position.x, path[i].pose.position.y])
        return norm(p1-p2)*(len(path)-1)
    else:
        return inf
# ________________________________________________________________________________


def unvalid(mapData, pt):
    index = index_of_point(mapData, pt)
    r_region = 5
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if(mapData.data[i] == 1):
                    return True
    return False
# ________________________________________________________________________________


def Nearest(V, x):
    n = inf
    i = 0
    for i in range(0, V.shape[0]):
        n1 = norm(V[i, :]-x)
        if (n1 < n):
            n = n1
            result = i
    return result

# ________________________________________________________________________________


def Nearest2(V, x):
    n = inf
    result = 0
    for i in range(0, len(V)):
        n1 = norm(V[i]-x)

        if (n1 < n):
            n = n1
    return i
# ________________________________________________________________________________

def gridValueMergedMap(mapData, Xp, distance=2):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data  = mapData.data
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    index = (floor((Xp[1]-Xstarty)/resolution)*width) + \
        (floor((Xp[0]-Xstartx)/resolution))

    outData = squareAreaCheck(Data, index, width, distance)    
    if len(outData) > 1:
        if 100 not in outData:
            if max(outData,key=outData.count) == -1 and max(outData) == 0:
                return -1
            elif max(outData,key=outData.count) == -1 and max(outData) == -1 and 0 not in outData:
                return 100
            elif max(outData,key=outData.count) == -1 and max(outData) == -1 and 0 in outData:
                return -1
            elif max(outData,key=outData.count) == 0 and -1 in outData:
                return -1
            else:
                return -1
        else:
            return 100
    else:
        return 100

# ________________________________________________________________________________
def squareAreaCheck(data, index, width, distance=2):
    # now using the data to perform a square area check on the data for removing the invalid point
    dataOutList = []
    for j in range(-1*distance, distance+1):
        for i in range(int(index)+((width*j)-distance), int(index)+((width*j)+distance)):
            if i < len(data):
                dataOutList.append(data[int(i)])
    return dataOutList

# ________________________________________________________________________________
def gridValue(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data = mapData.data
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    index = (floor((Xp[1]-Xstarty)/resolution)*width) + \
        (floor((Xp[0]-Xstartx)/resolution))

    outData = squareAreaCheck(Data, index, width, distance=2)
    if len(outData) > 1:
        return max(outData)
    else:
        return 100
# ________________________________________________________________________________
def getHigestIndex(revenueList, attempt):
    # get the maximum index number 
    if len(revenueList) > 0:
        tempList = sorted(revenueList, reverse=True)
        maxValue = tempList[attempt]
        index = revenueList.index(maxValue)
        return index
    else:
        return -1
# ________________________________________________________________________________
