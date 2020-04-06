#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion
import math
from PIL import Image






def createBlankMapArray(width, height):
    mapArray = []
    for i in range(width):
        tempList = []
        for j in range(height):
            tempList.append(0)
        mapArray.append(tempList)
    return mapArray


def pixelsToImage(pixelArray, imageName):
    width = len(pixelArray)
    height = len(pixelArray[0])

    newImage = Image.new('RGB', (width, height), color="White")
    pixels = newImage.load()

    for i in range(width):
        for j in range(height):
            if pixelArray[i][j]:
                pixels[i, j] = (0, 0, 0, 255)

    newImage.save(imageName)


robot = [0,0,0]
laser_scan = None
goal = None
mapArray = createBlankMapArray(150,150)
places = {}


def robot_callback(data):
    #This function updates the robots position and yaw, based on the ground truth (we don't have localization yet)
    global robot
    [r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    robot = [data.pose.pose.position.x,data.pose.pose.position.y,yaw]

def laser_callback(data):
    #This function sets the global laser_scan variable to hold the most recent laser scan data
    global laser_scan
    laser_scan = data

def goalCallback(data):
    #This function will update the goal of the robot
    global goal
    goal = [data.x, data.y]

def add_forces(a, b):
    #This function adds to force vectors together and returns the result
    assert len(a) == len(b), "Force vectors differ in length"
    c = [a[i] + b[i] for i in range(len(a))]
    return c

def wrap_angle(angle):
    #This function will take any angle and wrap it into the range [-pi, pi]
    while angle >= math.pi:
        angle = angle - 2*math.pi
        
    while angle <= -math.pi:
        angle = angle + 2*math.pi
    return angle

#####################
##### END HELPER FUNCTIONS
#####################

#####################
# BEGIN MODIFIABLE LAB CODE [ALTHOUGH MOST MODIFICATIONS SHOULD BE WHERE SPECIFIED]
#####################

#This function takes in a force [x,y] (in robot coordinates) and returns the drive command (Twist) that should be sent to the robot motors
def drive_from_force(force):
    
    #####################################################
    #PARAMETERS : MODIFY TO GET ROBOT TO MOVE EFFECTIVELY
    
    #This is multiplied by the angle of the drive force to get the turn command 
    turn_multiplier = 1 # lab2c - 90000
    
    #If the absolute value of the angle of the force direction is greater than this, we only spin
    spin_threshold = math.pi/8 # lab2c -/6
    
    #This is multiplied by the magnitude of the force vector to get the drive forward command
    drive_multiplier = 0.5 # lab2c - 10
    
    #END OF PARAMETERS
    #####################################################

    #The twist command to fill out and return
    twist = Twist()

    #Determine the angle and magnitude of the force
    force_angle = math.atan2(force[1],force[0])
    # force_angle = wrap_angle(force_angle - robot[2])
    force_mag = math.hypot(force[0],force[1])

    #Get turn speed
    twist.angular.z = turn_multiplier * force_angle

    #Do we just spin
    if abs(force_angle) < spin_threshold:
        twist.linear.x = drive_multiplier * force_mag

    return twist

# This function determines and returns the attractive force (force_to_goal) to the goal.  
# This force should be in robot coordinates
def goal_force( ):
        
    #This is the robot's actual global location, set in robot_callback
    global robot #format [x_position, y_position, yaw]

    #Goal location is in the global 'goal' variable

    #####################################################
    #PARAMETERS : MODIFY TO GET ROBOT TO MOVE EFFECTIVELY
    
    #Parameter : MODIFY
    #This should be used to scale the magnitude of the attractive goal force
    strength = 50.0 
    
    #END OF PARAMETERS
    #####################################################
    
    force_to_goal = [0,0]    
    
    #########################
    # LAB 2 PART A : BEGIN
    #########################

    # PART A CODE HERE: 
    #    1. Compute goal force vector and put it in the 'force_to_goal' variable

    a_to_goal = math.atan2(goal[1] - robot[1], goal[0] - robot[0] )    
    # Get differeant angle - angle diff
    #Make force in that direction with magnitude a_force    
    force_angle = wrap_angle(a_to_goal - robot[2])
    force_to_goal = [strength*math.cos(force_angle), strength*math.sin(force_angle)]
    # force_to_goal = [strength*math.cos(a_to_goal), strength*math.sin(a_to_goal)]

    # Convert to robot directions (yaw including here)----


    #########################
    # LAB 2 PART A : END
    #########################
    # return [goal[0] - robot[0], goal[1] - robot[1]]
    return force_to_goal


#This function looks at the current laser reading, then computes and returns the obstacle avoidance force vector (in local robot coordinates)
def obstacle_force():  

    #The most recent laser_scan.  It has the following fields 
    #   laser_scan.angle_min : angle of the first distance reading
    #   laser_scan.angle_increment : the angular difference between consecutive distance readings
    #   laser_scan.ranges : an array all of the distance readings
    global laser_scan
    global mapArray
    
    #Only run if we have a laser scan to work with
    if laser_scan is None:
        return [0,0]

    #The obstacle repulsion force variable, will be returned
    #This will accumulate all of the obstacle forces acting upon us
    force_from_obstacles = [0,0]

    cur_angle = laser_scan.angle_min 
    #cur_angle will always have the relative angle between the robot's yaw and the current laser reading

    for i in range(len(laser_scan.ranges)):

        # Get the magnitude of the repulsive force for this distance reading
        # CHANGE WHICH FUNCTION IS CALLED FOR LAB 2 PART C
        # strength = get_pf_magnitude_constant(laser_scan.ranges[i])
        strength = get_pf_magnitude_linear(laser_scan.ranges[i])


        # if (cur_angle < 1.5708 and cur_angle > -1.5708):
        angleToUse = cur_angle + robot[2]
        distance = laser_scan.ranges[i]
        yDelta = math.sin(angleToUse)*distance
        xDelta = math.cos(angleToUse)*distance
        mapScaler = 9.5
        p = [robot[0] + xDelta, robot[1] + yDelta]
        p[0] += 7.5
        p[1] -= 7.5
        p[1] = abs(p[1])

        p[0] = int(round(p[0]*mapScaler))
        p[1] = int(round(p[1]*mapScaler))

        if p[0] > 149: p[0] = 149
        if p[1] > 149: p[1] = 149


        try:
            mapArray[p[0]][p[1]] = True
            places[str(p[0])+","+str(p[1])] = True
        except:
            print "Out of bounds: " + str(p)

        x = -strength * math.cos(wrap_angle(cur_angle)) # Make negative if not on part D
        y = -strength * math.sin(wrap_angle(cur_angle)) # Make negative if not on part D
        force_from_obstacles[0] += x
        force_from_obstacles[1] += y

        cur_angle = cur_angle + laser_scan.angle_increment

    return force_from_obstacles

# This function returns the magnitude of repulsive force for the input distance
# using a linear drop-off function
def get_pf_magnitude_linear(distance):

    #####################################################
    #PARAMETERS: MODIFY TO GET THINGS WORKING EFFECTIVELY
        
    #How close to the obstacle do we have to be to begin feeling repulsive force
    distance_threshold = 0.5  # lab2c - 0.65

    #The maximum strength of the repulsive force
    max_strength = 1000.0

    #END OF PARAMETERS
    #####################################################
    
    #########################
    # LAB 2 PART C : BEGIN
    #########################

    # PART C CODE HERE: 
    #   1. Compute the magnitude of the force for the given distance and return it
    if (distance < distance_threshold):
        # return 1/(distance/5) if 1/(distance/5) < max_strength else max_strength
        strength = (distance_threshold - distance)/distance_threshold# lab2c * 10
        # print("Feeling repulsive strength")
        return strength

    #########################
    # LAB 2 PART C : END
    #########################

    return 0

# This function returns the magnitude of repulsive force for the input distance
# using a constant value if the obstacles is closer than a threshold
def get_pf_magnitude_constant(distance):

    #####################################################
    #PARAMETERS: MODIFY TO GET THINGS WORKING EFFECTIVELY
        
    #How close to the obstacle do we have to be to begin feeling repulsive force
    distance_threshold = 5.0

    #Strength of the repulsive force
    strength = 50.0

    #END OF PARAMETERS
    #####################################################

    if distance < distance_threshold:
        return strength

    return 0


# This is the main loop of the lab code.  It runs continuously, navigating our robot
# (hopefully) towards the goal, without hitting any obstacles
def potential():
    rospy.init_node('lab2', anonymous=True) #Initialize the ros node
    pub = rospy.Publisher('cmd_vel', Twist) #Create our publisher to send drive commands to the robot
    rospy.Subscriber("base_scan", LaserScan, laser_callback) #Subscribe to the laser scan topic
    rospy.Subscriber("base_pose_ground_truth", Odometry, robot_callback) #Subscribe to the robot pose topic
    rospy.Subscriber("next_waypoint", Point, goalCallback)#Subscribe to the goal location topic

    rate = rospy.Rate(10) #10 Hz    
    show = True
    count = 0
    
    while not rospy.is_shutdown():
        count += 1
        #Don't do anything until the goal location has been received
        if goal is None:
            rate.sleep()
            continue

        if show:
            print "X: " + str(robot[0]) , "Y: " + str(robot[1]) , "Yaw: " + str(robot[2])
            # print laser_scan.angle_min
            print ""
        show = not show

        # twist = Twist()
        # twist.angular.z = 50


        #1. Compute attractive force to goal
        g_force = goal_force()
        # print "G_force: " + str(g_force)
        
        #2. Compute obstacle avoidance force
        o_force = obstacle_force()
        # print "O_Force: " + str(o_force)

        #3. Get total force by adding together
        total_force = add_forces(g_force, o_force)
        # print "Total_force: " + str(total_force)
        
        #4. Get final drive command from total force
        twist = drive_from_force(total_force) 

        #5. Publish drive command, then sleep 
        pub.publish(twist)
        # print ""
        rate.sleep() #sleep until the next time to publish

        if count == 400:
            print "Saving file..."
            pixelsToImage(mapArray, "/home/patrick/catkin_ws/src/ai_labs/testImage.png")
            print(places.keys())
            break
        

    #Send empty twist command to make sure robot stops
    twist = Twist()
    pub.publish(twist)

if __name__ == '__main__':
    try:
        potential()
    except rospy.ROSInterruptException:
        pass
