#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

pose = [0,0,0]
regions = {0,0,0,0,0}

def func(x):                                                                                               #task1.1 path                                      
    return 2*(math.sin(x))*(math.sin(x/2))                          

def odom_callback(data):                                                                                   #odometry function
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

def laser_callback(msg):            																	   #laserscan function
    global regions
    regions = {
		'front'  : min(msg.ranges[2*720/5:3*720/5]),
		'fleft'  : min(msg.ranges[3*720/5:(4*720/5)-80]),
        'fright' : min(msg.ranges[720/5 + 80:2*720/5]),
        'bleft'  : min(msg.ranges[(4*720/5)+80:720]),
        'bright' : min(msg.ranges[:720/5 + 80])
    }

def check_obs(angle):                                                                                       #checking for obstacle in path of bot and goal

	if (angle > -72) and (angle < -27):
		return bool(regions['fleft'] < 500)
	elif (angle > -27) and (angle < 27):
		return bool(regions['front'] < 500)
	elif (angle > 27) and (angle < 72):
		return bool(regions['fright'] < 500)
	else:
		return False


def control_loop():                                                                                        #main loop to control bot
    rospy.init_node('ebot_controller')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0

    pub.publish(velocity_msg)

    dist = 0.1

    while not rospy.is_shutdown():                                                                           #for task1.1 function path
        p = 1.5                                             
        n_x = pose[0] + dist 
        n_y = func(n_x)             
        ntheta = math.atan((n_y - pose[1]) / dist)
        velocity_msg.linear.x = 0.1                          
        velocity_msg.angular.z = p * (ntheta-pose[2]) 
        pub.publish(velocity_msg)
        if pose[0] > 2 * math.pi - 0.22: break 
        rate.sleep()

    for _ in range(10):
    	velocity_msg.linear.x = 0.3
    	velocity_msg.angular.z = 0
    	pub.publish(velocity_msg)
    	rate.sleep()

    p=0.6

    while not rospy.is_shutdown():                                                                            #for avoiding obstacle and reaching goal

    	fi = math.atan(-1*pose[1]/(12.6 - pose[0]))
    	final_ang = (fi - pose[2])*180/math.pi

    	if not check_obs(-1 * final_ang):
    		velocity_msg.linear.x = 0.1
    		velocity_msg.angular.z = p*final_ang*math.pi/180
    		pub.publish(velocity_msg)
    		if pose[0]>=12.5:                                                                                 #for stopping at the goal
    			velocity_msg.linear.x = 0
    			velocity_msg.angular.z = 0
    			pub.publish(velocity_msg)
    			break
    	else:
    		mask_dir = [(x>1000) for x in regions.values()]      
    		lst_dir = [0 , 54, -54, 108, -108]
    		for _dir in zip(mask_dir, lst_dir):
    			if _dir[0]:
    				velocity_msg.linear.x = 0.05
    				velocity_msg.angular.z = p*_dir[1]*math.pi/180
    				pub.publish(velocity_msg)
    		if pose[0]>=12.5:                                                                                  #for stopping at the goal
    			velocity_msg.linear.x = 0
    			velocity_msg.angular.z = 0
    			pub.publish(velocity_msg)
    			break
		rate.sleep()


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
