#!/usr/bin/env python3 
 
import rospy 
import math 
 
import tf2_ros 
from tf.transformations import * 
from geometry_msgs.msg import Quaternion 
import tf2_geometry_msgs 
 
from robot_vision_lectures.msg import XYZarray 
from robot_vision_lectures.msg import SphereParams 
 
# import the plan message 
from ur5e_control.msg import Plan 
from geometry_msgs.msg import Twist 
 
# For pause/unpause feature 
from std_msgs.msg import Bool 
from std_msgs.msg import UInt8 
 

filterered_data_points  = SphereParams() 
motion_check = Bool()
 
# Set values to SphereParam after doing filtering 
def get_sphere_params(data): 
    global filterered_data_points 
 
    filterered_data_points.xc = data.xc
    filterered_data_points.yc = data.yc 
    filterered_data_points.zc = data.zc 
    filterered_data_points.radius = data.radius 
 
 
def set_new_params(plan, new_x, new_y, new_z, mode): 
    plan_point = Twist() 
 
    point_mode = UInt8() 
    # just a quick solution to send two target points 
    # define a point close to the initial position 
    plan_point.linear.x = new_x 
    plan_point.linear.y = new_y 
    plan_point.linear.z = new_z 
    plan_point.angular.x = 3.126 
    plan_point.angular.y = 0.016
    plan_point.angular.z = 1.530
 
    #mode =0 - Regular Motion Mode, 1 - Open the gripper, 2 - Close the gripper 
    #point_mode.data = mode 
 
    # add this point to the plan 
    plan.points.append(plan_point) 
    #plan.modes.append(point_mode) 
    
    print("This is the plan point \n", plan_point)
    return plan
 
 
def check_stuff(data):
	global motion_check
	motion_check = data
	
 
 
if __name__ == '__main__': 
    # initialize the node 
    rospy.init_node('simple_planner', anonymous=True) 
 
    # add a publisher for sending joint position commands 
    plan_pub = rospy.Publisher('/plan', Plan, queue_size=10) 
 
    # Get sphere params 
    rospy.Subscriber('/sphere_params', SphereParams, get_sphere_params) 
    
    #
    rospy.Subscriber('/motion_starter', Bool, check_stuff) 
 
 
 
    # set a 10Hz frequency for this loop 
    loop_rate = rospy.Rate(10) 
 
    # define a plan variable 
    plan = Plan() 
 
    tfBuffer = tf2_ros.Buffer() 
    listener = tf2_ros.TransformListener(tfBuffer) 
    
    q_rot = Quaternion()
 
 
    plan = set_new_params(plan, -0.016, -0.406, 0.429, 0)  
 
    start_x = -0.016
    start_y = -0.406
    start_z = 0.429
 
    flag = False 

    while not rospy.is_shutdown(): 
 
        # try getting the most update transformation between the tool frame and the base frame 
        try: 
            trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time()) 
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException): 
            print('Frames not available!!!') 
            loop_rate.sleep() 
            continue 
 
        # extract the xyz coordinates 
        x = trans.transform.translation.x 
        y = trans.transform.translation.y 
        z = trans.transform.translation.z 
        # extract the quaternion and converto RPY 
        q_rot = trans.transform.rotation 
        roll, pitch, yaw, = euler_from_quaternion([q_rot.x, q_rot.y, q_rot.z, q_rot.w]) 
 
        # define a testpoint in the tool frame (let's say 10 cm away from flange) 
        pt_in_tool = tf2_geometry_msgs.PointStamped() 
        pt_in_tool.header.frame_id = 'camera_color_optical_frame' 
        pt_in_tool.header.stamp = rospy.get_rostime() 
 
        pt_in_tool.point.x = filterered_data_points.xc 
        pt_in_tool.point.y = filterered_data_points.yc 
        pt_in_tool.point.z = filterered_data_points.zc 
 
        # convert the 3D point to the base frame coordinates 
        pt_in_base = tfBuffer.transform(pt_in_tool, 'base', rospy.Duration(1.0)) 
 

 
        if flag == False: 
            # the_answers = the_magic() 
 
            #Directly Over Ball 
            plan = set_new_params(plan, pt_in_base.point.x, pt_in_base.point.y, start_z, 0) 
 
            #to ball 
            plan = set_new_params(plan, pt_in_base.point.x, pt_in_base.point.y, pt_in_base.point.z, 0) 
            
            #Close gripper 
            #plan = set_new_params(plan, pt_in_base.point.x, pt_in_base.point.y, pt_in_base.point.z, 2) 
 
            #Back up 
            plan = set_new_params(plan, pt_in_base.point.x, pt_in_base.point.y, start_z, 0) 
 
            #Original Spot 
            plan = set_new_params(plan, start_x, start_y, start_z, 0) 
 
            #Dropoff point 
            plan = set_new_params(plan, start_x, start_y, pt_in_base.point.z, 0) 
            #plan = set_new_params(plan, start_x, start_y, pt_in_base.point.z, 1) 
 
            flag = True 
            
           
        # publish the plan 
        if motion_check.data:
        	plan_pub.publish(plan) 
 
        # wait for 0.1 seconds until the next loop and repeat 
        loop_rate.sleep() 

 
