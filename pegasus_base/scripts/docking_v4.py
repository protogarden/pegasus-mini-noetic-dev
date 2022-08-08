#!/usr/bin/env python
import rospy 
import tf
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import std_msgs.msg
from nav_msgs.msg  import Odometry
from geometry_msgs.msg import Point, Twist, PoseStamped, Pose
from tf.transformations import euler_from_quaternion 
from math import atan2, pow, sqrt, pi
import math 
import time
from std_msgs.msg import Int32

import RPi.GPIO as GPIO
import time


class DockingInterface():
    def __init__(self):
        self.pid_staging_alignment_gain = rospy.get_param('~pid_staging_alignment_gain', 0.6) 
        self.pid_staging_alignment_output_limit = rospy.get_param('~pid_staging_alignment_output_limit', 0.2) 

        self.pid_staging_tag_detect_gain = rospy.get_param('~pid_staging_tag_detect_gain', 0.3) 
        self.pid_staging_tag_detect_output_limits = rospy.get_param('~pid_staging_tag_detect_output_limits', 0.2) 

        self.pid_staging_angular_gain = rospy.get_param('~pid_staging_angular_gain', 0.3) 
        self.pid_staging_angular_output_limits = rospy.get_param('~pid_staging_angular_output_limits', 0.1)

        self.pid_staging_linear_gain = rospy.get_param('~pid_staging_linear_gain', 0.5)
        self.pid_staging_linear_output_limits = rospy.get_param('~pid_staging_linear_output_limits', 0.08)

        self.pid_final_alignment_gain = rospy.get_param('~pid_final_alignment_gain', 0.5)
        self.pid_final_alignment_output_limit = rospy.get_param('~pid_final_alignment_output_limit', 0.05)

        self.docking_bundle_name = rospy.get_param('~docking_bundle_name', 'DOCKING_TAGS') 
        self.left_bundle_tag = rospy.get_param('~left_bundle_tag', 'LEFT-TAG') 
        self.right_bundle_tag = rospy.get_param('~right_bundle_tag', 'RIGHT-TAG') 
        self.alignment_docking_tag_name = rospy.get_param('~alignment_tag', 'TAG') 
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_link_frame = rospy.get_param('~base_link_frame', 'base_link') 
        
        self.stage_1_dist = rospy.get_param('~stage_1_dist', 0.8)
        self.stage_2_dist = rospy.get_param('~stage_2_dist', 0.6)
        self.stage_3_dist = rospy.get_param('~stage_3_dist', 0.5)
        self.engage_dist = rospy.get_param('~engage_dist', 0.095)

        self.stage_1_radius_tolerance = rospy.get_param('~stage_1_radius_tolerance', 0.03)
        self.stage_2_radius_tolerance = rospy.get_param('~stage_2_radius_tolerance', 0.03)
        self.stage_3_radius_tolerance = rospy.get_param('~stage_3_radius_tolerance', 0.025)

        self.stage_align_thresh = rospy.get_param('~stage_alignment_tolerance', 0.008)
        self.final_align_thresh = rospy.get_param('~final_alignment_tolerance', 0.008)

        self.camera_freq = rospy.get_param('~framerate', 5)
        self.tag_check_ratio = rospy.get_param('~tag_detect_ratio', 2)
        self.tag_check_freq = self.camera_freq*self.tag_check_ratio
        

        self.odom_freq = rospy.get_param('~odom_frequency', 100)
        self.odom_update_ratio = rospy.get_param('~odom_update_ratio', 0.5)
        self.odom_update_freq = self.odom_freq*self.odom_update_ratio
        
        self.undock_linear_speed = 0.02

        self.current_xpose = 0
        self.current_ypose = 0
        self.current_theta = 0

        self.dt = 0
        self.dt_left = 0
        self.dt_right = 0

        self.docking_cmd = 0
        self.docking_feedback = 1
        self.dock_feedback = 3
        self.undocking_feedback = 2
        self.undock_feedback = 4

        self.driving_led = 1
        self.charging_led = 2
        self.docking_led = 3
        self.finnish_charging = 4
        self.startup_led = 5
        self.off_led = 6


        self.charge_input_pin = 18
        GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
        GPIO.setup(self.charge_input_pin, GPIO.IN)  # set pin as an input pin



      
        
    def pid(self, gain, input, output_limit):

        output = gain*input
        if output > output_limit:
            output = output_limit 
        elif output < -output_limit:
            output = - output_limit
        return output

    def undock(self):

        
        self.dock_feedback_pub.publish(self.undocking_feedback)

        stage_linear_speed = -0.04
        stage_angular_speed = 0
        self.speed.linear.x = stage_linear_speed
        self.speed.angular.z = stage_angular_speed
        self.pub.publish(self.speed)
        time.sleep(10)

        stage_linear_speed = 0
        stage_angular_speed = 0
        self.speed.linear.x = stage_linear_speed
        self.speed.angular.z = stage_angular_speed
        self.pub.publish(self.speed)
        
        self.dock_feedback_pub.publish(self.undock_feedback)


    def dock(self):
        
        def angle_check(angle_diff, goal_angle): #check for robot 180 to -180 problem causing robot to try go long way round due to angle difference
            #print("check")
            if abs(angle_diff) > math.pi and self.current_theta > 0:
                angle_diff = (2*math.pi + atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose)) - self.current_theta
            elif abs(angle_diff) > math.pi and self.current_theta < 0:
                angle_diff = -((2*math.pi - atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose)) + self.current_theta)
            
            return angle_diff

        self.dock_feedback_pub.publish(self.docking_feedback)
        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)

        ##stage 1
        ##stage 1.1: Turning to angle at distance to docking
        
        goal_xpose, goal_ypose, goal_thetha = self.pose_update_staging(self.stage_1_dist)
        goal_angle = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose)
        
        angle_diff = goal_angle - self.current_theta
        print("Stage 1 - current thetha",self.current_theta)
        print("Stage 1 - goal thetha", goal_angle)
        print("Stage 1 - angle diff", angle_diff)
        angle_diff = angle_check(angle_diff, goal_angle)
        print('Turning towards goal pose - Stage 1')
        while angle_diff > self.stage_align_thresh or angle_diff < -self.stage_align_thresh:
            #goal_xpose, goal_ypose, goal_thetha = pose_update_staging(stage_1_dist)
            angle_diff = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose) - self.current_theta
            stage_angular_speed = self.pid(self.pid_staging_alignment_gain, angle_diff, self.pid_staging_alignment_output_limit)
            self.speed.linear.x = 0.0
            self.speed.angular.z = stage_angular_speed
            self.pub.publish(self.speed)
            self.odom_update_rate.sleep()
        

        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
        time.sleep(0.2)
        print('pointing towards goal pose - Stage 1')
        
        ##stage 1.2: Going to point from docking 

        print('moving towards goal pose - Stage 1')
        while sqrt(pow((goal_xpose - self.current_xpose), 2) + pow((goal_ypose - self.current_ypose), 2)) > self.stage_1_radius_tolerance:
            angle_diff = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose) - self.current_theta
            euclidean_distance = sqrt(pow((goal_xpose - self.current_xpose), 2) + pow((goal_ypose - self.current_ypose), 2))
            stage_linear_speed = abs(self.pid(self.pid_staging_linear_gain, euclidean_distance, self.pid_staging_linear_output_limits))
            stage_angular_speed = self.pid(self.pid_staging_angular_gain, angle_diff, self.pid_staging_angular_output_limits)
            self.speed.linear.x = stage_linear_speed
            self.speed.angular.z = stage_angular_speed
            self.pub.publish(self.speed)
            self.odom_update_rate.sleep()

    
        print('reached goal pose - Stage 1')
        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)

        #Stage 2
        ##stage 2.1: Turning to angle at distance to docking

        self.current_time = rospy.get_time()
        print('checking dt')
        self.left_bundle_tag_detect()
        self.right_bundle_tag_detect()
        print('dt',self.dt)

        #goal_xpose, goal_ypose, goal_thetha = pose_update_staging(stage_1_dist) 
        angle_diff = goal_thetha - self.current_theta
        angle_diff = angle_check(angle_diff, goal_thetha)
        print('Turning bot until to goal orientation until left/right tag detected')
       
        ##The following while loop turn the bot to goal orientation until april tag is detected as which point it moves to the bot in a manner to orientation it towards the next docking station distance goal

        stage_angular_speed = self.pid(self.pid_staging_tag_detect_gain, angle_diff, self.pid_staging_tag_detect_output_limits)
        while True:
    
            self.speed.linear.x = 0.0
            self.speed.angular.z = stage_angular_speed
            

            self.left_bundle_tag_detect()
            self.right_bundle_tag_detect()
            

            if self.dt_left > 0 and self.dt_right > 0:

                print('dt_left',self.dt_left)
                print('dt_right',self.dt_right)
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                time.sleep(1)

                print("tag detected , turning towards Stage-2 offset")
                goal_xpose, goal_ypose, goal_thetha = self.pose_update_staging(self.stage_2_dist)
                goal_angle = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose)
                print("Stage 2 - current thetha",self.current_theta)
                print("Stage 2 - goal thetha", goal_angle)
                print("Stage 2 - angle diff", angle_diff)
                angle_diff = goal_angle - self.current_theta
                angle_diff = angle_check(angle_diff, goal_angle)
                
                while angle_diff > self.stage_align_thresh or angle_diff < -self.stage_align_thresh:
                    goal_angle = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose)
                    angle_diff = goal_angle - self.current_theta
                    angle_diff = angle_check(angle_diff, goal_angle)
                    stage_angular_speed = self.pid(self.pid_staging_alignment_gain, angle_diff, self.pid_staging_alignment_output_limit)
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = stage_angular_speed
                    self.pub.publish(self.speed)
                    self.odom_update_rate.sleep()
    
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                break


            self.pub.publish(self.speed)
            self.tag_check_rate.sleep()

        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
        print('pointing towards goal pose - Stage 2')
        time.sleep(0.1)

        print('moving towards goal pose - Stage 2')
        while sqrt(pow((goal_xpose - self.current_xpose), 2) + pow((goal_ypose - self.current_ypose), 2)) > self.stage_2_radius_tolerance:
            angle_diff = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose) - self.current_theta
            euclidean_distance = sqrt(pow((goal_xpose - self.current_xpose), 2) + pow((goal_ypose - self.current_ypose), 2))
            stage_linear_speed = abs(self.pid(self.pid_staging_linear_gain, euclidean_distance, self.pid_staging_linear_output_limits))
            stage_angular_speed = self.pid(self.pid_staging_angular_gain, angle_diff, self.pid_staging_angular_output_limits)
            self.speed.linear.x = stage_linear_speed
            self.speed.angular.z = stage_angular_speed
            self.pub.publish(self.speed)
            self.odom_update_rate.sleep()


        print('At goal pose - Stage 2')
        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
        
        ##stage 2.1: Turning tp angle at distance to docking

        self.current_time = rospy.get_time()
        print('checking dt')
        self.left_bundle_tag_detect()
        self.right_bundle_tag_detect()
        print('dt',self.dt)

        #goal_xpose, goal_ypose, goal_thetha = pose_update_staging(stage_1_dist) ##
        angle_diff = goal_thetha - self.current_theta

        angle_diff = angle_check(angle_diff, goal_thetha)
        print('Turning bot until to goal orientation until tag detected')

        ##The following while loop turn the bot to goal orientation until april tag is detected as which point it moves to the bot in a manner to orientation it towards the next docking station distance goal
        
        stage_angular_speed = self.pid(self.pid_staging_tag_detect_gain, angle_diff, self.pid_staging_tag_detect_output_limits)
        
        while True:
            
            self.speed.linear.x = 0.0
            self.speed.angular.z = stage_angular_speed
            #print("x_pose",x_pose)
            #print("angle_diff",angle_diff)
            self.left_bundle_tag_detect()
            self.right_bundle_tag_detect()
            
            if self.dt_left > 0 and self.dt_right > 0:

                print('dt',self.dt)
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                time.sleep(1)
                print("tag detected , turning towards Stage-3 offset")
                
                goal_xpose, goal_ypose, goal_thetha = self.pose_update_staging(self.stage_3_dist)
                
                goal_angle = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose)
                angle_diff = goal_angle - self.current_theta
                angle_diff = angle_check(angle_diff, goal_angle)

                print("Stage 3 - current thetha",self.current_theta)
                print("Stage 3 - goal thetha", goal_angle)
                print("Stage 3 - angle diff", angle_diff)

                while angle_diff > self.stage_align_thresh or angle_diff < -self.stage_align_thresh:
                    
                    goal_angle = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose)
                    angle_diff = goal_angle - self.current_theta
                    angle_diff = angle_check(angle_diff, goal_angle)
                    stage_angular_speed = self.pid(self.pid_staging_alignment_gain, angle_diff, self.pid_staging_alignment_output_limit)
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = stage_angular_speed
                    self.pub.publish(self.speed)
                    self.odom_update_rate.sleep()

                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                break

            self.pub.publish(self.speed)
            self.tag_check_rate.sleep()

        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
        print('pointing towards goal pose - Stage 3')
        time.sleep(0.1)

        print('moving towards goal pose - Stage 3')

        while sqrt(pow((goal_xpose - self.current_xpose), 2) + pow((goal_ypose - self.current_ypose), 2)) > self.stage_3_radius_tolerance:

            angle_diff = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose) - self.current_theta
            euclidean_distance = sqrt(pow((goal_xpose - self.current_xpose), 2) + pow((goal_ypose - self.current_ypose), 2))
            stage_linear_speed = abs(self.pid(self.pid_staging_linear_gain, euclidean_distance, self.pid_staging_linear_output_limits))
            stage_angular_speed = self.pid(self.pid_staging_angular_gain, angle_diff, self.pid_staging_angular_output_limits)
            self.speed.linear.x = stage_linear_speed
            self.speed.angular.z = stage_angular_speed
            self.pub.publish(self.speed)
            self.odom_update_rate.sleep()

        print('At goal pose - Stage 3')
        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
    
        ##stage 3.1: Turning tp angle at distance to docking

        self.current_time = rospy.get_time()
        print('checking dt')
        x_pose_align, y_pose_align, z_pose = self.pose_update_alignment()
        print('dt',self.dt)
        angle_diff = goal_thetha - self.current_theta
        angle_diff = angle_check(angle_diff, goal_thetha)
        print('Turning bot until to goal orientation until tag detected')

        stage_angular_speed = self.pid(self.pid_staging_tag_detect_gain, angle_diff, self.pid_staging_tag_detect_output_limits)

        ##The following while loop turn the bot to goal orientation until april tag is detected as which point it moves to the bot in a manner to orientation it towards the next docking station distance goal

        while True:

            self.speed.linear.x = 0.0
            self.speed.angular.z = stage_angular_speed
            x_pose_align, y_pose_align, z_pose = self.pose_update_alignment()
            
            if self.dt > 0:
                print('dt',self.dt)
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                time.sleep(1)
                print("tag detected , turning towards Stage-3 offset")
                
                x_pose, y_pose, z_pose = self.pose_update_alignment()
                angle_diff = math.atan(y_pose/x_pose)

                while angle_diff > self.final_align_thresh or angle_diff < -self.final_align_thresh:

                    x_pose, y_pose, z_pose = self.pose_update_alignment()
                    angle_diff = math.atan(y_pose/x_pose)
                    stage_angular_speed = self.pid(self.pid_final_alignment_gain, angle_diff, self.pid_final_alignment_output_limit)
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = stage_angular_speed
                    #print("x_pose",x_pose)
                    #print("angle_diff",angle_diff)
                    self.pub.publish(self.speed)
                    self.odom_update_rate.sleep()

                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                break

            self.pub.publish(self.speed)
            self.tag_check_rate.sleep()

        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
        time.sleep(0.1)
        print("engaging")

        x_pose, y_pose, z_pose = self.pose_update_alignment()
        print(z_pose)

        while GPIO.input(self.charge_input_pin) == GPIO.LOW:
            x_pose, y_pose, z_pose = self.pose_update_alignment()

            #print(z_pose)
            if self.dt > 0:
                angle_diff = math.atan(y_pose/x_pose)
                stage_linear_speed = self.undock_linear_speed
                stage_angular_speed = self.pid(self.pid_staging_angular_gain, angle_diff, self.pid_staging_angular_output_limits)
                self.speed.linear.x = stage_linear_speed
                self.speed.angular.z = stage_angular_speed
                self.pub.publish(self.speed)
         
            
            else:
                stage_linear_speed = self.undock_linear_speed
                stage_angular_speed = 0
                self.speed.linear.x = stage_linear_speed
                self.speed.angular.z = stage_angular_speed
                self.pub.publish(self.speed)

            self.odom_update_rate.sleep()


        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
        print("Docking Complete")

        self.dock_feedback_pub.publish(self.dock_feedback)
        
    def docking_callback(self, data):
        self.docking_cmd = data.data

    def odometryCb(self, msg):
        self.current_xpose = msg.pose.pose.position.x
        self.current_ypose = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.current_theta = yaw
    
    def pose_update_staging(self, value): 
        while True: 

            pose = PoseStamped()
            pose.header = std_msgs.msg.Header()
            
            pose.header.frame_id = self.docking_bundle_name
            pose.pose = Pose()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = value
            pose.pose.orientation.x = 0.707
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = -0.707
            pose.pose.orientation.w = 0


            pose.header.stamp = rospy.Time(0)
            try:
                transform = self.tf_buffer.lookup_transform(self.odom_frame,
                                            # source frame:
                                            pose.header.frame_id,
                                            # get the tf at the time the last pose was valid
                                            pose.header.stamp,
                                            # wait for at most 1 second for transform, otherwise throw
                                            rospy.Duration(1))
                                        
                
                pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
                goal_xpose = pose_transformed.pose.position.x
                goal_ypose = pose_transformed.pose.position.y
            
                #current_time = pose_transformed.header.stamp.to_sec()
                #dt = pose_transformed.header.stamp - current_time
                #current_time = pose_transformed.header.stamp
                #print('time',current_time)

                orientation_q = pose_transformed.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (roll, pitch, goal_thetha) = euler_from_quaternion (orientation_list)
                print(goal_thetha)
                break

            except:
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                rospy.loginfo("NO TAG TF")
                continue
      
        return goal_xpose, goal_ypose, goal_thetha

    def pose_update_alignment(self):

        try:
            (trans,rot,) = self.t.lookupTransform(  self.base_link_frame, self.alignment_docking_tag_name, rospy.Time(0))
            lookup_time = self.t.getLatestCommonTime(  self.base_link_frame, self.alignment_docking_tag_name)
            #print("lastest", lookup_time)
            self.dt = lookup_time.to_sec() - self.current_time
            self.current_time = lookup_time.to_sec()
            x_pose_align = trans[0]
            y_pose_align = trans[1]
            z_pose = trans[2]
            
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.speed.linear.x = 0
            self.speed.angular.z = 0
            self.pub.publish(self.speed)
            rospy.loginfo("NO TAG TF")

        return x_pose_align, y_pose_align, z_pose

    def left_bundle_tag_detect(self):
        try:
            
            lookup_time = self.t.getLatestCommonTime( self.base_link_frame, self.left_bundle_tag)
            self.dt_left = lookup_time.to_sec() - self.current_time

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.speed.linear.x = 0
            self.speed.angular.z = 0
            self.pub.publish(self.speed)
            rospy.loginfo("NO TAG TF")

    def right_bundle_tag_detect(self):
        try:
            lookup_time = self.t.getLatestCommonTime( self.base_link_frame, self.right_bundle_tag)
            self.dt_right = lookup_time.to_sec() - self.current_time

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.speed.linear.x = 0
            self.speed.angular.z = 0
            self.pub.publish(self.speed)
            rospy.loginfo("NO TAG TF")

    def pose_update_final(self): #Stage 3 - Final movement 

        try:
            (trans,rot) = t.lookupTransform( self.docking_bundle_name, self.base_link_frame, rospy.Time(0))
            z_pose = trans[2]
            
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.speed.linear.x = 0
            self.speed.angular.z = 0
            self.pub.publish(self.speed)
            rospy.loginfo("NO TAG TF")

        return z_pose

    def run(self):
        rospy.init_node('docking_node')

        self.dock_feedback_pub = rospy.Publisher('/docking_result', Int32, queue_size=1)
        rospy.Subscriber('/docking_cmd',  Int32 , self.docking_callback, queue_size=1)
        self.pub =rospy.Publisher("/cmd_vel",Twist,  queue_size=10)
        rospy.Subscriber(self.odom_frame ,Odometry,self.odometryCb)
        self.t = tf.TransformListener()
        self.led_cmd_pub = rospy.Publisher("led_cmd", Int32, queue_size=10)
        self.odom_update_rate = rospy.Rate(self.odom_update_freq)
        self.tag_check_rate = rospy.Rate(self.tag_check_freq)
        rate = rospy.Rate(10)

        pose = PoseStamped()
        pose.header = std_msgs.msg.Header()

        self.speed = Twist()
        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        while not rospy.is_shutdown():
           
            if self.docking_cmd == 1:
                self.led_cmd_pub.publish(self.docking_led)
                self.dock()
                self.led_cmd_pub.publish(self.charging_led)

            if self.docking_cmd == 2:
                self.led_cmd_pub.publish(self.finnish_charging)
                time.sleep(3)
                self.led_cmd_pub.publish(self.docking_led)
                time.sleep(2)
                self.undock()
                time.sleep(2)
                self.led_cmd_pub.publish(self.driving_led)


            rate.sleep()
      

if __name__ == '__main__':
    
    d_interface = DockingInterface()

    try:
        
        d_interface.run()

    except rospy.ROSInterruptException:
        pass
    
 



 
