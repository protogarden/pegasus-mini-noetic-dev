import rospy 
import tf
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import std_msgs.msg
from nav_msgs.msg  import Odometry
from geometry_msgs.msg import Point, Twist, PoseStamped, Pose
from tf.transformations import euler_from_quaternion 
from math import atan2, pow, sqrt
import math
import time
from simple_pid import PID

pid_staging_1_angular = PID(0.6, 0, 0, setpoint=0) #PID that controls staging angular velocity 
pid_staging_1_angular.output_limits = (-0.5, 0.5)

pid_staging_1_linear = PID(0.4, 0, 0, setpoint=0) #PID that controls staging angular velocity 
pid_staging_1_linear.output_limits = (-0.08, 0.08)

pid_staging_2_angular = PID(0.4, 0, 0, setpoint=0) #PID that controls staging angular velocity 
pid_staging_2_angular.output_limits = (-0.3, 0.3)



docking_tag_name = 'DOCKING_TAGS' #bases_link currently for testing
alignment_docking_tag_name = 'TAG'
odom_frame = 'odom'
base_link_frame = 'base_link'
stage_1_dist = 0.8
stage_2_dist = 0.6
stage_3_dist = 0.4
probe_offset = 0.0001
final_dist = 0.5
angle_thresh = 1 #initial staging thresh

angle_thresh_2 = 0.04 #second staging thresh
angle_thresh_3 = 0.03
angle_thresh_4 = 0.01
angle_thresh_2_align = 0.05

current_xpose = 0
current_ypose = 0

current_theta = 0
goal_xpose = 0
goal_ypose = 0
goal_thetha = 0
dt = 0






def odometryCb(msg):
    global current_xpose 
    global current_ypose 
    global current_theta
    current_xpose = msg.pose.pose.position.x
    current_ypose = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    current_theta = yaw
   



def pose_update_staging(value): #stage-1 Staging 
    while True: 

        pose = PoseStamped()
        pose.header = std_msgs.msg.Header()
        
        pose.header.frame_id = docking_tag_name
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
            transform = tf_buffer.lookup_transform(odom_frame,
                                        # source frame:
                                        pose.header.frame_id,
                                        # get the tf at the time the pose was valids
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
            speed.linear.x = 0
            speed.angular.z = 0
            pub.publish(speed)
            rospy.loginfo("NO TAG TF")
            continue

   
  
    return goal_xpose, goal_ypose, goal_thetha

def pose_update_alignment():
    global current_time
    global dt
     #Stage 2 - alignment


    try:
        (trans,rot,) = t.lookupTransform( base_link_frame, alignment_docking_tag_name, rospy.Time(0))
        lookup_time = t.getLatestCommonTime( base_link_frame, alignment_docking_tag_name)
        #print("lastest", lookup_time)
        dt = lookup_time.to_sec() - current_time
        current_time = lookup_time.to_sec()
        #print(dt)
        #print("Initial Trans",trans )
        #print("Rotation")
        #print(rot)
        x_pose_align = trans[0]
        y_pose_align = trans[1]
        
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)
        rospy.loginfo("NO TAG TF")

    return x_pose_align, y_pose_align

def pose_update_final(): #Stage 3 - Final movement 

    try:
        (trans,rot) = t.lookupTransform( docking_tag_name, base_link_frame, rospy.Time(0))
        #print("Initial Trans",trans )
        #print("Rotation")
        #print(rot)
        z_pose = trans[2]
        
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)
        rospy.loginfo("NO TAG TF")

    return z_pose


if __name__ == '__main__':
    global current_time
    
    rospy.init_node('docking_node')
    pub =rospy.Publisher("/cmd_vel",Twist,  queue_size=1)
    rospy.Subscriber('odom',Odometry,odometryCb)
    t = tf.TransformListener()
    rate = rospy.Rate(2)
    speed = Twist()

    pose = PoseStamped()
    pose.header = std_msgs.msg.Header()

    


    speed.linear.x = 0
    speed.angular.z = 0
    pub.publish(speed)

    

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1))  # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    # send PoseStamped
    
    while not rospy.is_shutdown():

        ##stage 1

        ##stage 1.1: Turning tp angle at distance to docking

        goal_xpose, goal_ypose, goal_thetha = pose_update_staging(stage_1_dist)
        angle_diff = atan2(goal_ypose - current_ypose, goal_xpose - current_xpose) - current_theta
        print('Turning towards goal pose - Stage 1')
        while angle_diff > angle_thresh_2 or angle_diff < -angle_thresh_2:
            #goal_xpose, goal_ypose, goal_thetha = pose_update_staging(stage_1_dist)
            angle_diff = atan2(goal_ypose - current_ypose, goal_xpose - current_xpose) - current_theta
            
            

            #print('goal_thetha', goal_thetha)
            #print('currrent_thetha', current_theta)
            #print('angle diff', angle_diff)



           
            stage_angular_speed = pid_staging_2_angular(angle_diff)
            speed.linear.x = 0.0
            speed.angular.z = -stage_angular_speed
            pub.publish(speed)
        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)

        print('pointing towards goal pose - Stage 1')
        #print('goal thetha', goal_thetha)
        
        ##stage 1.2: Going to point from docking 

        print('moving towards goal pose - Stage 1')

        while sqrt(pow((goal_xpose - current_xpose), 2) + pow((goal_ypose - current_ypose), 2)) > 0.03:

            

            

            angle_diff = atan2(goal_ypose - current_ypose, goal_xpose - current_xpose) - current_theta

            #print("distance", sqrt(pow((goal_xpose - current_xpose), 2) + pow((goal_ypose - current_ypose), 2)))
            #print("angle", angle_diff)
           
            


            stage_linear_speed = abs(pid_staging_1_linear(sqrt(pow((goal_xpose - current_xpose), 2) + pow((goal_ypose - current_ypose), 2))))
            stage_angular_speed = -pid_staging_1_angular(angle_diff)
            speed.linear.x = stage_linear_speed
            speed.angular.z = stage_angular_speed
            pub.publish(speed)
        print('reached goal pose - Stage 1')
        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)
        time.sleep(0.01)


        #stage 2
        ##stage 2.1: Turning tp angle at distance to docking
        current_time = rospy.get_time()
        print('checking dt')
        x_pose_align, y_pose_align = pose_update_alignment()
        print('dt',dt)

        #goal_xpose, goal_ypose, goal_thetha = pose_update_staging(stage_1_dist) ##
        angle_diff = goal_thetha - current_theta
        print('Turning bot until to goal orientation until tag detected')

        ##The following while loop turn the bot to goal orientation until april tag is detected as which point it moves to the bot in a manner to orientation it towards the next docking station distance goal
        while angle_diff > angle_thresh_2 or angle_diff < -angle_thresh_2:

            
            

            #print('goal_thetha', goal_thetha)
            #print('currrent_thetha', current_theta)
            #print('angle diff', angle_diff)
            
         


            angle_diff = goal_thetha - current_theta
            stage_angular_speed = pid_staging_2_angular(angle_diff)
            speed.linear.x = 0.0
            speed.angular.z = -stage_angular_speed
            


            #print("x_pose",x_pose)
            #print("angle_diff",angle_diff)
            

            x_pose_align, y_pose_align = pose_update_alignment()
            

            if dt > 0:

                print('dt',dt)

                speed.linear.x = 0
                speed.angular.z = 0
                pub.publish(speed)

                print("tag detected , turning towards Stage-2 offset")
                
                goal_xpose, goal_ypose, goal_thetha = pose_update_staging(stage_2_dist)
                angle_diff = atan2(goal_ypose - current_ypose, goal_xpose - current_xpose) - current_theta
                time.sleep(0.5)
                
                while angle_diff > angle_thresh_2 or angle_diff < -angle_thresh_2:
                    
                    angle_diff = atan2(goal_ypose - current_ypose, goal_xpose - current_xpose) - current_theta
                    
                    

                    #print('goal_thetha', goal_thetha)
                    #print('currrent_thetha', current_theta)
                    #print('angle diff', angle_diff)



                
                    stage_angular_speed = pid_staging_2_angular(angle_diff)
                    speed.linear.x = 0.0
                    speed.angular.z = -stage_angular_speed
                    pub.publish(speed)
                speed.linear.x = 0
                speed.angular.z = 0
                pub.publish(speed)
                break


            pub.publish(speed)

        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)

    

        print('pointing towards goal pose - Stage 2')
        print('moving towards goal pose - Stage 2')


        while sqrt(pow((goal_xpose - current_xpose), 2) + pow((goal_ypose - current_ypose), 2)) > 0.02:

            

            angle_diff = atan2(goal_ypose - current_ypose, goal_xpose - current_xpose) - current_theta

            #print("distance", sqrt(pow((goal_xpose - current_xpose), 2) + pow((goal_ypose - current_ypose), 2)))
            #print("angle", angle_diff)
            
            


            stage_linear_speed = abs(pid_staging_1_linear(sqrt(pow((goal_xpose - current_xpose), 2) + pow((goal_ypose - current_ypose), 2))))
            stage_angular_speed = -pid_staging_1_angular(angle_diff)
            speed.linear.x = stage_linear_speed
            speed.angular.z = stage_angular_speed
            pub.publish(speed)
        print('At goal pose - Stage 2')
        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)


        ##stage 2.1: Turning tp angle at distance to docking
        current_time = rospy.get_time()
        print('checking dt')
        x_pose_align, y_pose_align = pose_update_alignment()
        print('dt',dt)

        #goal_xpose, goal_ypose, goal_thetha = pose_update_staging(stage_1_dist) ##
        angle_diff = goal_thetha - current_theta
        print('Turning bot until to goal orientation until tag detected')

        ##The following while loop turn the bot to goal orientation until april tag is detected as which point it moves to the bot in a manner to orientation it towards the next docking station distance goal
        while angle_diff > angle_thresh_2 or angle_diff < -angle_thresh_2:
            #print('goal_thetha', goal_thetha)
            #print('currrent_thetha', current_theta)
            #print('angle diff', angle_diff)
            angle_diff = goal_thetha - current_theta
            stage_angular_speed = pid_staging_2_angular(angle_diff)
            speed.linear.x = 0.0
            speed.angular.z = -stage_angular_speed
            #print("x_pose",x_pose)
            #print("angle_diff",angle_diff)
            pub.publish(speed)

            x_pose_align, y_pose_align = pose_update_alignment()
            

            if dt > 0:

                print('dt',dt)

                speed.linear.x = 0
                speed.angular.z = 0
                pub.publish(speed)
                time.sleep(0.01)
                print("tag detected , turning towards Stage-3 offset")
                
                goal_xpose, goal_ypose, goal_thetha = pose_update_staging(stage_3_dist)
                angle_diff = atan2(goal_ypose - current_ypose, goal_xpose - current_xpose) - current_theta
                
                while angle_diff > angle_thresh_3 or angle_diff < -angle_thresh_3:
                    
                    angle_diff = atan2(goal_ypose - current_ypose, goal_xpose - current_xpose) - current_theta
                    
                    

                    #print('goal_thetha', goal_thetha)
                    #print('currrent_thetha', current_theta)
                    #print('angle diff', angle_diff)



                
                    stage_angular_speed = pid_staging_2_angular(angle_diff)
                    speed.linear.x = 0.0
                    speed.angular.z = -stage_angular_speed
                    pub.publish(speed)
                speed.linear.x = 0
                speed.angular.z = 0
                pub.publish(speed)
                break


            pub.publish(speed)

        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)

    

        print('pointing towards goal pose - Stage 3')
        print('moving towards goal pose - Stage 3')

        while sqrt(pow((goal_xpose - current_xpose), 2) + pow((goal_ypose - current_ypose), 2)) > 0.02:

            

            angle_diff = atan2(goal_ypose - current_ypose, goal_xpose - current_xpose) - current_theta

            #print("distance", sqrt(pow((goal_xpose - current_xpose), 2) + pow((goal_ypose - current_ypose), 2)))
            #print("angle", angle_diff)
            
            


            stage_linear_speed = abs(pid_staging_1_linear(sqrt(pow((goal_xpose - current_xpose), 2) + pow((goal_ypose - current_ypose), 2))))
            stage_angular_speed = -pid_staging_1_angular(angle_diff)
            speed.linear.x = stage_linear_speed
            speed.angular.z = stage_angular_speed
            pub.publish(speed)
        print('At goal pose - Stage 3')
        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)
        time.sleep(0.01)

##stage 2.1: Turning tp angle at distance to docking
        current_time = rospy.get_time()
        print('checking dt')
        x_pose_align, y_pose_align = pose_update_alignment()
        print('dt',dt)
        #goal_xpose, goal_ypose, goal_thetha = pose_update_staging(probe_offset)
        #goal_xpose, goal_ypose, goal_thetha = pose_update_staging(stage_1_dist) ##
        angle_diff = goal_thetha - current_theta
        print('Turning bot until to goal orientation until tag detected')

        ##The following while loop turn the bot to goal orientation until april tag is detected as which point it moves to the bot in a manner to orientation it towards the next docking station distance goal
        while angle_diff > angle_thresh_2 or angle_diff < -angle_thresh_2:
            #print('goal_thetha', goal_thetha)
            #print('currrent_thetha', current_theta)
            #print('angle diff', angle_diff)
            angle_diff = goal_thetha - current_theta
            stage_angular_speed = pid_staging_2_angular(angle_diff)
            speed.linear.x = 0.0
            speed.angular.z = -stage_angular_speed
            pub.publish(speed)
            #print("x_pose",x_pose)
            #print("angle_diff",angle_diff)
            

            x_pose_align, y_pose_align = pose_update_alignment()
            

            if dt > 0:

                print('dt',dt)

                speed.linear.x = 0
                speed.angular.z = 0
                pub.publish(speed)
                time.sleep(0.01)
                print("tag detected , turning towards Stage-3 offset")
                
                goal_xpose, goal_ypose, goal_thetha = pose_update_staging(probe_offset)
                angle_diff = atan2(goal_ypose - current_ypose, goal_xpose - current_xpose) - current_theta
                
                while angle_diff > angle_thresh_4 or angle_diff < -angle_thresh_4:
                    
                    angle_diff = atan2(goal_ypose - current_ypose, goal_xpose - current_xpose) - current_theta
                    
                    

                    #print('goal_thetha', goal_thetha)
                    #print('currrent_thetha', current_theta)
                    #print('angle diff', angle_diff)



                
                    stage_angular_speed = pid_staging_2_angular(angle_diff)
                    speed.linear.x = 0.0
                    speed.angular.z = -stage_angular_speed
                    pub.publish(speed)
                speed.linear.x = 0
                speed.angular.z = 0
                pub.publish(speed)
                break


            pub.publish(speed)

        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)



        rate.sleep()
   
        break
        


