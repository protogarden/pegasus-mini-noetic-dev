import rospy 
import tf
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import std_msgs.msg
from nav_msgs.msg  import Odometry
from geometry_msgs.msg import Point, Twist, PoseStamped, Pose
from tf.transformations import euler_from_quaternion 
from math import atan2
import math
import time
from simple_pid import PID

pid_staging_1_angular = PID(0.2, 0, 0, setpoint=0) #PID that controls staging angular velocity 
pid_staging_1_angular.output_limits = (-0.1, 0.1)

pid_staging_1_linear = PID(0.6, 0, 0.1, setpoint=0) #PID that controls staging angular velocity 
pid_staging_1_linear.output_limits = (-0.08, 0.08)

pid_staging_2_angular = PID(0.3, 0, 0, setpoint=0) #PID that controls staging angular velocity 
pid_staging_2_angular.output_limits = (-0.1, 0.1)



docking_tag_name = 'TAG' #bases_link currently for testing
base_link_frame = 'base_link'
stage_dist = 1
final_dist = 0.5
angle_thresh = 0.1 #initial staging thresh

angle_thresh_2 = 0.01 #second staging thresh

x_pose = 0
y_pose = 0

def pose_update_staging(): #stage-1 Staging 
    while True: 


        pose.header.stamp = rospy.Time(0)
        try:
            transform = tf_buffer.lookup_transform(base_link_frame,
                                        # source frame:
                                        pose.header.frame_id,
                                        # get the tf at the time the pose was valids
                                        pose.header.stamp,
                                        # wait for at most 1 second for transform, otherwise throw
                                        rospy.Duration(1))

            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)

            x_pose = pose_transformed.pose.position.x
            print("xpos",x_pose)
            y_pose = pose_transformed.pose.position.y
            break

            
        except:
            speed.linear.x = 0
            speed.angular.z = 0
            pub.publish(speed)
            rospy.loginfo("NO TAG TF")
            continue

    print('here')
  
    return x_pose, y_pose

def pose_update_alignment(): #Stage 2 - alignment

    try:
        (trans,rot) = t.lookupTransform( base_link_frame, docking_tag_name, rospy.Time(0))
        #print("Initial Trans",trans )
        #print("Rotation")
        #print(rot)
        x_pose = trans[0]
        y_pose = trans[1]
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)
        rospy.loginfo("NO TAG TF")

    return x_pose, y_pose

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
    rospy.init_node('docking_node')
    pub =rospy.Publisher("/cmd_vel",Twist,  queue_size=1)
    t = tf.TransformListener()
    rate = rospy.Rate(100)
    speed = Twist()

    pose = PoseStamped()
    pose.header = std_msgs.msg.Header()
    
    pose.header.frame_id = docking_tag_name
    pose.pose = Pose()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = stage_dist
    pose.pose.orientation.w = 1


    speed.linear.x = 0
    speed.angular.z = 0
    pub.publish(speed)
    

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1))  # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    # send PoseStamped
    
    while not rospy.is_shutdown():
     


        
        x_pose, y_pose = pose_update_staging()

        '''

        while x_pose > 0.001:

            
            angle_diff = math.atan(y_pose/x_pose)
            stage_linear_speed = abs(pid_staging_1_linear(x_pose))
            stage_angular_speed = -pid_staging_1_angular(angle_diff)
   

         
            speed.linear.x = stage_linear_speed
            speed.angular.z = stage_angular_speed
            print("stage_linear_speed",stage_linear_speed )
            print("stage_angular_speed",stage_angular_speed )
      

            pub.publish(speed)
            x_pose, y_pose = pose_update_staging()
        '''


        x_pose, y_pose = pose_update_staging()
        angle_diff = math.atan(y_pose/x_pose)

        speed.linear.x = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
        print("staging complete")
        time.sleep(0.1)

        
        while angle_diff > angle_thresh_2 or angle_diff < -angle_thresh_2:

            x_pose, y_pose = pose_update_alignment()
            angle_diff = math.atan(y_pose/x_pose)
            stage_angular_speed = -pid_staging_2_angular(angle_diff)
            speed.linear.x = 0.0
            speed.angular.z = stage_angular_speed
            #print("x_pose",x_pose)
            #print("angle_diff",angle_diff)


            pub.publish(speed)

        print("staging complete")
        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)
        '''
        z_pose = pose_update_final()

        while z_pose > final_dist:

            z_pose = pose_update_final()
            speed.linear.x = 0.1
            speed.angular.z = 0.0
            print("Final Straight")
            pub.publish(speed)

        print("docking complete")
        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)

        time.sleep(2)

        z_pose = pose_update_final()

        while z_pose < stage_dist:

            z_pose = pose_update_final()
            speed.linear.x = -0.1
            speed.angular.z = 0.0
            print("Final Straight")
            pub.publish(speed)
      
        print("undocking complete")
        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)


        rate.sleep()
        #break
        '''

