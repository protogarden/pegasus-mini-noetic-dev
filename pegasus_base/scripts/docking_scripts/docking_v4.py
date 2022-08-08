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
from std_msgs.msg import Int32


class DockingInterface():
    def __init__(self):
        self.pid_staging_alignment = PID(0.6, 0, 0, setpoint=0) #PID that controls staging angular velocity 
        self.pid_staging_alignment.output_limits = (-0.2, 0.2)
        self.pid_staging_tag_detect = PID(0.8, 0, 0, setpoint=0) #PID that controls staging angular velocity 
        self.pid_staging_tag_detect.output_limits = (-0.3, 0.3)
        self.pid_staging_angular = PID(0.3, 0, 0, setpoint=0) #PID that controls staging angular velocity 
        self.pid_staging_angular.output_limits = (-0.1, 0.1)
        self.pid_staging_linear = PID(0.5, 0, 0, setpoint=0) #PID that controls staging angular velocity 
        self.pid_staging_linear.output_limits = (-0.08, 0.08)
        self.pid_staging_alignment.auto_mode = False
        self.pid_staging_tag_detect.auto_mode = False
        self.pid_staging_angular.auto_mode = False
        self.pid_staging_linear.auto_mode = False
        

        self.docking_tag_name = 'DOCKING_TAGS' #bases_link currently for testing
        self.left_tag = 'LEFT-TAG'
        self.right_tag = 'RIGHT-TAG'
        self.alignment_docking_tag_name = 'TAG'
        self.odom_frame = 'odom'
        self.base_link_frame = 'base_link'
        self.camera_link = 'base_link'

        self.stage_1_dist = 0.8
        self.stage_2_dist = 0.6
        self.stage_3_dist = 0.5

        self.final_dist = 0.07
        self.angle_thresh = 1 #initial staging thresh

        self.angle_thresh_2 = 0.01 #second staging thresh
        self.angle_thresh_3 = 0.01
        self.angle_thresh_4 = 0.01
        self.angle_thresh_2_align = 0.05
        self.angle_thresh_3_align = 0.008

        self.current_xpose = 0
        self.current_ypose = 0
        self.current_theta = 0
        '''
        self.goal_xpose = 0
        self.goal_ypose = 0
        self.goal_thetha = 0
        '''

        self.dt = 0
        self.dt_left = 0
        self.dt_right = 0

        self.stage_1_radius_tolerance = 0.03
        self.stage_2_radius_tolerance = 0.03
        self.stage_3_radius_tolerance = 0.025

        self.docking_cmd = 0

        self.docking_feedback = 1
        self.dock_feedback = 3

        self.undocking_feedback = 2
        self.undock_feedback = 4
      
        

    def undock(self):

        
        self.dock_feedback_pub.publish(self.undocking_feedback)

        stage_linear_speed = -0.02
        stage_angular_speed = 0
        self.speed.linear.x = stage_linear_speed
        self.speed.angular.z = stage_angular_speed
        self.pub.publish(self.speed)
        time.sleep(4)

        stage_linear_speed = 0
        stage_angular_speed = 0
        self.speed.linear.x = stage_linear_speed
        self.speed.angular.z = stage_angular_speed
        self.pub.publish(self.speed)

        
        self.dock_feedback_pub.publish(self.undock_feedback)


 

    def dock(self):



        
        self.dock_feedback_pub.publish(self.docking_feedback)
        

        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)

        ##stage 1
        ##stage 1.1: Turning to angle at distance to docking
        
        goal_xpose, goal_ypose, goal_thetha = self.pose_update_staging(self.stage_1_dist)
        angle_diff = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose) - self.current_theta
        self.pid_staging_alignment.auto_mode = True #Turn PID ON
        print('Turning towards goal pose - Stage 1')
        while angle_diff > self.angle_thresh_2 or angle_diff < -self.angle_thresh_2:
            #goal_xpose, goal_ypose, goal_thetha = pose_update_staging(stage_1_dist)
            angle_diff = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose) - self.current_theta
            stage_angular_speed = self.pid_staging_alignment(angle_diff)
            self.speed.linear.x = 0.0
            self.speed.angular.z = -stage_angular_speed
            self.pub.publish(self.speed)
        self.pid_staging_alignment.auto_mode = False #Turn PID OFF
        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
        time.sleep(0.2)
        print('pointing towards goal pose - Stage 1')
        
        ##stage 1.2: Going to point from docking 

        print('moving towards goal pose - Stage 1')
        self.pid_staging_angular.auto_mode = True
        self.pid_staging_linear.auto_mode = True
        while sqrt(pow((goal_xpose - self.current_xpose), 2) + pow((goal_ypose - self.current_ypose), 2)) > self.stage_1_radius_tolerance:
            angle_diff = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose) - self.current_theta
            stage_linear_speed = abs(self.pid_staging_linear(sqrt(pow((goal_xpose - self.current_xpose), 2) + pow((goal_ypose - self.current_ypose), 2))))
            stage_angular_speed = -self.pid_staging_angular(angle_diff)
            self.speed.linear.x = stage_linear_speed
            self.speed.angular.z = stage_angular_speed
            self.pub.publish(self.speed)
        self.pid_staging_angular.auto_mode = False
        self.pid_staging_linear.auto_mode = False
    
        print('reached goal pose - Stage 1')
        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
        


        #stage 2
        ##stage 2.1: Turning tp angle at distance to docking
        self.current_time = rospy.get_time()
        print('checking dt')
        self.left_tag_detect()
        self.right_tag_detect()
        print('dt',self.dt)

        #goal_xpose, goal_ypose, goal_thetha = pose_update_staging(stage_1_dist) ##
        angle_diff = goal_thetha - self.current_theta
        print('Turning bot until to goal orientation until left/right tag detected')

        ##The following while loop turn the bot to goal orientation until april tag is detected as which point it moves to the bot in a manner to orientation it towards the next docking station distance goal
        self.pid_staging_tag_detect.auto_mode = True
        while angle_diff > 0 or angle_diff < 0:
            angle_diff = goal_thetha - self.current_theta
            stage_angular_speed = self.pid_staging_tag_detect(angle_diff)
            self.speed.linear.x = 0.0
            self.speed.angular.z = -stage_angular_speed
            

            self.left_tag_detect()
            self.right_tag_detect()
            

            if self.dt_left > 0 and self.dt_right > 0:
                self.pid_staging_tag_detect.auto_mode = False
                print('dt_left',self.dt_left)
                print('dt_right',self.dt_right)
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                time.sleep(1)

                print("tag detected , turning towards Stage-2 offset")
                goal_xpose, goal_ypose, goal_thetha = self.pose_update_staging(self.stage_2_dist)
                angle_diff = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose) - self.current_theta
                
                self.pid_staging_alignment.auto_mode = True
                while angle_diff > self.angle_thresh_2 or angle_diff < -self.angle_thresh_2:
                    angle_diff = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose) - self.current_theta
                    stage_angular_speed = self.pid_staging_alignment(angle_diff)
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = -stage_angular_speed
                    self.pub.publish(self.speed)
                self.pid_staging_alignment.auto_mode = False
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                break


            self.pub.publish(self.speed)

        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
        time.sleep(0.1)

    

        print('pointing towards goal pose - Stage 2')
        print('moving towards goal pose - Stage 2')

        self.pid_staging_angular.auto_mode = True
        self.pid_staging_linear.auto_mode = True
        while sqrt(pow((goal_xpose - self.current_xpose), 2) + pow((goal_ypose - self.current_ypose), 2)) > self.stage_2_radius_tolerance:
            angle_diff = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose) - self.current_theta
            stage_linear_speed = abs(self.pid_staging_linear(sqrt(pow((goal_xpose - self.current_xpose), 2) + pow((goal_ypose - self.current_ypose), 2))))
            stage_angular_speed = -self.pid_staging_angular(angle_diff)
            self.speed.linear.x = stage_linear_speed
            self.speed.angular.z = stage_angular_speed
            self.pub.publish(self.speed)
        self.pid_staging_angular.auto_mode = False
        self.pid_staging_linear.auto_mode = False
        print('At goal pose - Stage 2')
        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
        


        ##stage 2.1: Turning tp angle at distance to docking
        self.current_time = rospy.get_time()
        print('checking dt')
        self.left_tag_detect()
        self.right_tag_detect()
        print('dt',self.dt)

        #goal_xpose, goal_ypose, goal_thetha = pose_update_staging(stage_1_dist) ##
        angle_diff = goal_thetha - self.current_theta
        print('Turning bot until to goal orientation until tag detected')

        ##The following while loop turn the bot to goal orientation until april tag is detected as which point it moves to the bot in a manner to orientation it towards the next docking station distance goal
        self.pid_staging_tag_detect.auto_mode = True
        while angle_diff > 0 or angle_diff < 0:
            angle_diff = goal_thetha - self.current_theta
            stage_angular_speed = self.pid_staging_tag_detect(angle_diff)
            self.speed.linear.x = 0.0
            self.speed.angular.z = -stage_angular_speed
            #print("x_pose",x_pose)
            #print("angle_diff",angle_diff)
            self.pub.publish(self.speed)

            self.left_tag_detect()
            self.right_tag_detect()
            

            if self.dt_left > 0 and self.dt_right > 0:

                self.pid_staging_tag_detect.auto_mode = False

                print('dt',self.dt)

                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                time.sleep(1)
                print("tag detected , turning towards Stage-3 offset")
                
                goal_xpose, goal_ypose, goal_thetha = self.pose_update_staging(self.stage_3_dist)
                angle_diff = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose) - self.current_theta
                self.pid_staging_alignment.auto_mode = True
                while angle_diff > self.angle_thresh_3 or angle_diff < -self.angle_thresh_3:
                    
                    angle_diff = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose) - self.current_theta
                    stage_angular_speed = self.pid_staging_alignment(angle_diff)
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = -stage_angular_speed
                    self.pub.publish(self.speed)
                self.pid_staging_alignment.auto_mode = False
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                break


            self.pub.publish(self.speed)

        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
        time.sleep(0.1)

    

        print('pointing towards goal pose - Stage 3')
        print('moving towards goal pose - Stage 3')
        self.pid_staging_angular.auto_mode = True
        self.pid_staging_linear.auto_mode = True
        while sqrt(pow((goal_xpose - self.current_xpose), 2) + pow((goal_ypose - self.current_ypose), 2)) > self.stage_3_radius_tolerance:

            angle_diff = atan2(goal_ypose - self.current_ypose, goal_xpose - self.current_xpose) - self.current_theta
            stage_linear_speed = abs(self.pid_staging_linear(sqrt(pow((goal_xpose - self.current_xpose), 2) + pow((goal_ypose - self.current_ypose), 2))))
            stage_angular_speed = -self.pid_staging_angular(angle_diff)
            self.speed.linear.x = stage_linear_speed
            self.speed.angular.z = stage_angular_speed
            self.pub.publish(self.speed)

        self.pid_staging_angular.auto_mode = False
        self.pid_staging_linear.auto_mode = False
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
        print('Turning bot until to goal orientation until tag detected')
        self.pid_staging_tag_detect.auto_mode = True
        ##The following while loop turn the bot to goal orientation until april tag is detected as which point it moves to the bot in a manner to orientation it towards the next docking station distance goal
        while angle_diff > 0 or angle_diff < 0:

            angle_diff = goal_thetha - self.current_theta
            stage_angular_speed = self.pid_staging_tag_detect(angle_diff)
            self.speed.linear.x = 0.0
            self.speed.angular.z = -stage_angular_speed
           
            x_pose_align, y_pose_align, z_pose = self.pose_update_alignment()
            

            if self.t > 0:
                self.pid_staging_tag_detect.auto_mode = False
                print('dt',self.dt)

                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                time.sleep(1)
                print("tag detected , turning towards Stage-3 offset")
                
                x_pose, y_pose, z_pose = self.pose_update_alignment()
                angle_diff = math.atan(y_pose/x_pose)
                self.pid_staging_alignment.auto_mode = True
                while angle_diff > self.angle_thresh_3_align or angle_diff < -self.angle_thresh_3_align:

                    x_pose, y_pose, z_pose = self.pose_update_alignment()
                    angle_diff = math.atan(y_pose/x_pose)
                    stage_angular_speed = -self.pid_staging_alignment(angle_diff)
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = stage_angular_speed
                    #print("x_pose",x_pose)
                    #print("angle_diff",angle_diff)


                    self.pub.publish(self.speed)
                self.pid_staging_alignment.auto_mode = False
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                break


            self.pub.publish(speed)

        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
      
        time.sleep(0.1)
        x_pose, y_pose, z_pose = self.pose_update_alignment()
        print(z_pose)


        self.pid_staging_angular.auto_mode = True
        self.pid_staging_linear.auto_mode = True
        while z_pose > 0.096:

            #print(z_pose)
            
            x_pose, y_pose, z_pose = self.pose_update_alignment()
            angle_diff = math.atan(y_pose/x_pose)
            stage_linear_speed = 0.02
            stage_angular_speed = -self.pid_staging_angular(angle_diff)
            self.speed.linear.x = stage_linear_speed
            self.speed.angular.z = stage_angular_speed
            self.pub.publish(self.speed)

        self.pid_staging_angular.auto_mode = False
        self.pid_staging_linear.auto_mode = False


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
            
            pose.header.frame_id = self.docking_tag_name
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
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.pub.publish(self.speed)
                rospy.loginfo("NO TAG TF")
                continue

    
      
        return goal_xpose, goal_ypose, goal_thetha

    def pose_update_alignment(self):

        try:
            (trans,rot,) = self.t.lookupTransform( self.camera_link, self.alignment_docking_tag_name, rospy.Time(0))
            lookup_time = self.t.getLatestCommonTime( self.camera_link, self.alignment_docking_tag_name)
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

    def left_tag_detect(self):
        try:
            
            lookup_time = self.t.getLatestCommonTime( self.base_link_frame, self.left_tag)
            self.dt_left = lookup_time.to_sec() - self.current_time

   
            
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.speed.linear.x = 0
            self.speed.angular.z = 0
            self.pub.publish(self.speed)
            rospy.loginfo("NO TAG TF")

  

    def right_tag_detect(self):
        try:
            lookup_time = self.t.getLatestCommonTime( self.base_link_frame, self.right_tag)
            self.dt_right = lookup_time.to_sec() - self.current_time

            
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.speed.linear.x = 0
            self.speed.angular.z = 0
            self.pub.publish(self.speed)
            rospy.loginfo("NO TAG TF")






    def pose_update_final(self): #Stage 3 - Final movement 

        try:
            (trans,rot) = t.lookupTransform( self.docking_tag_name, self.base_link_frame, rospy.Time(0))
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
        rospy.Subscriber('/docking_cmd',  Int32 , self.docking_callback)
        self.pub =rospy.Publisher("/cmd_vel",Twist,  queue_size=1)
        rospy.Subscriber('odom',Odometry,self.odometryCb)
        self.t = tf.TransformListener()
        rate = rospy.Rate(2)

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
                self.dock()

            if self.docking_cmd == 2:
                self.undock()


            rate.sleep()
      

            



            

            

if __name__ == '__main__':
    
    d_interface = DockingInterface()

    try:
        
        d_interface.run()

    except rospy.ROSInterruptException:
        pass
    
 



 
