#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, UInt32, Int64MultiArray, Int32
from sensor_msgs.msg import Joy

from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
import tf
from math import pi, cos, sin
import math

class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width):
        

        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = rospy.Time.now()
        self.start_up = 1

   
    @staticmethod
    def normalize_angle(angle):
        
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time
       
       

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        if self.start_up == 1:
            self.last_enc_right = enc_right
            self.last_enc_left = enc_left
            self.start_up = 0
        vel_x, vel_theta = self.update(enc_left, enc_right)
        self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)
        

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()

        br = tf.TransformBroadcaster()
        br.sendTransform((cur_x, cur_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, cur_theta),
                         current_time,
                         "base_link",
                         "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance
      
        self.odom_pub.publish(odom)

class PegasusBase:
    def __init__(self):
        
        self.encoder = None
        self.MAX_SPEED = 2.5
        self.TICKS_PER_METER = 8148.71
        self.BASE_WIDTH = 0.25363
        self.left_speed = 0
        self.right_speed = 0


    def cb_encoder(self,data):
        
        enc1 = data.data[0]
        enc2 = data.data[1]
        
        

        if self.encoder != None:
            self.encoder.update_publish(enc1, enc2)

    def cb_cmdvel(self,twist):
        
        
        #rospy.loginfo("cmdvel")
        #last_set_speed_time = rospy.get_rostime()

        linear_x = twist.linear.x
        if linear_x > self.MAX_SPEED:
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        vr = linear_x + twist.angular.z * self.BASE_WIDTH / 2.0  # m/s
        vl = linear_x - twist.angular.z * self.BASE_WIDTH / 2.0

        vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
        vl_ticks = int(vl * self.TICKS_PER_METER)

        #rospy.logdebug("vr_ticks:%d vl_ticks: %d", vr_ticks, vl_ticks)

        if vr_ticks == 0 and vl_ticks == 0:
            self.left_speed = 0
            self.right_speed = 0
        else:
            self.left_speed = vr_ticks 
            self.right_speed = vl_ticks


    def run(self):
        
        rospy.init_node('pegasus_base_node', anonymous=True)
        
        #rospy.Subscriber("encoder", Int64MultiArray, self.cb_encoder)
        
        rospy.Subscriber("cmd_vel", Twist, self.cb_cmdvel)
        

        ticks_per_meter = rospy.get_param('~ticks_per_meter', 8130.0)
        base_width = rospy.get_param('~base_width', 0.25868)
        rate = rospy.Rate(10) # 10hz    

        self.encoder = EncoderOdom(ticks_per_meter, base_width)    
        

        wheelSpeedPublisher = rospy.Publisher('wheelspeed', Int64MultiArray, queue_size=2)
        

        speed_data = Int64MultiArray()

        while not rospy.is_shutdown():
            
            speed_data.data = [self.left_speed, self.right_speed]
            wheelSpeedPublisher.publish(speed_data)        

            rate.sleep()


if __name__ == '__main__':
    
    pegasus_base = PegasusBase()
    try:
        
        pegasus_base.run()
    except rospy.ROSInterruptException:
        pass
    
