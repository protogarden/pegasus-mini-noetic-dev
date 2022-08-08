#!/usr/bin/env python
import rospy
from std_msgs.msg import String, UInt32, Int64MultiArray, Float32MultiArray,Int32, Float32,String


class SafetyInterface():
    def __init__(self):
        self.left_motor_current = 0
        self.right_motor_current = 0
        self.accum_ah_left_motor = 0
        self.accum_ah_right_motor = 0
        self.accum_time = 0

        self.check_time_frame = 2

    def cb_motorcurrentCallBack(self, data):

        self.left_motor_current = data.data[0]
        self.right_motor_current = data.data[1]

    def run(self):

        rospy.init_node('safety_feature_node', anonymous=True)
        self.last_motor_current_time = rospy.Time.now()
        critical_error_pub = rospy.Publisher('/critical_error', String, queue_size=10)
        rospy.Subscriber('/motor_currents', Int64MultiArray, self.cb_motorcurrentCallBack)
        rate = rospy.Rate(50) # 10hz 


        while not rospy.is_shutdown():

            
            motor_current_time = rospy.Time.now()
            d_time = (motor_current_time - self.last_motor_current_time).to_sec()
            
            self.accum_ah_left_motor += self.left_motor_current*d_time
            self.accum_ah_right_motor += self.right_motor_current*d_time
            self.accum_time += d_time
            if self.accum_time > self.check_time_frame:
                print(self.accum_ah_left_motor/self.check_time_frame)


                print(self.accum_ah_right_motor/self.check_time_frame)
                if (self.accum_ah_left_motor/self.check_time_frame > 210) or (self.accum_ah_right_motor/self.check_time_frame) > 200:
                    print("motor_warning")
                    if self.accum_ah_left_motor > 210:
                        critical_error = "Left Motor exceed max mA/s"
                        rospy.loginfo("Critical Error: Left Motor Max Motor mA/s Exceeded")
                    if self.accum_ah_right_motor > 200:
                        critical_error = "Left Motor Exceed Max mA/s"
                        rospy.loginfo("Critical Error: Right Motor Max Motor mA/s Exceeded")
                    critical_error_pub.publish(critical_error)

                self.accum_time = 0
                self.accum_ah_left_motor = 0
                self.accum_ah_right_motor = 0
            
       

            self.last_motor_current_time = motor_current_time
            rate.sleep()

if __name__ == '__main__':
    
    safety_interface = SafetyInterface()

    try:
        
        safety_interface.run()

    except rospy.ROSInterruptException:
        pass
    
    #hw_interface.stop()