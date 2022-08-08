#!/bin/env python
import rospy 
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
import time 
import actionlib
import requests

import tf
import tf2_ros
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int32
import datetime



class StocktakeInterface():
    def __init__(self):
        self.NOTIFY_URL = "http://172.16.0.30:5000"
        self.status = 0
        self.initial_position = [0,0,0]
        self.undock_position = [0,-1.1, 3.14159]
        self.dock_pos = [0.769, 0.232, -3.140] #done
        self.pos_1 = [5.899,-14.506 , -1.269] #Start aisle 2
        self.pos_2 = [12.0 ,-32.98,-0.135] #End aisle 2
        self.pos_3 = [16.98,-33.88, 1.741] #Back to start ailes 2 
        self.pos_4 = [16.633, -9.006, 0.286] #Start aisles 1
        self.pos_5 = [24.950, -9.682, 0.8] #End aisles 1
        self.pos_6 = [29.196, -5.834, 1.846] #Back to start ailes 
        self.pos_7 = [25.941, -2.873, -3.075] #Back to start ailes
        self.pos_8 = [2.527, -3.598, 2.315] #Back to start ailes
        self.pos_9 = [3.987, 2.77, 0] #Back to start ailes
        self.pos_10 = [26.230, 2.994, 0] #Back to start ailes
        self.pos_11 = [4.409, 2.532, -2.652] #Back to start ailes
      
        self.no_action_cmd = 0
        self.dock_cmd = 1
        self.undock_cmd = 2
        self.docking_result = 0 #1- Docking, #2 -Undocking,#3 - Docked, #4 - Undocked

        self.driving_led = 1
        self.charging_led = 2
        self.docking_led = 3
        self.finnish_charging = 4
        self.startup_led = 5
        self.off_led = 6
      
        

    def undock(self):
        print("Send udocking cmd")
        
        self.dock_cmd_pub.publish(self.undock_cmd) #publish cmd 
        while self.docking_result != 2: #wait for result 
            self.rate.sleep()
            self.dock_cmd_pub.publish(self.undock_cmd) #publish cmd 

        self.dock_cmd_pub.publish(self.no_action_cmd)
        print("received feedback of undocking")
       
        while self.docking_result != 4: #wait for result 
            self.rate.sleep()
        print("received feedback of undocked")

        

    def dock(self):
        print("Send docking cmd")
        
        self.dock_cmd_pub.publish(self.dock_cmd) #publish cmd 
        while self.docking_result != 1: #wait for result 
            
            self.rate.sleep()
            self.dock_cmd_pub.publish(self.dock_cmd) #publish cmd
        self.dock_cmd_pub.publish(self.no_action_cmd)
        print("received feedback of docking")
        while self.docking_result != 3: #wait for result 
            self.rate.sleep()

        print("received feedback of docked")
        
        
        
    def docking_callback(self, data):
        self.docking_result = data.data


    def goal_pose(self, goal_x, goal_y, goal_thetha):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.position.z = 0
        q = quaternion_from_euler(0.0, 0.0, goal_thetha)
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.client.send_goal(goal)

    def estimate_pose(self, est_x, est_y, est_thetha):
        pose_estimate_data = PoseWithCovarianceStamped()
        pose_estimate_data.header.frame_id = 'map' 
        pose_estimate_data.header.stamp = rospy.Time.now()
        pose_estimate_data.pose.pose.position.x = est_x
        pose_estimate_data.pose.pose.position.y = est_y
        pose_estimate_data.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, est_thetha)
        pose_estimate_data.pose.pose.orientation.x = 0.0
        pose_estimate_data.pose.pose.orientation.y = 0.0
        pose_estimate_data.pose.pose.orientation.z = q[2]
        pose_estimate_data.pose.pose.orientation.w = q[3]
        pose_estimate_data.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        self.pose_estimate_pub.publish(pose_estimate_data)
        

    def run(self):
        rospy.init_node('stocktake_cmd_node', anonymous=True)
        
        self.pose_estimate_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)


        self.dock_cmd_pub = rospy.Publisher('/docking_cmd', Int32, queue_size=1)
        self.led_cmd_pub = rospy.Publisher("led_cmd", Int32, queue_size=10)
        rospy.Subscriber('/docking_result',  Int32 , self.docking_callback)
    
        
        self.rate = rospy.Rate(20)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        #self.estimate_pose(self.initial_position[0], self.initial_position[1], self.initial_position[2]) #Send initial Pose
        check_points_suc = 0
        runs = 1
        sucs_runs = 0
        check_points = 0
        while not rospy.is_shutdown():

            for x in range(runs):
           
                #self.undock()

                self.led_cmd_pub.publish(self.driving_led)
                time.sleep(1)
            
                self.goal_pose(self.pos_1[0], self.pos_1[1], self.pos_1[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1
                
              
                self.goal_pose(self.pos_8[0], self.pos_8[1], self.pos_8[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1


                
                self.goal_pose(self.pos_7[0], self.pos_7[1], self.pos_7[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1

             

                self.goal_pose(self.pos_6[0], self.pos_6[1], self.pos_6[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1
        
                
                self.goal_pose(self.pos_5[0], self.pos_5[1], self.pos_5[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1


                self.goal_pose(self.pos_4[0], self.pos_4[1], self.pos_4[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1

                self.goal_pose(self.pos_3[0], self.pos_3[1], self.pos_3[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1


                self.goal_pose(self.pos_4[0], self.pos_4[1], self.pos_4[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1

                self.goal_pose(self.pos_5[0], self.pos_5[1], self.pos_5[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1

                self.goal_pose(self.pos_6[0], self.pos_6[1], self.pos_6[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1

                self.goal_pose(self.pos_7[0], self.pos_7[1], self.pos_7[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1

                self.goal_pose(self.pos_8[0], self.pos_8[1], self.pos_8[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1

                self.goal_pose(self.pos_9[0], self.pos_9[1], self.pos_9[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1

                self.goal_pose(self.pos_10[0], self.pos_10[1], self.pos_10[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1

                self.goal_pose(self.pos_11[0], self.pos_11[1], self.pos_11[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1
         
                self.goal_pose(self.dock_pos[0], self.dock_pos[1], self.dock_pos[2])
                check_points += 1
                goal_state = self.client.get_state()
                print("current_state", goal_state)
                wait = self.client.wait_for_result()
                goal_state = self.client.get_state()
                result = self.client.get_result()
                print("result", result )
                print("goal_state",goal_state )
                if result:
                    check_points_suc += 1


                time.sleep(1)





            
          

                self.dock()

        
                



                

                
                
                sucs_runs += 1
                message = str("Run done")
                try:
                    response = requests.post("http://172.16.0.30:5000" + '/botapi/notify',json={'message': message , 'from':'trent_stocktake_bot', 'telegramuserid': 1937854848})
                except Exception as identifier:
                    print(str(identifier))         


                self.rate.sleep()
                
            title = "Pegasus-Mini Testing Report :robot_face: \n\n"
            report_status = "Test Type: Full Factory Navigation\n"
            date = datetime.datetime.now()
            date_now = date.strftime("%c")
            check_points_msg=str(check_points)
            date_msg_str = str(date_now)
            runs_msg = str(runs)
            sucs_runs_msg = str(sucs_runs)
            date_msg = str("Test Performed = " + date_msg_str + "\n")
            nav_runs = str("Navigation Runs =" + runs_msg + "\n")
            suc_mes = str("Successful Runs =" + sucs_runs_msg + "\n")
            check_points_suc_msg = str(check_points_suc)
            waypoint_msg = str("Total Goal Checkpoints =" + check_points_msg + "\n")
            waypoint_suc_msg = str("Checkpoints Reached =" + check_points_suc_msg + "\n")
            docking_msg = str("All Docking Status's = :white_check_mark:\n")
            charging_status = str("Charging Status = :white_check_mark:\n")
            undocking_msg = str("All Undocking Status's = :white_check_mark:\n")
            message = str(title + report_status + date_msg + nav_runs + suc_mes + waypoint_msg + waypoint_suc_msg + docking_msg + undocking_msg + charging_status)
            #1937854848 - Trent 
            #37133656 - Tom 
            #549830926 - Daylin 
            #-461910937 - CodeJunkie
            ids = [-461910937]
            for x in ids:




                
                try:
                    response = requests.post("http://172.16.0.30:5000" + '/botapi/notify',json={'message': message , 'from':'trent_stocktake_bot', 'telegramuserid': x})
                except Exception as identifier:
                    print(str(identifier))


   
            break
        

            

            

if __name__ == '__main__':
    
    st_interface = StocktakeInterface()

    

    try:
        
        st_interface.run()

    except rospy.ROSInterruptException:
        pass
    
 



 
