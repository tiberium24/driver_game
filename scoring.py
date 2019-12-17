#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

class Driver:
    
    def __init__(self):
        self.score = 0
        self.speed = None
        self.speed1= 0
        self.speed2= 0
        self.odom = None
        self.time11= 0
        self.time12= 0
        self.time13= 0
        self.time14= 0
        self.condition2_tailgate= False
        self.kinematic_timer_started= False 
        self.speed_timer_started= False
        self.tailgate_timer_started= False       
        self.temp_distance_travelled= 0
        self.positive_score_timer_started= False
        #Limits
        self.speed_limit= 20
        self.speed_limit_time= 1
        self.kinematic_limit_time= 1
        self.following_dist_limit= 1
        self.long_accel= 0.35
        self.lat_accel= 0.05
        self.yaw= 0.10471976 

    ###Aggressive
    def imu_callback(self, data):
        self.imu = data
        if (data.linear_acceleration.x>self.long_accel or data.linear_acceleration.y>self.lat_accel or data.angular_velocity.z>self.yaw):
            condition= True
        else:
            condition= False
        if condition==True and self.kinematic_timer_started==False:
            self.time11= time.time()
            self.kinematic_timer_started= True
        elif condition==True and self.kinematic_timer_started==True:
            time2= time.time()
            total= time2-self.time11
            if total>=self.kinematic_limit_time:
                self.time11= time.time()
                self.score -= 1
        elif condition==False:
            self.kinematic_timer_started= False
        self.compute_callback()
    
    ###Tailgate
    def radar_callback(self, data):
        distance= data.markers[0].pose.position.x
        following_dist= distance/self.speed
        #setting the condition
        if (following_dist<self.following_dist_limit and self.speed>20):
            condition= True
        else:
            condition= False
        #    
        if condition==True and self.tailgate_timer_started==False:
            self.time12= time.time()
            self.tailgate_timer_started= True
        elif condition==True and self.tailgate_timer_started==True:
            time2= time.time()
            total= time2-self.time12
            if total>=5 and self.condition2_tailgate==False:
                self.time12= time.time()
                self.score -= 1
                self.condition2_tailgate= True
            elif total>=1 and self.condition2_tailgate== True:
                self.time12= time.time()
                self.score -= 1
        elif condition==False:
            self.tailgate_timer_started= False
            self.condition2_tailgate= False

    ###Speed
    def odom_callback(self, data):
        self.speed = data.data
        if (self.speed>self.speed_limit):
            condition= True
        else:
            condition= False
         
        if condition==True and self.speed_timer_started==False:
            self.time13= time.time()
            self.speed_timer_started= True
        elif condition==True and self.speed_timer_started==True:
            time2= time.time()
            total= time2-self.time13
            if total>=self.speed_limit_time:
                self.time13= time.time()
                self.score -= 1
        elif condition==False:
            self.speed_timer_started= False

    def positive_score_callback(self):
        if self.speed is not None: 
           if self.positive_score_timer_started==False:
               self.time14= time.time()
               self.speed1= self.speed
               self.positive_score_timer_started= True
           else:
               time2= time.time()
               total= time2-self.time1
               self.speed2= self.speed
               speed= 0.5*(self.speed1+self.speed2)
               self.temp_distance_travelled += speed*total
               if self.temp_distance_travelled>100:
                   self.score +=1
                   self.temp_distance_travelled=0
               self.positive_score_timer_started= False
        

if __name__ == '__main__':
    rospy.init_node('listener')
    driver = Driver()

    rospy.Subscriber("/imu/imu", Imu, driver.imu_callback)
    rospy.Subscriber("/gps/speed_float", Float64, driver.odom_callback)
    rospy.Subscriber("/sms_radar_markers", MarkerArray, driver.radar_callback)
    #rospy.spin()
    while not rospy.is_shutdown():
       if driver.speed_timer_started==True:
           print('Speeding')
           driver.temp_distance_travelled= 0
           driver.positive_score_timer_started= False
       if driver.tailgate_timer_started==True:
           print('Tailgating')
           driver.temp_distance_travelled= 0
           driver.positive_score_timer_started= False
       if driver.kinematic_timer_started==True:
           print('Aggressive')
           driver.temp_distance_travelled= 0
           driver.positive_score_timer_started= False
       print('Driver score: ' + str(driver.score))
       rospy.sleep(0.1)
