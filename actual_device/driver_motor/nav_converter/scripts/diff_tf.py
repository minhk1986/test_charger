#!/usr/bin/env python

import rospy
import roslib
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32
from nav_converter.msg import speed_wheel

class DiffTf:
    def __init__(self):
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
        
        #### parameters #######
        self.rate = rospy.get_param('~rate', 10.0)  # the rate at which to publish the transform
        # self.ticks_meter = float(rospy.get_param('ticks_meter', 30))  # The number of wheel encoder ticks per meter of travel
        # self.ticks_meter = float(rospy.get_param('ticks_meter', 1686))  # The number of wheel encoder ticks per meter of travel
        self.ticks_meter = float(rospy.get_param('ticks_meter', 56.2))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.5)) # The wheel base width in meters (The distance between the wheels)
        
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        
        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()

        driver_left  = rospy.get_param(self.nodename + "/driver_name_left")
        driver_right = rospy.get_param(self.nodename + "/driver_name_right")

        # subscriptions
        rospy.Subscriber(driver_left + "/encoders", speed_wheel, self.lwheelCallback)
        rospy.loginfo("diff_tf.py-58- Subscriber topic /" + driver_left + "/encoders")
        rospy.Subscriber(driver_right + "/encoders", speed_wheel, self.rwheelCallback)
        rospy.loginfo("diff_tf.py-60- Subscriber topic /" + driver_right + "/encoders")

        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
    def update(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            
            # calculate odometry
            if (self.enc_left == None) and (self.enc_right == None):
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                rospy.loginfo("diff_tf.py-85- left: " + str(self.left))
                rospy.loginfo("diff_tf.py-86- enc_left: " + str(self.enc_left))
                rospy.loginfo("diff_tf.py-87- d_left: " + str(d_left))
                d_right = (self.right - self.enc_right) / self.ticks_meter
                rospy.loginfo("diff_tf.py-89- right: " + str(self.right))
                rospy.loginfo("diff_tf.py-90- enc_right: " + str(self.enc_right))
                rospy.loginfo("diff_tf.py-91- d_right: " + str(d_right))
            self.enc_left = self.left
            self.enc_right = self.right
           
            # distance traveled is the average of the two wheels 
            d = (d_left + d_right) / 2
            # this approximation works (in radians) for small angles
            th = (d_right - d_left) / self.base_width
            # calculate velocities
            self.dx = d/elapsed
            self.dr = th/elapsed
           
            if(d != 0):
                # calculate distance traveled in x and y
                x = cos(th) * d
                y = -sin(th) * d
                # calculate the final position of the robot
                self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
                self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
            if(th != 0):
                self.th = self.th + th
                
            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th/2)
            quaternion.w = cos(self.th/2)
            self.odomBroadcaster.sendTransform((self.x, self.y, 0), 
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(), self.base_frame_id, self.odom_frame_id)

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)
            
    def lwheelCallback(self, msg):
        # rospy.loginfo("diff_tf.py-135- lwheelCallback()")
        enc = msg.encoder
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lencoder = enc
        
    def rwheelCallback(self, msg):
        # rospy.loginfo("diff_tf.py-145- rwheelCallback()")
        enc = msg.encoder
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_lbase_frame_idow_wrap):
            self.rmult = self.rmult - 1
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc
        

if __name__ == '__main__':
    try:
        print("diff_tf.py-153- main()")
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass