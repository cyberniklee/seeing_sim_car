#!/usr/bin/env python

import serial
import RPi.GPIO as GPIO
import threading
import roslib
import rospy
from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
import tf
from math import sin, cos, pi


#front_left_roll_forward_pin = 31
#front_left_roll_backward_pin = 29
#front_right_roll_forward_pin = 13
#front_right_roll_backward_pin = 11
rear_left_roll_forward_pin = 31
rear_left_roll_backward_pin = 29
rear_right_roll_forward_pin = 18
rear_right_roll_backward_pin = 22
#front_left_roll_speed = 0
#front_right_roll_speed = 0
rear_left_roll_speed = 0
rear_right_roll_speed = 0

right_count = 0
left_count = 0

last_time = 0

last_vel_time = 0

wheel_track = 0.265
wheel_diameter = 0.068
control_rate = 10
pid_rate = 8.8

vel_timeout = 1.0

x = 0
y = 0
th = 0
v_des_left = 0
v_des_right = 0
v_left = 0
v_right = 0

def sendOdom(left_count_arg, right_count_arg):
        global right_count
        global left_count
        global last_time
        global x
        global y
        global th
        global v_des_left
        global v_des_right
        global v_left
        global v_right
        #global front_left_roll_forward_pin
        #global front_left_roll_backward_pin
        #global front_right_roll_forward_pin
        #global front_right_roll_backward_pin
        global rear_left_roll_forward_pin
        global rear_left_roll_backward_pin
        global rear_right_roll_forward_pin
        global rear_right_roll_backward_pin
        
        #global front_left_roll_speed
        #global front_right_roll_speed
        global rear_left_roll_speed
        global rear_right_roll_speed

        present_time = rospy.Time.now()
        if present_time > (last_time - period):
                dt = present_time - last_time
                last_time = present_time
                dt = dt.to_sec()

                if v_right < 0:
                        right_count_arg = -right_count_arg

                if v_left < 0:
                        left_count_arg = -left_count_arg

                dleft = left_count_arg/ticks_per_meter
                dright = right_count_arg/ticks_per_meter
                dxy_ave = (dleft + dright)/2.0
                dth = (dright - dleft)/wheel_track

                vxy = dxy_ave/dt
                vth = dth/dt

                if dxy_ave != 0:
                        dx = cos(dth)*dxy_ave
                        dy = -sin(dth)*dxy_ave
                        x += (cos(th)*dx - sin(th)*dy)
                        y += (sin(th)*dx + cos(th)*dy)

                if dth != 0:
                        th += dth

                quaternion = Quaternion()
                quaternion.x = 0.0
                quaternion.y = 0.0
                quaternion.z = sin(th/2.0)
                quaternion.w = cos(th/2.0)

                odom_tf.sendTransform((x, y, 0), (quaternion.x, quaternion.y, quaternion.z, quaternion.w), rospy.Time.now(), "base_link", "odom")

                odom = Odometry()
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                odom.header.stamp = present_time
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.position.z = 0
                odom.pose.pose.orientation = quaternion
                odom.twist.twist.linear.x = vxy
                odom.twist.twist.linear.y = 0
                odom.twist.twist.angular.z = vth

                odom_pub.publish(odom)

                if present_time > (last_vel_time + rospy.Duration(vel_timeout)):
                        v_des_left = 0
                        v_des_right = 0

                if v_left < v_des_left:
                        v_left += max_acc
                        if v_left > v_des_left:
                                v_left = v_des_left
                else:
                        v_left -= max_acc
                        if v_left < v_des_left:
                                v_left = v_des_left

                if v_right < v_des_right:
                        v_right += max_acc
                        if v_right > v_des_right:
                                v_right = v_des_right
                else:
                        v_right -= max_acc
                        if v_right < v_des_right:
                                v_right = v_des_right

                if abs(v_left) > 255 or abs(v_right) > 255:
                        print "Error speed >>>"
                        print v_left
                        print v_right
                        print "<<<"
                        return
                        
                #front_left_roll_speed = int(abs(v_left))
                #front_right_roll_speed = int(abs(v_right))
                rear_left_roll_speed = int(abs(v_left))
                rear_right_roll_speed = int(abs(v_right))
        
                if v_right > 0:
                        #GPIO.output(front_right_roll_forward_pin, True)
                        #GPIO.output(front_right_roll_backward_pin, False)
                        GPIO.output(rear_right_roll_forward_pin, True)
                        GPIO.output(rear_right_roll_backward_pin, False)
                elif v_right < 0:
                        #GPIO.output(front_right_roll_forward_pin, False)
                        #GPIO.output(front_right_roll_backward_pin, True)
                        GPIO.output(rear_right_roll_forward_pin, False)
                        GPIO.output(rear_right_roll_backward_pin, True)
                else:
                        #GPIO.output(front_right_roll_forward_pin, False)
                        #GPIO.output(front_right_roll_backward_pin, False)
                        GPIO.output(rear_right_roll_forward_pin, False)
                        GPIO.output(rear_right_roll_backward_pin, False)
                        
                if v_left > 0:
                        #GPIO.output(front_left_roll_forward_pin, True)
                        #GPIO.output(front_left_roll_backward_pin, False)
                        GPIO.output(rear_left_roll_forward_pin, True)
                        GPIO.output(rear_left_roll_backward_pin, False)
                elif v_left < 0:
                        #GPIO.output(front_left_roll_forward_pin, False)
                        #GPIO.output(front_left_roll_backward_pin, True)
                        GPIO.output(rear_left_roll_forward_pin, False)
                        GPIO.output(rear_left_roll_backward_pin, True)
                else:
                        #GPIO.output(front_left_roll_forward_pin, False)
                        #GPIO.output(front_left_roll_backward_pin, False)
                        GPIO.output(rear_left_roll_forward_pin, False)
                        GPIO.output(rear_left_roll_backward_pin, False)

                

def timerEvent():
        #global front_left_roll_speed
        #global front_right_roll_speed
        global rear_left_roll_speed
        global rear_right_roll_speed
        global right_count
        global left_count
        
        ser.write(chr(0xAA))
        ser.write(chr(0x00))
        ser.write(chr(0x00))
        ser.write(chr(rear_left_roll_speed))
        ser.write(chr(rear_right_roll_speed))
        if ser.inWaiting() == 2:
                left_count = ord(ser.read())
                right_count = ord(ser.read())
                print("Right count:%d, Left count:%d; Right speed:%d, Left speed:%d."%(right_count, left_count, rear_right_roll_speed, rear_left_roll_speed))
        else:
                ser.read(ser.inWaiting())
        sendOdom(left_count, right_count)
        global timer
        timer = threading.Timer(1.0/control_rate, timerEvent)
        timer.start()

def callback(msg):
#	rospy.loginfo("Received a /cmd_vel message!")
#       rospy.loginfo("Linear:[%f,%f,%f]", msg.linear.x, msg.linear.y, msg.linear.z)
#	rospy.loginfo("Angular:[%f,%f,%f]", msg.angular.x, msg.angular.y, msg.angular.z)
        #global front_left_roll_forward_pin
        #global front_left_roll_backward_pin
        #global front_right_roll_forward_pin
        #global front_right_roll_backward_pin
        global rear_left_roll_forward_pin
        global rear_left_roll_backward_pin
        global rear_right_roll_forward_pin
        global rear_right_roll_backward_pin
        
        #global front_left_roll_speed
        #global front_right_roll_speed
        global rear_left_roll_speed
        global rear_right_roll_speed

        global v_des_left
        global v_des_right

        global last_vel_time

        last_vel_time = rospy.Time.now()

        v = msg.linear.x
        yaw = msg.angular.z

        if v == 0:
                right_v = yaw*wheel_track*16/2.0
                left_v = -right_v
        elif yaw == 0:
                left_v = right_v = v
        else:
                left_v = v - yaw*wheel_track*16/2.0
                right_v = v + yaw*wheel_track*16/2.0
        
        v_des_left = int(left_v*ticks_per_meter/pid_rate)
        v_des_right = int(right_v*ticks_per_meter/pid_rate)
	
if __name__ == '__main__':
        print("Seeing smartcar is starting...")

        ser = serial.Serial('/dev/ttyACM0', 115200);
        if ser.isOpen() is False:
                print("Open serial port fail!");
                quit();

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        #GPIO.setup(front_left_roll_forward_pin, GPIO.OUT)
        #GPIO.setup(front_left_roll_backward_pin, GPIO.OUT)
        #GPIO.setup(front_right_roll_forward_pin, GPIO.OUT)
        #GPIO.setup(front_right_roll_backward_pin, GPIO.OUT)
        GPIO.setup(rear_left_roll_forward_pin, GPIO.OUT)
        GPIO.setup(rear_left_roll_backward_pin, GPIO.OUT)
        GPIO.setup(rear_right_roll_forward_pin, GPIO.OUT)
        GPIO.setup(rear_right_roll_backward_pin, GPIO.OUT)
        
        rospy.init_node("cmd_vel_listener")
        rospy.Subscriber("/cmd_vel", Twist, callback)
        odom_pub = rospy.Publisher('odom', Odometry, queue_size=5)
        odom_tf = tf.TransformBroadcaster()
        print("...ok")

        ticks_per_meter = 62*16/(wheel_diameter*pi)
        max_acc = ticks_per_meter/(control_rate*control_rate)

        last_time = rospy.Time.now()
        last_vel_time = last_time
        period = rospy.Duration(1.0/control_rate)

        timer = threading.Timer(1.0/control_rate, timerEvent)
        timer.start()
