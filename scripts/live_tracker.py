#!/usr/bin/env python


import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, TimeReference
import traceback
import os
import sys
import requests
import time

velocity_topic = "vel"
acceleration_topic = "accel"
relative_leadervel_topic = "rel_vel"
relative_distance_topic = "lead_dist"
acc_status_topic = None

velocity = 0.0
acceleration = 0.0
relative_leadervel = 0.0
relative_distance = 0.0
acc_status = 0
can_update_time = None

gps_fix_topic = "gps_fix"
gps_fix_time_reference_topic = "gps_fix_time"
gpstime = None
systime = None
latitude = None
longitude = None
altitude = None
status = None
gps_update_time = None

vin=None
vin_path="/etc/libpanda.d/vin"
web_path="http://ransom.isis.vanderbilt.edu/LIVE_VIEWER_SITE/rest.php"

def readAllFile(path):
        assert os.path.exists(path), path + " file does not exist apparently!"
        file = open(path, mode='r')
        res = file.read()
        file.close()
        return res

def getVIN():
        return readAllFile(vin_path)

def velocity_callback(data):
	global velocity
	global can_update_time
	velocity = data.data
	can_update_time = rospy.Time.now()

def acceleration_callback(data):
	global acceleration
	global can_update_time
	acceleration = data.data
	can_update_time = rospy.Time.now()

def relative_leadervel_callback(data):
        global relative_leadervel
	global can_update_time
        relative_leadervel = data.data
        can_update_time = rospy.Time.now()

def relative_distance_callback(data):
        global relative_distance
	global can_update_time
        relative_distance = data.data
        can_update_time = rospy.Time.now()

def acc_status_callback(data):
        pass

def gps_fix_callback(data):
	global systime
	global latitude
	global longitude
	global altitude
	global status
	global gps_update_time

	latitude = data.latitude
	longitude = data.longitude
	altitude = data.altitude
	status = data.status.status
	systime = rospy.Time.now()
	gps_update_time = systime

def gps_fix_time_reference_callback(data):
	global gpstime
	gpstime = data.time_ref

def getGPSResultStr():
	global gpstime
	global systime
	global latitude
	global longitude
	global altitude
	global status
        return ','.join([gpstime, systime, latitude, longitude, altitude, status])

def getCANResultStr(can_file):
	global velocity
	global acceleration
	global relative_leadervel
	global relative_distance
	global acc_status
        return ','.join([velocity, acceleration, relative_leadervel, relative_distance, acc_status])

class LiveTracker:
	def __init__(self):
		global vin
		rospy.init_node('LiveTracker', anonymous=True)
		rospy.Subscriber(velocity_topic, Float64, velocity_callback)
		rospy.Subscriber(acceleration_topic, Float64, acceleration_callback)
		rospy.Subscriber(relative_leadervel_topic, Float64, relative_leadervel_callback)
		rospy.Subscriber(relative_distance_topic, Float64, relative_distance_callback)
		rospy.Subscriber(gps_fix_topic, NavSatFix, gps_fix_callback)
		rospy.Subscriber(gps_fix_time_reference_topic, TimeReference, gps_fix_time_reference_callback)
		self.rate = rospy.Rate(1)
		while vin is None:
			try:
				vin = getVIN()
			except Exception as e:
				print(e)
				traceback.print_exc()
				print("Cannot get VIN at this time!")
				time.sleep(1.0) #Wait 1 second hard-coded between checking for the VIN file

	def loop(self):
		while not rospy.is_shutdown():
			try:
				global vin
				current_time = rospy.Time.now()
				assert gps_update_time is not None, "GPS data has never been received!"
				assert can_update_time is not None, "CAN data has never been received!"
				assert abs((current_time - gps_update_time).to_sec()) < 30, "GPS data more than 30 seconds old!"
				assert abs((current_time - can_update_time).to_sec()) < 30, "CAN data more than 30 seconds old!"
				gps = getGPSResultStr()
				can = getCANResultStr()
				data_str = "?circles," + vin + "," + gps + "," + can
				get_str = WEB_PATH + data_str
				print(get_str)
				print(requests.get(get_str))
			except Exception as e:
				print(e)
				traceback.print_exc()
				print("Not uploading any data at this time.")
			self.rate.sleep()

if __name__ == '__main__':
    try:
        tracker = LiveTracker()
        tracker.loop()
    except Exception as e:
        print(e)
        traceback.print_exc()
        print("An exception occurred")
