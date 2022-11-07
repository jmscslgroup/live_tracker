#!/usr/bin/env python


import rospy
from std_msgs.msg import Bool, Float64, Int16, String
from sensor_msgs.msg import NavSatFix, TimeReference
import traceback
import os
import sys
import requests
import time

position_topic = "xpos"
velocity_topic = "vel"
acceleration_topic = "accel"
relative_leadervel_topic = "rel_vel"
relative_distance_topic = "lead_dist"
left_relvel_topic = "left_relvel"
left_yaw_topic = "left_yaw"
right_relvel_topic = "right_relvel"
right_yaw_topic = "right_yaw"
acc_speed_topic = "acc/set_speed"
acc_status_topic = "acc/cruise_state"

position = 0.0
last_positions = [0.0] * 10
velocity = 0.0
acceleration = 0.0
relative_leadervel = 0.0
relative_distance = 0.0
left_relvel = 0.0
left_yaw = 90.0
right_relvel = 0.0
right_yaw = 90.0
acc_speed = 20
acc_status = None
can_update_time = None

gps_fix_topic = "gps_fix"
gps_fix_time_reference_topic = "gps_fix_time"
gpstime = None
systime = None
latitude = None
longitude = None
status = None
gps_update_time = None

vin=None
vin_path="/etc/libpanda.d/vin"
web_path="http://ransom.isis.vanderbilt.edu/inrix/api/veh_ping.php"

def readAllFile(path):
    assert os.path.exists(path), path + " file does not exist apparently!"
    file = open(path, mode='r')
    res = file.read()
    file.close()
    return res

def getVIN():
    return readAllFile(vin_path)

def position_callback(data):
    global position
    global can_update_time
    position = data.data
    can_update_time = rospy.Time.now()

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

# def left_relvel_callback(data):
#     global left_relvel
#     global can_update_time
#     left_relvel = data.data
#     can_update_time = rospy.Time.now()

# def left_yaw_callback(data):
#     global left_yaw
#     global can_update_time
#     left_yaw = data.data
#     can_update_time = rospy.Time.now()

# def right_relvel_callback(data):
#     global right_relvel
#     global can_update_time
#     right_relvel = data.data
#     can_update_time = rospy.Time.now()

# def right_yaw_callback(data):
#     global right_yaw
#     global can_update_time
#     right_yaw = data.data
#     can_update_time = rospy.Time.now()
    
def acc_speed_callback(data):
    global acc_speed
    global can_update_time
    acc_speed = data.data
    can_update_time = rospy.Time.now()

def acc_status_callback(data):
    global acc_status
    global can_update_time
    acc_status = data.data
    can_update_time = rospy.Time.now()

def gps_fix_callback(data):
    global systime
    global latitude
    global longitude
    global status
    global gps_update_time

    latitude = data.latitude
    longitude = data.longitude
    status = data.status
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
    global status
    global position
    return '&'.join(['gpstime={}'.format(gpstime.to_sec()),
                     'systime={}'.format(systime.to_sec()),
                     'latitude={}'.format(latitude),
                     'longitude={}'.format(longitude),
                     'status={}'.format(status),
                     'position={}'.format(position)])

def getCANResultStr():
    global velocity
    global acceleration
    global relative_leadervel
    global relative_distance
    global acc_status
    acc_status_int = {'enabled': 1}.get(acc_status, 0)
    return '&'.join(['velocity={}'.format(velocity),
                     'acceleration={}'.format(acceleration),
                     'relative_leadervel={}'.format(relative_leadervel),
                     'relative_distance={}'.format(relative_distance),
                     'left_relvel={}'.format(left_relvel),
                     'left_yaw={}'.format(left_yaw),
                     'right_relvel={}'.format(right_relvel),
                     'right_yaw={}'.format(right_yaw),
                     'acc_speed_setting={}'.format(acc_speed),
                     'acc_status={}'.format(acc_status_int)])

def is_wb(positions):
    """Check that positions increase monotonically."""
    return all([x1 - x2 >= 0 for (x1, x2) in zip(positions[:-1], positions[1:])])


class LiveTracker:
    def __init__(self):
        global vin
        rospy.init_node('LiveTracker', anonymous=True)
        rospy.Subscriber(position_topic, Float64, position_callback)
        rospy.Subscriber(velocity_topic, Float64, velocity_callback)
        rospy.Subscriber(acceleration_topic, Float64, acceleration_callback)
        rospy.Subscriber(relative_leadervel_topic, Float64, relative_leadervel_callback)
        rospy.Subscriber(relative_distance_topic, Float64, relative_distance_callback)
#         rospy.Subscriber(left_relvel_topic, Float64, left_relvel_callback)
#         rospy.Subscriber(left_yaw_topic, Float64, left_yaw_callback)
#         rospy.Subscriber(right_relvel_topic, Float64, right_relvel_callback)
#         rospy.Subscriber(right_yaw_topic, Float64, right_yaw_callback)
        rospy.Subscriber(acc_speed_topic, Int16, acc_speed_callback)
        rospy.Subscriber(acc_status_topic, String, acc_status_callback)
        rospy.Subscriber(gps_fix_topic, NavSatFix, gps_fix_callback)
        rospy.Subscriber(gps_fix_time_reference_topic, TimeReference, gps_fix_time_reference_callback)

        self.is_wb_pub = rospy.Publisher('/is_westbound', Bool, queue_size=10)
        self.rate = rospy.Rate(1)
        while vin is None:
            try:
                vin = getVIN()
            except Exception as e:
                print(e)
                traceback.print_exc()
                print("Cannot get VIN at this time!")
                time.sleep(1.0)  # Wait 1 second hard-coded between checking for the VIN file

    def loop(self):
        while not rospy.is_shutdown():
            try:
                global vin
                current_time = rospy.Time.now()
                assert gps_update_time is not None, "GPS data has never been received!"
                assert can_update_time is not None, "CAN data has never been received!"
                assert abs((current_time - gps_update_time).to_sec()) < 30, "GPS data more than 30 seconds old!"
                assert abs((current_time - can_update_time).to_sec()) < 30, "CAN data more than 30 seconds old!"
                last_positions.pop()
                last_positions.insert(0, position)
                wb = is_wb(last_positions)
                self.is_wb_pub.publish(wb)
                gps = getGPSResultStr()
                can = getCANResultStr()
                data_str = "?vin=" + vin + "&" + gps + "&" + can + "&is_wb={}".format(int(wb))
                get_str = web_path + data_str
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
