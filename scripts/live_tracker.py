#!/usr/bin/env python


import rospy
from std_msgs.msg import Float64, Int16, String
from sensor_msgs.msg import NavSatFix, TimeReference
import traceback
import os
import sys
import requests
import time
import bisect
import numpy as np

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
acc_status_topic = "acc/cruise_state_int"

position = 0.0
last_positions = [0.0] * 10
lat = 0.0
last_lats = [0.0] * 10
lon = 0.0
last_lons = [0.0] * 10
velocity = 0.0
acceleration = 0.0
relative_leadervel = 0.0
relative_distance = 0.0
left_relvel = 0.0
left_yaw = 90.0
right_relvel = 0.0
right_yaw = 90.0
acc_speed = 20
side_street_thresh = 0.0017995483133733433   # Around 200 meters, in lat/long degrees
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
    global status
    global position
    return '&'.join(['gpstime={}'.format(int(gpstime.to_sec() * 1000)),
                     'systime={}'.format(int(systime.to_sec() * 1000)),
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
    acc_status_int = {25: 1}.get(acc_status, 0)
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

points_east = np.array([
    [-86.724947, 36.115263],
    [-86.723923, 36.113178],
    [-86.723324, 36.112089],
    [-86.722862, 36.111298],
    [-86.722495, 36.1107],
    [-86.721975, 36.109827],
    [-86.721059, 36.108573],
    [-86.72047, 36.107855],
    [-86.720107, 36.10745],
    [-86.719805, 36.10713],
    [-86.719196, 36.106539],
    [-86.718267, 36.10573],
    [-86.717676, 36.105275],
    [-86.717187, 36.104926],
    [-86.716743, 36.104627],
    [-86.715996, 36.104156],
    [-86.715439, 36.10379],
    [-86.715049, 36.10352],
    [-86.714558, 36.103157],
    [-86.71396, 36.102676],
    [-86.713551, 36.102322],
    [-86.713327, 36.102114],
    [-86.71298, 36.101785],
    [-86.712538, 36.101339],
    [-86.712165, 36.100925],
    [-86.711899, 36.100616],
    [-86.711292, 36.099871],
    [-86.710323, 36.098678],
    [-86.7095, 36.097665],
    [-86.708427, 36.096351],
    [-86.704763, 36.091822],
    [-86.703679, 36.090415],
    [-86.701463, 36.087727],
    [-86.70027, 36.086125],
    [-86.699222, 36.084519],
    [-86.698461, 36.083289],
    [-86.696924, 36.080669],
    [-86.69237, 36.073348],
    [-86.691109, 36.071317],
    [-86.690461, 36.07042],
    [-86.689839, 36.069633],
    [-86.689672, 36.069464],
    [-86.689559, 36.06934],
    [-86.689177, 36.068944],
    [-86.686828, 36.066687],
    [-86.686384, 36.066255],
    [-86.68534, 36.065241],
    [-86.684479, 36.064414],
    [-86.683133, 36.063115],
    [-86.679623, 36.059737],
    [-86.678333, 36.058498],
    [-86.676034, 36.056287],
    [-86.674682, 36.054977],
    [-86.674009, 36.054356],
    [-86.673475, 36.053889],
    [-86.672979, 36.053508],
    [-86.672327, 36.053029],
    [-86.671659, 36.052585],
    [-86.670245, 36.051761],
    [-86.669644, 36.051405],
    [-86.662882, 36.047478],
    [-86.661168, 36.046483],
    [-86.654944, 36.042854],
    [-86.653744, 36.042158],
    [-86.653164, 36.041822],
    [-86.65188, 36.041078],
    [-86.648946, 36.039336],
    [-86.648148, 36.038835],
    [-86.647468, 36.038335],
    [-86.646789, 36.03779],
    [-86.645693, 36.03683],
    [-86.643865, 36.035185],
    [-86.639731, 36.031446],
    [-86.627952, 36.020911],
    [-86.624142, 36.017503],
    [-86.619547, 36.01338],
    [-86.6159, 36.010066],
    [-86.615237, 36.009515],
    [-86.613432, 36.0079],
    [-86.612779, 36.007309],
    [-86.609975, 36.00478],
    [-86.607239, 36.002343],
    [-86.606539, 36.001752],
    [-86.605858, 36.00121],
    [-86.605057, 36.000595],
    [-86.604428, 36.000138],
    [-86.603548, 35.999533],
    [-86.602738, 35.999005],
    [-86.600608, 35.99765],
    [-86.598399, 35.996254],
    [-86.597598, 35.995744],
    [-86.595924, 35.994679],
    [-86.594234, 35.993613],
    [-86.590916, 35.991495],
    [-86.589118, 35.990357],
    [-86.588385, 35.989893],
    [-86.586559, 35.988737],
    [-86.585634, 35.988157],
    [-86.584627, 35.987515],
    [-86.583848, 35.987029],
    [-86.583327, 35.986686],
    [-86.582779, 35.986309],
    [-86.582321, 35.985981],
    [-86.581752, 35.985555],
    [-86.581264, 35.98517],
    [-86.580661, 35.984671],
    [-86.579916, 35.984012],
    [-86.579232, 35.983357],
    [-86.578152, 35.98224],
    [-86.576337, 35.980359],
    [-86.575095, 35.979077],
    [-86.574627, 35.978585],
    [-86.573866, 35.977797],
    [-86.573318, 35.977203],
    [-86.572813, 35.976625],
    [-86.572047, 35.975702],
    [-86.571468, 35.974939],
    [-86.57104, 35.97436],
    [-86.570729, 35.973907],
    [-86.569429, 35.97196],
    [-86.568106, 35.969986],
    [-86.567143, 35.968555],
    [-86.566261, 35.967227],
    [-86.565458, 35.966038],
    [-86.564135, 35.964064],
    [-86.563808, 35.963577],
    [-86.562923, 35.962431],
    [-86.562208, 35.961637],
    [-86.561347, 35.960743],
    [-86.560586, 35.960003],
    [-86.558722, 35.958325],
    [-86.556911, 35.956699],
    [-86.555076, 35.955047],
    [-86.553243, 35.953402],
    [-86.55125, 35.951614],
    [-86.549477, 35.950019],
    [-86.547638, 35.94837],
    [-86.545789, 35.946705],
    [-86.543903, 35.945007],
    [-86.541997, 35.943288],
    [-86.540144, 35.941633],
    [-86.538282, 35.939952],
    [-86.53674, 35.938568],
    [-86.536123, 35.938009],
    [-86.535361, 35.937314],
    [-86.533463, 35.93562],
    [-86.532222, 35.934494],
    [-86.530468, 35.932916],
    [-86.529282, 35.93182],
    [-86.526661, 35.929481],
    [-86.524888, 35.927878],
    [-86.52305, 35.926228],
    [-86.521151, 35.924521],
    [-86.519184, 35.922747],
    [-86.517596, 35.921334],
    [-86.516905, 35.920744],
    [-86.515161, 35.919312],
    [-86.514264, 35.918565],
    [-86.512109, 35.916667],
    [-86.510242, 35.914978],
    [-86.508388, 35.913314],
    [-86.506519, 35.911636],
    [-86.504655, 35.909955],
    [-86.502791, 35.90828],
    [-86.501057, 35.906718],
    [-86.497301, 35.903334],
    [-86.495421, 35.901635],
    [-86.493553, 35.899964],
    [-86.49176, 35.898351],
    [-86.489893, 35.896662],
    [-86.488842, 35.895718],
    [-86.487981, 35.894941],
    [-86.486079, 35.893225],
    [-86.481612, 35.889199],
    [-86.479728, 35.887483],
    [-86.478599, 35.88646],
    [-86.478359, 35.886764],
    [-86.477473, 35.885964],
    [-86.477302, 35.885279],
    [-86.47581, 35.884458],
    [-86.47578, 35.883895],
    [-86.474104, 35.882379],
    [-86.472244, 35.881185],
    [-86.471766, 35.880774],
    [-86.471656, 35.880148],
    [-86.470062, 35.878651],
    [-86.468884, 35.877582],
 [-86.46807, 35.876863],
 [-86.466479, 35.87544],
 [-86.466044, 35.875051],
 [-86.46552, 35.875064],
 [-86.46287, 35.872243],
 [-86.462718, 35.872452],
 [-86.462304, 35.871727],
 [-86.462158, 35.871939],
 [-86.461551, 35.871387],
 [-86.461256, 35.870772],
 [-86.460852, 35.870715],
 [-86.460619, 35.870217],
 [-86.458247, 35.868054],
 [-86.458044, 35.868165],
 [-86.456565, 35.866532],
 [-86.456362, 35.866633],
 [-86.454251, 35.864425],
 [-86.454043, 35.864526],
 [-86.453851, 35.864041],
 [-86.453459, 35.863999],
 [-86.449547, 35.86012],
 [-86.449357, 35.860284],
 [-86.447554, 35.858318],
 [-86.447323, 35.858441],
 [-86.444287, 35.85534],
 [-86.444049, 35.855467],
 [-86.440992, 35.852347],
 [-86.440793, 35.852504],
 [-86.43994, 35.851739],
 [-86.439824, 35.85128]])
points_west = np.array([[-86.723107, 36.112157], [-86.722774, 36.111553], [-86.722244, 36.110692], [-86.721938, 36.110212], [-86.721366, 36.109348], [-86.721008, 36.108858], [-86.720564, 36.108289], [-86.720156, 36.107812], [-86.719591, 36.10721], [-86.719238, 36.106859], [-86.718618, 36.106293], [-86.718132, 36.105883], [-86.717631, 36.105486], [-86.717242, 36.105204], [-86.716923, 36.104985], [-86.716388, 36.104634], [-86.715736, 36.104222], [-86.715403, 36.104008], [-86.715214, 36.103874], [-86.714728, 36.103536], [-86.714401, 36.103287], [-86.714003, 36.10297], [-86.713494, 36.102543], [-86.712981, 36.10207], [-86.712597, 36.101692], [-86.71221, 36.101284], [-86.711994, 36.101041], [-86.711719, 36.100729], [-86.710824, 36.099689], [-86.708148, 36.096341], [-86.701283, 36.08786], [-86.700291, 36.086542], [-86.699787, 36.085822], [-86.698912, 36.084421], [-86.698048, 36.083042], [-86.696539, 36.080665], [-86.693332, 36.075614], [-86.69156, 36.072505], [-86.691499, 36.072406], [-86.690937, 36.071489], [-86.690256, 36.070509], [-86.690042, 36.070255], [-86.689336, 36.069445], [-86.688948, 36.06903], [-86.688801, 36.068891], [-86.687265, 36.067384], [-86.686772, 36.06691], [-86.68464, 36.064852], [-86.682614, 36.062901], [-86.68018, 36.060561], [-86.678717, 36.059149], [-86.675894, 36.05643], [-86.674809, 36.055382], [-86.674185, 36.054782], [-86.673487, 36.054166], [-86.672985, 36.053763], [-86.672471, 36.053376], [-86.671933, 36.053001], [-86.671274, 36.052573], [-86.670169, 36.051922], [-86.669553, 36.051566], [-86.660869, 36.046524], [-86.659095, 36.045488], [-86.658635, 36.045231], [-86.656611, 36.044073], [-86.652232, 36.041518], [-86.651389, 36.041028], [-86.650422, 36.040467], [-86.649292, 36.039803], [-86.648634, 36.039412], [-86.647971, 36.038968], [-86.647275, 36.038454], [-86.646417, 36.037757], [-86.645838, 36.037228], [-86.64462, 36.036147], [-86.639615, 36.031659], [-86.633619, 36.02628], [-86.630294, 36.0233], [-86.624844, 36.018414], [-86.617725, 36.012028], [-86.616907, 36.011301], [-86.615415, 36.009959], [-86.612372, 36.007228], [-86.611343, 36.006305], [-86.60998, 36.005082], [-86.608146, 36.003433], [-86.606866, 36.002295], [-86.606063, 36.001635], [-86.605613, 36.00128], [-86.604935, 36.000757], [-86.603886, 36.000012], [-86.602857, 35.999322], [-86.601388, 35.998392], [-86.598759, 35.996727], [-86.596498, 35.995287], [-86.594244, 35.993858], [-86.592442, 35.99271], [-86.589934, 35.991128], [-86.587781, 35.989762], [-86.585541, 35.988341], [-86.583477, 35.987024], [-86.582856, 35.98661], [-86.582037, 35.98603], [-86.581327, 35.985488], [-86.580614, 35.984904], [-86.580086, 35.984446], [-86.579007, 35.98343], [-86.578419, 35.982824], [-86.57759, 35.981967], [-86.576743, 35.981091], [-86.574716, 35.979005], [-86.573478, 35.977698], [-86.572806, 35.976956], [-86.571743, 35.975676], [-86.571057, 35.974752], [-86.57023, 35.97357], [-86.569317, 35.972191], [-86.568695, 35.97128], [-86.567547, 35.969539], [-86.566506, 35.967976], [-86.566124, 35.967411], [-86.565288, 35.966179], [-86.564383, 35.964937], [-86.563726, 35.964101], [-86.562858, 35.963099], [-86.561525, 35.961702], [-86.560374, 35.960604], [-86.559466, 35.959784], [-86.557822, 35.958305], [-86.555949, 35.956626], [-86.5541, 35.954952], [-86.552149, 35.953202], [-86.550387, 35.951617], [-86.5486, 35.950005], [-86.546685, 35.948286], [-86.544857, 35.946641], [-86.542911, 35.944893], [-86.541105, 35.94327], [-86.539289, 35.941621], [-86.537414, 35.939938], [-86.535623, 35.938337], [-86.533648, 35.936537], [-86.53239, 35.9354], [-86.531836, 35.934898], [-86.529967, 35.933224], [-86.529234, 35.932561], [-86.52876, 35.932134], [-86.52831, 35.931735], [-86.527362, 35.930889], [-86.526257, 35.929905], [-86.52444, 35.928269], [-86.520695, 35.924895], [-86.51884, 35.923226], [-86.516968, 35.921536], [-86.515133, 35.919885], [-86.51328, 35.918217], [-86.511545, 35.916661], [-86.50959, 35.914899], [-86.50775, 35.913244], [-86.505935, 35.911607], [-86.50402, 35.90988], [-86.502282, 35.90832], [-86.500744, 35.906927], [-86.498908, 35.905275], [-86.498435, 35.904849], [-86.496586, 35.903185], [-86.494718, 35.901502], [-86.491054, 35.898209], [-86.488529, 35.89593], [-86.487348, 35.894864], [-86.485704, 35.893381], [-86.482591, 35.89058], [-86.479942, 35.888181], [-86.478359, 35.886764], [-86.477473, 35.885964], [-86.47581, 35.884458], [-86.472244, 35.881185], [-86.471766, 35.880774], [-86.46552, 35.875064], [-86.462718, 35.872452], [-86.462158, 35.871939], [-86.461551, 35.871387], [-86.460852, 35.870715], [-86.458044, 35.868165], [-86.456362, 35.866633], [-86.454043, 35.864526], [-86.453459, 35.863999], [-86.449357, 35.860284], [-86.447323, 35.858441], [-86.444049, 35.855467], [-86.440793, 35.852504], [-86.43994, 35.851739]])


def is_wb(positions, lon_list, lat_list):
    n_true = 0
    n_side = 0

    curr_point = np.asarray([lon_list[0], lat_list[0]])
    for i in range(len(positions)):
        if lon_list[i] == 0 or lat_list[i] == 0:
            continue
        point = np.asarray([lon_list[i], lat_list[i]])
        dist_east = dist_to_line(point, points_east)
        dist_west = dist_to_line(point, points_west)
        if dist_west < dist_east:
            n_true += 1
        else:
            n_true -= 1

        with open("/tmp/log.txt", "a") as f:
            f.write(f"{dist_east}{dist_west}\n")
        if dist_west > side_street_thresh and dist_east > side_street_thresh:
            n_side += 1
        else:
            n_side -= 1

    #for i in range(len(positions) - 1):
    #    if positions[i+1] > positions[i]:
    #        n_true += 1
    #    else:
    #        n_true -= 1

    wb = 0
    if n_side > 0:  # Majority of last 10 readings are off the highway
        wb = 0  # On a side street
    elif n_true > 0:
        wb = 1  # Westbound
    else:
        wb = -1  # Eastbound

    dist_east = dist_to_line(curr_point, points_east)
    dist_west = dist_to_line(curr_point, points_west)
    return wb, n_true, n_side, dist_east, dist_west

def dist_to_line(point, polyline):
    xc, yc = point
    x = polyline[:, 0]
    y = polyline[:, 1]
    ix0 = bisect.bisect(x, xc)
    if ix0 >= polyline.shape[0]:
        return side_street_thresh + 0.1
    xa = x[ix0]
    xb = x[ix0+1]
    ya = y[ix0]
    yb = y[ix0+1]

    return abs((xb-xa)*(yc-ya) - (yb-ya)*(xc-xa)) / np.sqrt((xb-xa) ** 2 + np.square(yb-ya) ** 2)

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
        rospy.Subscriber(acc_status_topic, Int16, acc_status_callback)
        rospy.Subscriber(gps_fix_topic, NavSatFix, gps_fix_callback)
        rospy.Subscriber(gps_fix_time_reference_topic, TimeReference, gps_fix_time_reference_callback)

        self.is_wb_pub = rospy.Publisher('/is_westbound', Int16, queue_size=10)
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
                global latitude
                global longitude
                current_time = rospy.Time.now()
                assert gps_update_time is not None, "GPS data has never been received!"
                assert can_update_time is not None, "CAN data has never been received!"
                assert abs((current_time - gps_update_time).to_sec()) < 30, "GPS data more than 30 seconds old!"
                assert abs((current_time - can_update_time).to_sec()) < 30, "CAN data more than 30 seconds old!"
                last_positions.pop()
                last_positions.insert(0, position)
                if longitude is not None:
                    last_lons.pop()
                    last_lons.insert(0, longitude)
                if latitude is not None:
                    last_lats.pop()
                    last_lats.insert(0, latitude)
                wb, n_true, n_side, dist_east, dist_west = is_wb(last_positions, last_lons, last_lats)
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