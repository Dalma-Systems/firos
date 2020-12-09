import os
import sys
import rospy
import importlib
import time
import copy
import json
import requests
import tf
import tf2_ros

from include.logger import Log
from include.constants import Constants as C 
#from include.libLoader import LibLoader
from include.ros.rosConfigurator import RosConfigurator
from include import confManager
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
#from include.ros.topicHandler import loadMsgHandlers

class FeatsHandler:
    ''' The class FeatsHandler is used to handle all the events
    resulting from the operation and workflow of FEATS, in the
    scope of the DIH2 programme.
    '''
    def __init__(self):
        '''Initialize the class by creating the required publishers
        and subscribers.
        '''
        Log("INFO", ('\nCreating a new FEATS handler...'))

        self.firstRun = True
        # Get Orion configuration
        self.configData = self.get_cb_config()
        self.lastBattery = 0.0
        self.idleGoal = False # this variable stores whether the robot is moving to an idle station or not

        # Init ROS publishers
        self.routePlannerXYTPub = rospy.Publisher('/route_planner/goalXYT', Vector3, queue_size=3)
        self.locationPub = rospy.Publisher('/' + C.ROBOT_ID + '/location', Pose, queue_size=3)
        self.statusPub = rospy.Publisher('/' + C.ROBOT_ID + '/status', String, queue_size=3)
        self.batteryPub = rospy.Publisher('/' + C.ROBOT_ID + '/battery', Float32, queue_size=3)

        # Init ROS subscribers
        rospy.Subscriber('/battery/level', Float32, self.battery_cb)
        rospy.Subscriber('/feats/status', String, self.status_cb)

        ## Set Configuration
        data = self.configData['contextbroker']
        if "address" not in data or "port" not in data: 
            raise Exception("No Context-Broker specified!")

        self.data = data
        self.CB_BASE_URL = "http://{}:{}/v2/entities/".format(data["address"], data["port"])

        # Get topics (already with robot_id from config)
        topics = confManager.getRobots(True)

        # Subscribe to topics in topics.json
        for key in topics:
            if topics[key][1] == 'publisher':
                self.init_sub(key)        

        Log("INFO", ('\nFEATS handler initialized!'))
        
        # Main loop
        self.loop()

    def loop(self):
        '''The main loop publishes the robot location
        periodically to the FIROS topic
        '''
        while not rospy.is_shutdown():
            self.publish_location()
            rospy.sleep(2)
        return

    def get_cb_config(self):
        '''Reads configuration from config.json file
        '''
        config = {}
        current_path = os.path.dirname(os.path.abspath(__file__))
        conf_path = current_path + "/../../../config"
        config = json.load(open(conf_path + "/config.json"))
        return config[config['environment']]

    def init_sub(self, topic):
        ''' This method simply creates the ROS subscriptions
        to the passed topic. The callback method is dependent
        on the topic.
        '''
        Log("INFO", ('\nSubscribing to ' + topic))
        topic_type = topic.split('/')[2]
        if topic_type == 'refDestination':
            rospy.Subscriber(topic, String, self.ref_destination_cb)
        return

    def ref_destination_cb(self, data):
        '''This method is the callback to the refDestination
        topic. It handles this topic by performing an HTTP GET
        to Orion, asking for the entity location.
        '''
        # Ignore on first run
        if self.firstRun:
            self.firstRun = False
            #return # uncomment if skipInitialNotification is not set in the subscription
        
        # GET request to obtain received entity location
        response = requests.get(self.CB_BASE_URL + data.data + "/attrs/location")
        if response.status_code != 200:
            Log("INFO", ("Request failed: received status code " + str(response.status_code)))
            return
        data = json.loads(response.content)

        # Check if goal is an idle station
        if 'Idlestation' in data:
            self.idleGoal = True
        else:
            self.idleGoal = False

        # Send data as a Vector3 (slight hack, did not want to calculate a quaternion here)
        pose = Vector3()
        pose.x = data['value']['coordinates'][0]
        pose.y = data['value']['coordinates'][1]
        pose.z = data['metadata']['angle']['value']
        self.routePlannerXYTPub.publish(pose)
        return
    
    def status_cb(self, data):
        '''Publishes received status data to FIROS topic (simple remap)
        '''
        # Check if status is of type 'stopped', to decide if 'idle' or 'stopped'
        status = data.data
        if status == 'stopped' and self.idleGoal:
            status = 'idle'
            self.idleGoal = False

        self.statusPub.publish(status)

    def battery_cb(self, data):
        '''Publishes received battery data to FIROS topic (simple remap)
        '''
        if data.data != self.lastBattery:
            self.batteryPub.publish(data.data)
            self.lastBattery = data.data
    
    def publish_location(self):
        '''Retrieves current robot location and publishes to FIROS topic
        '''
        self.locationPub.publish(get_robot_position())
        
def xytheta_to_pose_stamped(x,y,theta):
    """Turns x,y coordinates to a Pose object"""
    pose = Pose()
    pose.position = Point(x,y,0)
    q = tf.transformations.quaternion_from_euler(0,0,theta)
    pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    return pose

def get_robot_position():
    """Gets current robot coordinates using TF and returns a Pose object"""
    # temp: tests
    #return xytheta_to_pose_stamped(1.0, -2.0, 1.2)
    while True:
        try:
            tfBuffer = tf2_ros.Buffer()
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qw = trans.transform.rotation.z
            qz = trans.transform.rotation.w
            th = tf.transformations.euler_from_quaternion([qx,qy,qz,qw])[2]
            return xytheta_to_pose_stamped(x, y, th)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue