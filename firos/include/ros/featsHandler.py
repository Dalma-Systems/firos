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
import time

from include.logger import Log
from include.constants import Constants as C 
#from include.libLoader import LibLoader
from include.ros.rosConfigurator import RosConfigurator
from include import confManager
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion, PoseWithCovarianceStamped
from threading import Timer
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
        self.status = 'idle'
        # Get Orion configuration
        self.configData = self.get_cb_config()
        self.lastBattery = 0.0
        self.idleGoal = False # this variable stores whether the robot is moving to an idle station or not
        self.context_id = ""
        self.workorder_id = ""
        self.heartbeat_timer = None

        # Init ROS publishers
        self.routePlannerXYTPub = rospy.Publisher('/route_planner/goalXYT', Vector3, queue_size=3)
        self.routePlannerPausePub = rospy.Publisher('/route_planner/cancel', String, queue_size=3)
        self.routePlannerResumePub = rospy.Publisher('/route_planner/resume', String, queue_size=3)
        self.locationPub = rospy.Publisher('/' + C.ROBOT_ID + '/location', Pose, queue_size=3)
        self.statusPub = rospy.Publisher('/' + C.ROBOT_ID + '/status', String, queue_size=3)
        self.batteryPub = rospy.Publisher('/' + C.ROBOT_ID + '/battery', Float32, queue_size=3, latch=True)
        self.heartbeatPub = rospy.Publisher('/' + C.ROBOT_ID + '/heartbeat', String, queue_size=3)
        self.connectionPub = rospy.Publisher('/' + C.ROBOT_ID + '/connection', Bool, queue_size=3)
        self.selfStatusPub = rospy.Publisher('/feats/status', String, queue_size=3)

        # Init ROS subscribers
        rospy.Subscriber('/battery/level', Float32, self.battery_cb)
        rospy.Subscriber('/feats/status', String, self.status_cb)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.location_cb)
        rospy.Subscriber('/charging/plugged', Bool, self.charging_cb)
        rospy.Subscriber('/ui/goal/cancel', String, self.cancel_cb)
        rospy.Subscriber('/ui/goal/resume', String, self.resume_cb)
        rospy.Subscriber('/ui/ready', String, self.ready_cb)

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
                Log("INFO", ('\nSubscribing to ' + key))
                topic_type = key.split('/')[2]
                if topic_type == 'refDestination':
                    rospy.Subscriber(key, String, self.ref_destination_cb)
                elif topic_type == 'action':
                    rospy.Subscriber(key, String, self.action_cb)

        Log("INFO", ('\nFEATS handler initialized!'))

        # Send first heartbeat
        self.send_heartbeat()
        
        # Main loop
        self.loop()

    def loop(self):
        '''The main loop publishes the robot location
        periodically to the FIROS topic
        '''
        while not rospy.is_shutdown():
            rospy.sleep(1)
        rospy.loginfo("FEATS Handler shutting down...")
        self.heartbeat_timer.cancel()
        return

    def send_heartbeat(self):
        '''Sends a heartbeat to ORION, i.e., an update
        of the "heartbeat" attribute
        '''
        self.checkConnectivity()
        self.heartbeatPub.publish('')
        self.heartbeat_timer = Timer(C.HEARTBEAT, self.send_heartbeat)
        self.heartbeat_timer.start()

    def checkConnectivity(self):
        try:
            requests.get("https://8.8.8.8", timeout=3)
            self.connectionPub.publish(True)
        except:
            self.connectionPub.publish(False)

    def get_cb_config(self):
        '''Reads configuration from config.json file
        '''
        config = {}
        current_path = os.path.dirname(os.path.abspath(__file__))
        conf_path = current_path + "/../../../config"
        config = json.load(open(conf_path + "/config.json"))
        return config[config['environment']]

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
        recv = json.loads(response.content)

        # Check if goal is an idle station
        if 'Idlestation' in data.data:
            self.idleGoal = True
        else:
            self.idleGoal = False

        # Save context ID
        self.workorder_id = C.CONTEXT_ID

        # Send data as a Vector3 (slight hack, did not want to calculate a quaternion here)
        pose = Vector3()
        pose.x = recv['value']['coordinates'][0]
        pose.y = recv['value']['coordinates'][1]
        pose.z = recv['metadata']['angle']['value']
        self.routePlannerXYTPub.publish(pose)
        return
    
    def action_cb(self, data):
        '''Handles actions by passing the info to route_planner
        '''
        action = data.data
        if action == 'pause':
            # robot stops current goal and replies "paused" + context_id
            self.routePlannerPausePub.publish('')
            self.statusPub.publish('paused')
        elif action == 'resume':
            # robot resumes goal and replies <last state> + context_id
            self.routePlannerResumePub.publish('')
            if self.status != 'moving':
                self.statusPub.publish(self.status)
        elif action == 'update':
            # perform update
            print('update')
            self.statusPub.publish('update')
        else:
            print("Action not recognized")
    
    def status_cb(self, data):
        '''Publishes received status data to FIROS topic (simple remap)
        '''
        # Check if status is of type 'stopped', to decide if 'idle' or 'stopped'
        status = data.data
        if status != 'charging':
            self.status = status
        
        if status == 'stopped' and self.idleGoal:
            status = 'idle'
            self.status = status
            self.idleGoal = False
        
        if status == 'stopped' or status == 'idle':
            C.CONTEXT_ID = self.workorder_id

        self.statusPub.publish(status)

    def battery_cb(self, data):
        '''Publishes received battery data to FIROS topic (simple remap)
        '''
        level = 0.0
        if data.data >= 250:
            level = 100.0
        elif data.data >= 240 and data.data < 250:
            level = 3.5 * data.data - 775
        elif data.data >= 230 and data.data < 240:
            level = 3.5 * data.data - 775
        elif data.data >= 220 and data.data < 230:
            level = 2.0 * data.data - 430
        elif data.data >= 210 and data.data < 220:
            level = 1.0 * data.data - 200
        elif data.data < 210:
            level = 0.0
        
        if level != self.lastBattery:
            self.batteryPub.publish(level)
            self.lastBattery = level

    def charging_cb(self, data):
        if data.data:
            self.selfStatusPub.publish('charging')
        else:
            self.selfStatusPub.publish(self.status)

    def location_cb(self, data):
        '''
        '''
        location = Pose()
        location.position = data.pose.pose.position
        location.orientation = data.pose.pose.orientation
        self.locationPub.publish(location)
    
    def publish_location(self):
        '''Retrieves current robot location and publishes to FIROS topic
        '''
        self.locationPub.publish(get_robot_position())

    ############ ROBOT UI ############
    
    def cancel_cb(self, data):
        # send specific context_id (action-robotui)
        C.CONTEXT_ID = 'action-robotui'
        self.routePlannerPausePub.publish('')

    def resume_cb(self, data):
        # send specific context_id (action-robotui)
        C.CONTEXT_ID = 'action-robotui'
        self.routePlannerResumePub.publish('')

    def ready_cb(self, data):
        # send specific context_id (action-robotui)
        C.CONTEXT_ID = 'action-robotui'
        self.statusPub.publish('ready')

############ AUXILIARY FUNCTIONS ############
        
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