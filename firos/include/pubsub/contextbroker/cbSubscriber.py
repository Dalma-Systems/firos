# MIT License
#
# Copyright (c) <2015> <Ikergune, Etxetar>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
# (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge,
# publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

__author__ = "Dominik Lux"
__credits__ = ["Peter Detzner"]
__maintainer__ = "Dominik Lux"
__version__ = "0.0.1a"
__status__ = "Developement"

import time
import requests
import json
import threading
try:
    # Python 3
    import _thread as thread
    from http.server import BaseHTTPRequestHandler
    from http.server import HTTPServer
except ImportError:
    # Pyrhon 2
    import thread
    from BaseHTTPServer import BaseHTTPRequestHandler
    from BaseHTTPServer import HTTPServer

from include.constants import Constants as C
from include.logger import Log
from include.pubsub.genericPubSub import Subscriber
from include.ros.topicHandler import RosTopicHandler
from include.FiwareObjectConverter.objectFiwareConverter import ObjectFiwareConverter

import rospy
from std_msgs.msg import String


class CbSubscriber(Subscriber):
    ''' The CbSubscriber handles the subscriptions on the ContextBroker.
        Only the url CONTEXT_BROKER / v2 / subcriptions  is used here!
        As on CbPublisher on shutdown all subscriptions are deleted form 
        ContextBroker. 

        this Objects also converts the received data from ContextBroker
        back into a Python-Object. 

        Here each topic of an robot is subscribed seperately.

        THIS IS THE ONLY FILE WHICH OPERATES ON /v2/subscriptions
    '''

    # Saves the subscriptions IDs returned from ContextBroker.
    # Follwoing Structure: subscriptionIds[ROBOT_ID][TOPIC] returns a sub-Id in String
    subscriptionIds = {}
    CB_BASE_URL = None
    FIROS_NOTIFY_URL = None

    def __init__(self):
        ''' 
            Lazy Initialization of CB_BASE_URL
            and setting up Configuration-Parameters
        '''
        # Do nothing if no Configuration is provided!
        if self.configData is None:
            Log("WARNING", "No Configuration for Context-Broker found!")
            self.noConf = True
            return
        else:
            self.noConf = False

        ## Set Configuration
        data = self.configData

        if data is not None and "address" not in data or "port" not in data:
            raise Exception("No Context-Broker specified!")

        if "subscription" not in data:
            data["subscription"] = dict(throttling=0, subscription_length=300, subscription_refresh_delay=0.9)

        if "throttling" not in data["subscription"]:
            data["subscription"]["throttling"] = 0
        else: 
            data["subscription"]["throttling"] = int(data["subscription"]["throttling"])

        if "subscription_length" not in data["subscription"]:
            data["subscription"]["subscription_length"] = 300
        else:
            data["subscription"]["subscription_length"] = int(data["subscription"]["subscription_length"])

        if "subscription_refresh_delay" not in data["subscription"]:
            data["subscription"]["subscription_refresh_delay"] = 0.9
        else:
            data["subscription"]["subscription_refresh_delay"] = float(data["subscription"]["subscription_refresh_delay"])


        self.data = data
        self.serverIsRunning = False
        self.CB_BASE_URL = "http://{}:{}".format(data["address"], data["port"])


    def subscribe(self, topicList, topicTypes, msgDefintions):
        ''' topicList: A list of topics
            msgDefintions: The Messages-Definitions from ROS

            This method only gets called once (or multiple times, if we get a reset!)! So we need to make sure, that in this file
            'RosTopicHandler.publish' is called somehow independently after some Signal arrived (from elsewhere).

            Keep in mind that Firos can get a Reset-Signal, in this case, this method is called again. Make sure that this method can get called 
            multiple times!

            In this Context-Broker-Subscriber we spawn ONE HTTP-Base-Server in another Thread, which will be used, 
            so that the Context-Broker can notify us after it received a Message. 

            In addition to that, the Context-Broker needs to know how to notify us. This is solved by adding subscriptions into 
            the Context-Broker, which we need to manually maintain. Again we start a Thread for each Topic which handles the Subscriptions



            So After everything is set up, Firos can be notified, explicitly here the method:
            """CBServer.CBHandler.do_post""" is invoked by Notification. This method handles the Conversion back into a conform "ROS-Message".

            After we did the Conversion, we simply need to call  """RosTopicHandler.publish"""


        '''
        # Do nothing if no Configuratuion
        if self.noConf:
            return
        
        ### Start the HTTPServer and wait until it is ready!
        if not self.serverIsRunning:
            server_ready = threading.Event()
            self.server = CBServer(server_ready)
            thread.start_new_thread(self.server.start, ())
            self.serverIsRunning = True
            server_ready.wait()

        # If not already subscribed, start a new thread which handles the subscription for each topic for an robot.
        # And only If the topic list is not empty!
        for topic in topicList:
            if topic not in self.subscriptionIds:
                Log("INFO", "Subscribing on Context-Broker to topics: " + str(list(topicList)))
                thread.start_new_thread(self.subscribeThread, (topic, topicTypes, msgDefintions)) #Start Thread via subscription         


    def unsubscribe(self):
        ''' 
            Simply unsubscribed from all tracked subscriptions
            and also stop the HTTP-Server
        '''
        # Do nothing if no Configuratuion
        if self.noConf:
            return

        # close HTTP-Server
        self.server.close()

        # Unsubscribe to all Topics
        for topic in self.subscriptionIds:
            response = requests.delete(self.CB_BASE_URL + self.subscriptionIds[topic])
            self._checkResponse(response, subID=self.subscriptionIds[topic])



    ####################################################
    ########## Helpful Classes and Methods #############
    ####################################################

    def subscribeThread(self, topic, topicTypes, msgDefintions):
        ''' 
            A Subscription-Thread. Its Life-Cycle is as follows:
            -> Subscribe -> Delete old Subs-ID -> Save new Subs-ID -> Wait ->

            topic: The Topic (string) to subscribe to.
        '''
        while True:
            # Subscribe
            jsonData = self.subscribeJSONGenerator(topic, topicTypes, msgDefintions)
            response = requests.post(self.CB_BASE_URL + "/v2/subscriptions", data=jsonData, headers={'Content-Type': 'application/json'})
            self._checkResponse(response, created=True, robTop=topic)

            if 'Location' in response.headers:
                newSubID = response.headers['Location'] # <- get subscription-ID
            else:
                Log("WARNING",  "Firos was not able to subscribe to topic: {}".format(topic))

            # Unsubscribe
            if topic in self.subscriptionIds:
                response = requests.delete(self.CB_BASE_URL + self.subscriptionIds[topic])
                self._checkResponse(response, subID=self.subscriptionIds[topic])
                
            # Save new ID
            self.subscriptionIds[topic] = newSubID

            # Wait
            time.sleep(int(self.data["subscription"]["subscription_length"] * self.data["subscription"]["subscription_refresh_delay"])) # sleep Length * Refresh-Rate (where 0 < Refresh-Rate < 1)
            Log("INFO", "Refreshing Subscription for topic: " + str(topic))


    def subscribeJSONGenerator(self, topic, topicTypes, msgDefintions):
        ''' 
            This method returns the correct JSON-format to subscribe to the ContextBroker. 
            The Expiration-Date/Throttle and Type of topics is retreived here via the configuration we got

            topic: The actual topic to subscribe to.
        '''
        # This struct correspondes to following JSON-format:
        # https://fiware-orion.readthedocs.io/en/master/user/walkthrough_apiv2/index.html#subscriptions
        struct =  {
            "subject": {
                "entities": [
                    {
                    "id": str(topic.split("/")[1]).replace("_", ":"),  # OCB Specific!!
                    #"type": topicTypes[topic].replace("/", "%2F") # OCB Specific!!
                    "type": "Robot" # OCB Specific!!
                    }
                ],
                "condition": {
                    "attrs": [str(topic.split("/")[2])]
                }
            },
            "notification": {
            "http": {
                "url": "http://{}:{}".format(C.EP_SERVER_ADRESS, self.server.port)
            },
            "attrs": [str(topic.split("/")[2])]
            },
            "expires": time.strftime("%Y-%m-%dT%H:%M:%S.00Z", time.gmtime(time.time() + self.data["subscription"]["subscription_length"])), # ISO 8601
            "throttling": self.data["subscription"]["throttling"]  
            }
        # Create ROS publisher
        global pub
        pub = rospy.Publisher(topic, String, queue_size=3, latch=False)
        return json.dumps(struct)


    def _checkResponse(self, response, robTop=None, subID=None, created=False):
        ''' 
            If a not good response from ContextBroker is received, the error will be printed.
    
            response: The response from ContextBroker
            robTop:   A string (topic), for the curretn robot/topic
            subID:    The Subscription ID string, which should get deleted
            created:  Creation or Deletion of a subscription (bool)
        '''
        if not response.ok:
            if created:
                Log("ERROR", "Could not create subscription for topic {} in Context-Broker :".format(robTop))
                Log("ERROR", response.content)
            else:
                Log("WARNING", "Could not delete subscription {} from Context-Broker :".format(subID))
                Log("WARNING", response.content)





class CBServer:
    '''
        This is the HTTPServer, which start listening on an adress and a free port
        Here we provide 3 methods: Initialize, start and stop. Start and stop either 
        start or stop this Server.
    '''
    def __init__(self, thread_event):
        '''
            Set up HTTPServer
            thread_event: The Event where the main Thread waits on
        '''
        self.stopped = False
        self.thread_event = thread_event

        Protocol = "HTTP/1.0"

        if C.EP_SERVER_PORT is not None and isinstance(C.EP_SERVER_PORT, int) :
            server_address = ("0.0.0.0", C.EP_SERVER_PORT)
        else:
            server_address = ("0.0.0.0", 0)

        self.CBHandler.protocol_version = Protocol
        self.httpd = HTTPServer(server_address, self.CBHandler)

    def start(self):
        '''
            This start the HTTPServer and notifies thread_event
        '''
        # Get Port and HostName and save port
        sa = self.httpd.socket.getsockname()
        self.port = sa[1]
        Log("INFO", "\nListening for Context-Broker-Messages on: ", C.EP_SERVER_ADRESS, ":", sa[1])

        # Notify and start handling Requests
        self.thread_event.set()
        while not self.stopped:
            self.httpd.handle_request()

    def close(self):
        '''
            Stops the HTTPServer
        '''
        self.stopped = True

    class CBHandler(BaseHTTPRequestHandler):
        ''' This is the FIROS-HTTP-Request-Handler. It is needed,
            because the ContextBroker sends Information about the
            subscriptions via HTTP. This Class just handles incoming 
            Requests and converts the received Data into a "ROS-conform Message".
            in """do_POST""" we invoke """RosTopicHandler.publish"""
        '''
        def log_message(self, format, *args):
            ''' Suppress prints! '''
            return

        def do_GET(self):
            '''
                We do not respond to GETs. We do Nothing!!
            '''
            pass


        def do_POST(self):
            ''' The ContextBroker is informing us via one of our subscriptions.
                We convert the received content back and publish 
                it in ROS.

                self: The "request" from the Context-Broker

                we invoke """RosTopicHandler.publish""" here!
            '''
            # retreive Data and get the updated information
            recData = self.rfile.read(int(self.headers['Content-Length']))
            receivedData = json.loads(recData)
            data = receivedData['data'][0] # Specific to NGSIv2 
            #jsonData = json.dumps(data)            
            pub_data = data['refDestination']['value']
            global pub
            pub.publish(pub_data)
            # # Send OK!
            self.send_response(204)
            self.end_headers() # Python 3 needs an extra end_headers after send_response

            # obj = self.TypeValue()
            # ObjectFiwareConverter.fiware2Obj(jsonData, obj, setAttr=True, useMetaData=False, encoded=True)
            # obj.id = obj.id.replace(".", "/")
            # obj.type = obj.type.replace("%2F", "/")
            
            # objType = obj.type
            # topic = obj.id

            # del data["id"]
            # del data["type"]
            # tempDict = dict(type=objType, value=data)

            # dataStruct = self._buildTypeStruct(tempDict)

            # RosTopicHandler.publish(topic, obj.__dict__, dataStruct)
            # # # Send OK!
            # self.send_response(204)
            # self.end_headers() # Python 3 needs an extra end_headers after send_response


        ### Back Conversion From Entity-JSON into Python-Object
        def _buildTypeStruct(self, obj):
            ''' This generates a struct containing a type (the actual ROS-Message-Type) and 
                its value (either empty or more ROS-Message-Types).

                This struct is used later to recursivley load needed Messages and fill them with 
                content before they are posted back to ROS.

                obj:    The received update from Context-Broker
            '''
            s = {}

            # Searching for a point to get ROS-Message-Types from the obj
            if 'value' in obj and 'type' in obj and "/" in obj['type'] : 
                s['type'] = obj['type']
                objval = obj['value'] 
                s['value'] = {}

                # For each value in Object repeat!
                for k in objval:
                    if 'type' in objval[k] and 'value' in objval[k] and objval[k]['type'] == 'array': # Check if we got an Array-Type value
                        l = []
                        for klist in objval[k]['value']:
                            l.append(self._buildTypeStruct(klist))
                        s['value'][k] = l
                    else:
                        s['value'][k] = self._buildTypeStruct(objval[k])

            return s

        class TypeValue(object):
            ''' A Stub-Object to parse the received data
            '''