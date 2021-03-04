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

import json
import requests
import os
import tf
import time
from include.logger import Log
from include.constants import Constants as C
from include.FiwareObjectConverter.objectFiwareConverter import ObjectFiwareConverter
from include.pubsub.genericPubSub import Publisher
from geometry_msgs.msg import Quaternion
import datetime
from queue import Queue
#from include.pubsub.contextbroker.cbSubscriber import context_id as CONTEXT_ID

class CbPublisher(Publisher):
    ''' The CbPublisher handles the Enities on CONTEXT_BROKER / v2 / entities .
        It creates not creaed Entities and updates their attributes via 'publishToCB'.
        On Shutdown the tracked Entities are deleted. 

        Also the rawMsg is converted here via the Object Converter

        THIS IS THE ONLY FILE WHICH OPERATES ON /v2/entities

        Also this Method is called, after FIROS received a Message 
    '''

    # Keeps track of the posted Content on the ContextBroker
    # via posted_history[ROBOT_ID + "/" + TOPIC] 
    posted_history = {}
    CB_HEADER = {'Content-Type': 'application/json'}
    CB_BASE_URL = None
    q = Queue()

    def __init__(self):
        ''' Lazy Initialization of CB_BASE_URL
            And set up the configuration via the config we received

            If No configuration is provided, we simply do nothing
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
        if "address" not in data or "port" not in data: 
            raise Exception("No Context-Broker specified!")

        self.data = data
        self.CB_BASE_URL = "http://{}:{}/v2/entities/".format(data["address"], data["port"])


    def publish(self, topic, rawMsg, msgDefintionDict):
        ''' This is the actual publish-Routine which updates and creates Entities on the
            ContextBroker. It also keeps track via posted_history on already posted entities and topics

            topic:   a string, corresponding to the topic in ros
            rawMsg:  the raw data directly obtained from rospy
            msgDefintionDict: The Definition as obtained directly from ROS-Messages

            We do not need to invoke something special here. This method gets called automatically,
            after Firos received a Message from the ROS-World

            TODO DL During Runtime an Entitiy might get deleted, check it here!
        '''
        # Do nothing if no Configuratuion
        if self.noConf:
            return

        # Create Update-JSON
        obj = {s: getattr(rawMsg, s, None) for s in rawMsg.__slots__}
        obj["id"] = topic.split("/")[1].replace('_', ':')
        try:
            attr = topic.split("/")[2]
        except:
            return
        
        data = self.set_data(attr, rawMsg)        
        
        if data is not False:
            jsonStr = json.dumps(data)
        else:
            return

        # if struct not initilized, intitilize it even on ContextBroker!
        if topic not in self.posted_history:
            self.posted_history[topic] = rawMsg
            response = requests.post(self.CB_BASE_URL + C.ID_PREFIX + obj["id"] + "/attrs", data=jsonStr, headers=self.CB_HEADER, timeout=5)
            self._responseCheck(response, attrAction=0, topEnt=topic)
            return

        # Replace previous rawMsg with current one
        self.posted_history[topic] = rawMsg

        # Update attribute on ContextBroker
        try:
            response = requests.patch(self.CB_BASE_URL + C.ID_PREFIX + obj["id"] + "/attrs", data=jsonStr, headers=self.CB_HEADER, timeout=5)
            self._responseCheck(response, attrAction=1, topEnt=topic)
            # send requests which are enqueued
            while not self.q.empty():
                response = requests.patch(self.CB_BASE_URL + C.ID_PREFIX + obj["id"] + "/attrs", data=self.q.get(), headers=self.CB_HEADER, timeout=5)
                self._responseCheck(response, attrAction=1, topEnt=topic)
                time.sleep(1)
        except:
            print("Connection issue, saving request to queue")
            # if the PATCH fails, save to queue and re-send when connection returns
            if not self.q.full():
                self.q.put(jsonStr)
            return


    def unpublish(self):
        ''' 
            Removes all previously tracked topics on ContextBroker
            This method also gets automaticall called, someone sent Firos the Shutdown Signal
        '''
        for idd in self.posted_history.keys():
            response = requests.delete(self.CB_BASE_URL + idd.replace("/", ".")) # OCB Specific!!
            self._responseCheck(response, attrAction=2, topEnt=idd)
        
        


    def _responseCheck(self, response, attrAction=0, topEnt=None):
        ''' Check if Response is ok (2XX and some 3XX). If not print an individual Error.
            
            response: the actual response
            attrAction: One of [0, 1, 2]  which maps to -> [Creation, Update, Deletion]
            topEnt: the String of an Entity or a topic, which was used
        '''
        if not response.ok:
            if attrAction == 0:
                Log("WARNING", "Could not create Entitiy {} in Contextbroker :".format(topEnt))
                Log("WARNING", response.content)
            elif attrAction == 1:
                Log("ERROR", "Cannot update attributes in Contextbroker for topic: {} :".format(topEnt))
                Log("ERROR", response.content)
            else:
                Log("WARNING", "Could not delete Entitiy {} in Contextbroker :".format(topEnt))
                Log("WARNING", response.content)

    def set_data(self, attribute, payload):
        ''' Return data for Fiware publication, according to attribute type (FEATS specific)
        
        '''
        if attribute == 'status':
            data = {
                attribute:
                {
                    'type': 'Text',
                    'value': payload.data,
                    'metadata': {
                        'context': {
                            'type': 'Text',
                            'value': C.CONTEXT_ID
                        },
                        'dateModified': {
                            "type": "DateTime",
                            "value": time.strftime("%Y-%m-%dT%H:%M:%S.00Z", time.gmtime(time.time())) # ISO 8601
                        }
                    }
                }
            }
        elif attribute == 'battery':
            data = {
                attribute:
                {
                    'type': 'Number',
                    'value': payload.data
                }
            }
        elif attribute == 'location':
            angle = tf.transformations.euler_from_quaternion([
                            payload.orientation.x,
                            payload.orientation.y,
                            payload.orientation.z,
                            payload.orientation.w])[2]
            data = {
                attribute:
                {
                    'type': 'geo:json',
                    'value': {
                        'type': 'Point',
                        'coordinates': [
                            payload.position.x,
                            payload.position.y
                        ]
                    },
                    "metadata": {
                        "angle": {
                            "type": "Double",
                            "value": angle
                        }
                    }
                }
            }
        elif attribute == 'heartbeat':
            data = {
                attribute: {
                    "type": "DateTime",
                    "value": time.strftime("%Y-%m-%dT%H:%M:%S.00Z", time.gmtime(time.time())), # ISO 8601
                    "metadata" : {}
                }
            }
        else:
            Log("WARNING", "You are trying to change the wrong attribute!")
            return
        # add current timestamp to data
        data['dateModified'] = {
            "type": "DateTime",
            "value": time.strftime("%Y-%m-%dT%H:%M:%S.00Z", time.gmtime(time.time())),
            "metadata": {}
        }
        return data