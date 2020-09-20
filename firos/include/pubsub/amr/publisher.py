import json
import requests
import os

from include.logger import Log
#from include.constants import Constants as C
#from include.FiwareObjectConverter.objectFiwareConverter import ObjectFiwareConverter
from include.pubsub.genericPubSub import Publisher
from geometry_msgs.msg import Point

class AMRPublisher(Publisher):
    '''
        This class just needs to inherit Publisher.

        You can here specify your own Routine which should happen. 
        This class publishes the data (which FIROS receives).
    '''
    CB_HEADER = {'Content-Type': 'application/json'}
    CB_BASE_URL = None

    def __init__(self):
        '''
            Here you can do things that need to be initialized.

            You can use the data provided by the 'config.json' here which can be retreived via:
            """self.configData""". This is not None, as long as in config.json a key exists, which has
            the same name as the subfolder this File is in.

            You can also use some other constants in: 
            """from include.constants import Constants as CONSTANTS"""
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

    def publish(self, topic, rawMsg, msgDefinitions):
        '''
            Here goes the Routine to publish something.
            It is called automatically!
        '''
        # Create Update-JSON
        obj = {s: getattr(rawMsg, s, None) for s in rawMsg.__slots__}
        obj["id"] = topic.split("/")[1].replace('_', ':')
        try:
            attribute = topic.split("/")[2]
        except:
            return
        
        if attribute == 'status':
            value = str(obj['data'])
            data = {
                attribute:
                {
                    'type': 'Text',
                    'value': value
                }
            }
        elif attribute == 'location':
            data = {
                attribute:
                {
                    'type': 'geo:json',
                    'value': {
                        'type': 'Point',
                        'coordinates': [
                            rawMsg.position.x,
                            rawMsg.position.y
                        ]
                    }
                }
            }
        else:
            Log("WARNING", "You are trying to change the wrong attribute!")
            return
        jsonStr = json.dumps(data)

        # Update attribute on ContextBroker
        response = requests.patch(self.CB_BASE_URL + obj["id"] + "/attrs", data=jsonStr, headers=self.CB_HEADER)
    
    def unpublish(self):
        '''
            Here goes the Routine to unpublish something.
            It is called automatically!
        '''
        pass
