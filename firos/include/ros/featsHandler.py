import os
import sys
import rospy
import importlib
import time
import copy

from include.logger import Log
from include.constants import Constants as C 
#from include.libLoader import LibLoader
from include.ros.rosConfigurator import RosConfigurator
#from include import confManager

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
        # Retrieves the whitelist.json. If it does not exists, it returns all topics.
        topics_regex = copy.deepcopy(RosConfigurator.systemTopics(True))

        # Retrieves the robots.json.
        topics_json = getTopicsByJson()
        if len(topics_json) == 0: 
            Log("ERROR", "The file 'topics.json' is either empty or does not exist!\n\nExiting")
            sys.exit(1)
        
        # replace first term of topic with robot ID from config file
        replace_id = C.ROBOT_ID
        new_topics = {}
        
        for key in topics_json:
            new_key = '/' + replace_id + '/' + key.split('/')[2]
            new_topics[new_key] = topics_json[key]

        # Merge both dictionaties:
        # Here topics_json overrides entries in topics_regex:
        topics_regex.update(new_topics)
        topics = topics_regex
        Log("INFO", ('\nFEATS handler initialized!'))
        print(topics)

        #self.init_pub()
        #self.init_sub()


    #def init_pub(self):

def getTopicsByJson():
    ''' Load the 'topics.json'-File 
    '''
    try:
        json_path = C.PATH + "/topics.json"
        return json.load(open(json_path))
    except:
        return {}