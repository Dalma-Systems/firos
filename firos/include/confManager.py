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

import sys
import json
import copy
import traceback

from include.logger import Log
from include.ros.rosConfigurator import RosConfigurator
from include.constants import Constants as C


def getRobots(refresh=False):
    ''' This retrieves the current configuration from FIROS.
        Here we load the `robots.json` and the 'whitelist.json'

        In case the `whitelist.json` is Empty: We do get the current robots 
        from ROS and are adding them as subscribers (by default). In a small
        ROS-World this usually is no problem. But in an environment with many 
        robots we might send a lot of data. 
    '''
    try:
        # Retrieves the whitelist.json. If it does not exists, it returns all topics.
        topics_regex = copy.deepcopy(RosConfigurator.systemTopics(refresh))
        
        # Retrieves the robots.json.
        topics_json = getTopicsByJson()
        if len(topics_json) == 0: 
            Log("ERROR", "The file 'topics.json' is either empty or does not exist!\n\nExiting")
            sys.exit(1)
        
        # check if structure is as needed
        for key in topics_json.keys():
            if len(topics_json[key]) != 2:
                Log("ERROR", "The topic: '{}', does not have a list of length 2 (topics.json)! \n\nExiting".format(key))
                sys.exit(1)

            if not key.startswith("/"):
                Log("ERROR", "The topic: '{}', does not start with '/'  (topics.json)! \n\nExiting".format(key))
                sys.exit(1)

            if topics_json[key][1] not in ["publisher", "subscriber"]:
                Log("ERROR", "The topic: '{}', does not specify publisher or subscriber (topics.json)! \n\nExiting".format(key))
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

        return topics

    except Exception as e:
        traceback.print_exc()
        Log("ERROR", e)
        return {}


def getTopicsByJson():
    ''' Load the 'topics.json'-File 
    '''
    try:
        json_path = C.PATH + "/topics.json"
        return json.load(open(json_path))
    except:
        return {}
