from include.pubsub.genericPubSub import Subscriber
from include.ros.topicHandler import RosTopicHandler


class AMRSubscriber(Subscriber):
    '''
        This class just needs to inherit Subscriber.

        You can here specify your own Routine which should happen. 
        This class subscribes to data (which FIROS should get).
    '''

    def __init__(self):
        '''
            Here you can do things that need to be initialized.

            You can use the data provided by the 'config.json' here which can be retreived via:
            """self.configData""". This is not None, as long as in config.json a key exists, which has
            the same name as the subfolder this File is in.

            You can also use some other constants in: 
            """from include.constants import Constants as CONSTANTS"""
        '''



    def subscribe(self, topicList, topicTypes, msgDefinitions):
        '''
            Here goes the Subscription Routine
            This Routine needs to make sure it that it be called somehow from an extern Signal
            E.G. In cbSubscriber  (ContextBroker)  a HttpServer is spawned in a new Thread to 
            make sure to handle incoming Context-Broker-Queries

            You need to make in this Routine sure (it is only called ONCE, but can be called multiple times), 
            that you have some mechanism, which calls the described function below!
            
            """"RosTopicHandler.publish("ROBOT_ID", "TOPIC", "CONVERTED_DATA", "DATA_STRUCT")""""
            
                ROBOT_ID is a string 
                TOPIC is a string
                CONVERTED_DATA is ROS-conform!
                DATA_STRUCT is: {"type": "STRING_OF_MESSAGE_TYPE_WITH_POINT"}
        '''
        
        pass
    
    def unsubscribe(self):
        '''
            Here goes the Routine which needs to be done to unsubscribe
            It is called automatically!
        '''
        pass
