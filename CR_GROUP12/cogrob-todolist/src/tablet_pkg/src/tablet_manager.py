#!/usr/bin/python3

import rospy
from pepper_nodes.srv import LoadUrl, LoadUrlRequest, LoadUrlResponse
from optparse import OptionParser
from std_msgs.msg import String

from config import * 

class Handler:
    '''
    The constructor creates the service proxy object, which is able to display the desired URL on the tablet.
    '''
    def __init__(self):
        self.tablet_service = rospy.ServiceProxy("load_url", LoadUrl)

    '''
    This method calls the tablet service and sends it the URL of the web page to be displayed.
    '''
    def load_url(self, url):
        msg = LoadUrlRequest()
        msg.url = url
        resp = self.tablet_service(msg)
        rospy.loginfo(resp.ack)

class TabletManager():
    
    def __init__(self, ip, port, handler_pepper:Handler):
        self.ip = ip
        self.port = port
        self.handler_pepper = handler_pepper
        
    def start(self):
        rospy.init_node(NODE_NAME)
        rospy.Subscriber('show_data', String, callback=self._callback)
        
        print(NODE_NAME, f"RUNNING on http://{self.ip}:{self.port}")
        self.home = 'http://{}:{}/'.format(self.ip, self.port)
        print('Home:',self.home)
        handler.load_url(self.home) # Open Home page
        rospy.spin()
        
    def _callback(self, data):
        user, category = data.data.split('#')
        url = ""
        
        if user == 'home':
            url = self.home
        elif category != "":
            url = "http://{}:{}/show?user={}&category={}".format(self.ip, self.port, user, category)
        else:
            url = "http://{}:{}/show?user={}&category=all".format(self.ip, self.port, user)

        print(url)
        # print(self.handler_pepper.load_url(url))
        print(handler.load_url(url))

if __name__ == "__main__":
    NODE_NAME = "[TABLET MANAGER]"
    
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="0.0.0.0")
    parser.add_option("--port", dest="port", default=5000)
    (options, args) = parser.parse_args()
    
    try:
        handler = Handler()
        tbm = TabletManager(options.ip, options.port, handler)
        tbm.start()
    except rospy.ROSInterruptException:
        pass