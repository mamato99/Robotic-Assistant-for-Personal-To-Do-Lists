#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, String, Float32MultiArray, Int16MultiArray
from ros_audio_pkg.srv import *

class TestReIdentification:
    def __init__(self,id):
        self.id = id
        self.count = 0

    def handle(self, data):

        print('\n###\ndata type:',type(data))

        try:
            identify = rospy.ServiceProxy('identify',Identify)
            save_id = rospy.ServiceProxy('save_id',SaveIdentity)

            req = IdentifyRequest(data)
            resp_identify = identify(req)

            id = resp_identify.id
            emb = resp_identify.emb
            print('id:',id)
            # print('emb:',emb)

            if self.count % 2 == 0:
                print('no audio')
                req = SaveIdentityRequest(None,emb,self.id)
                rsp = save_id(req)
                print(rsp)
            else:
                print('no emb')
                req = SaveIdentityRequest(data,None,self.id)
                rsp = save_id(req)
                print(rsp)
            # self.count += 1

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


if __name__ == '__main__':
    rospy.init_node('test_node')
    
    print('Waiting...')
    rospy.wait_for_service('identify')
    rospy.wait_for_service('save_id')
    print('OK')
    
    test = TestReIdentification('Francesco')
    rospy.Subscriber("voice_data", Int16MultiArray, test.handle) # need speech2text.launch
    rospy.spin()
    

